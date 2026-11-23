"""MAVLink bağlantısı: tek okuma iş parçacığı, kilitli durum, gönderimler tek kilitten.

Bu nesne çalışırken bağlantıdan başka hiçbir yer recv_match çağırmaz. Görev yükleme ve parametre okuma da yanıtları
okuma iş parçacığının kuyruğundan alır. fire_attack.Vehicle ve water_leg.StandaloneVehicle ile aynı arayüzü sunar
(state, update, upload, command, link), böylece run_water_leg değişmeden kullanabilir.
"""
import math
import queue
import threading
import time

from pymavlink import mavutil

M = mavutil.mavlink
PX4_MAIN_AUTO, PX4_MAIN_OFFBOARD = 4, 6
PX4_AUTO_LOITER, PX4_AUTO_MISSION = 3, 4


class Baglanti:
    REPLY_TYPES = ('MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK', 'PARAM_VALUE', 'COMMAND_ACK')

    def __init__(self, link, log=print, statustext=True, dinleyici=None):
        self.link = link
        self.dinleyici = dinleyici   # isteğe bağlı: dinleyici(msg, zaman), bağlı aracın her mesajı için (okuma iş parçacığında)
        self.log = log
        self.statustext = statustext
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._replies = queue.Queue()
        self._stop = threading.Event()
        self._state = dict(armed=False, mode=0, lat=0.0, lon=0.0, rel=0.0, alt=0.0, airspeed=0.0, groundspeed=0.0,
                           throttle=0, landed=0, seq=-1, roll=0.0, pitch=0.0, yaw=0.0, vn=0.0, ve=0.0, vd=0.0,
                           course=0.0, climb=0.0, t_attitude=0.0, t_position=0.0)
        self._thread = threading.Thread(target=self._read, name='mavlink-okuma', daemon=True)
        self._last_heartbeat = 0.0

    # --- yaşam döngüsü
    def start(self):
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)

    # --- okuma (tek yer)
    def _read(self):
        while not self._stop.is_set():
            now = time.time()
            if now - self._last_heartbeat > 1.0:
                self._last_heartbeat = now
                self.send(lambda mav: mav.heartbeat_send(M.MAV_TYPE_GCS, M.MAV_AUTOPILOT_INVALID, 0, 0, 0))
            try:
                msg = self.link.recv_match(blocking=True, timeout=0.05)
            except Exception as exc:  # bağlantı hatası okuma iş parçacığını öldürmesin
                self.log(f'MAVLink okuma hatası: {exc}')
                time.sleep(0.1)
                continue
            if msg is None:
                continue
            self._handle(msg, time.time())

    def _handle(self, msg, now):
        kind = msg.get_type()
        # Yalnız bağlı araç: aynı porta başka PX4 örneği yayın yapabilir (ör. -i >= 10 hepsi 14549'a)
        if kind != 'BAD_DATA' and msg.get_srcSystem() != self.link.target_system:
            return
        if kind in self.REPLY_TYPES:
            self._replies.put(msg)
        if self.dinleyici is not None:
            try:
                self.dinleyici(msg, now)
            except Exception as exc:  # dinleyici hatası okumayı durdurmasın
                self.log(f'dinleyici hatası: {exc!r}')
        with self._lock:
            s = self._state
            if kind == 'GLOBAL_POSITION_INT':
                vn, ve, vd = msg.vx / 100.0, msg.vy / 100.0, msg.vz / 100.0
                s.update(lat=msg.lat / 1e7, lon=msg.lon / 1e7, rel=msg.relative_alt / 1e3, alt=msg.alt / 1e3,
                         vn=vn, ve=ve, vd=vd, climb=-vd, groundspeed=math.hypot(vn, ve), t_position=now)
                if math.hypot(vn, ve) > 1.0:
                    s['course'] = math.degrees(math.atan2(ve, vn)) % 360.0
            elif kind == 'ATTITUDE':
                s.update(roll=msg.roll, pitch=msg.pitch, yaw=msg.yaw, t_attitude=now)
            elif kind == 'VFR_HUD':
                s.update(airspeed=msg.airspeed, throttle=msg.throttle)
            elif kind == 'EXTENDED_SYS_STATE':
                s.update(landed=msg.landed_state)
            elif kind == 'MISSION_CURRENT':
                s.update(seq=msg.seq)
            elif kind == 'HEARTBEAT' and msg.get_srcSystem() == self.link.target_system and msg.get_srcComponent() == 1:
                s.update(armed=bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED), mode=msg.custom_mode)
        if kind == 'STATUSTEXT' and self.statustext and msg.severity <= M.MAV_SEVERITY_WARNING and 'denied' not in msg.text:
            self.log(f'  PX4: {msg.text.strip()}')

    # --- durum
    @property
    def state(self):
        """Anlık kopya. (Eski arayüz state[...] okur; yazmaz.)"""
        with self._lock:
            return dict(self._state)

    def main_mode(self):
        mode = self.state['mode']
        return (mode >> 16) & 0xFF, (mode >> 24) & 0xFF

    def update(self, seconds):
        """Eski arayüz: okuma iş parçacığı zaten okuyor, burada yalnız beklenir."""
        time.sleep(max(seconds, 0.0))

    # --- gönderim (tek kilit)
    def send(self, fn):
        with self._send_lock:
            fn(self.link.mav)

    def command(self, command, *params):
        params = list(params) + [0.0] * (7 - len(params))
        self.send(lambda mav: mav.command_long_send(self.link.target_system, self.link.target_component, command, 0, *params))

    def set_mode(self, main, sub=0, timeout=5.0):
        """Kipi ister ve heartbeat'te görene kadar bekler. Görüldüyse True."""
        end = time.time() + timeout
        while time.time() < end:
            self.command(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main, sub)
            stop = time.time() + 0.5
            while time.time() < stop:
                got_main, got_sub = self.main_mode()
                if got_main == main and (main != PX4_MAIN_AUTO or got_sub == sub):
                    return True
                time.sleep(0.05)
        return False

    def _wait_reply(self, accept, timeout):
        end = time.time() + timeout
        while time.time() < end:
            try:
                msg = self._replies.get(timeout=max(0.0, min(0.5, end - time.time())))
            except queue.Empty:
                continue
            if accept(msg):
                return msg
        return None

    def _drain(self):
        while True:
            try:
                self._replies.get_nowait()
            except queue.Empty:
                return

    def read_param(self, name, timeout=3.0):
        """PARAM_VALUE değeri ya da None. name bytes ya da str."""
        text = name.decode() if isinstance(name, bytes) else name
        self.send(lambda mav: mav.param_request_read_send(self.link.target_system, self.link.target_component,
                                                          text.encode(), -1))
        msg = self._wait_reply(lambda m: m.get_type() == 'PARAM_VALUE' and m.param_id == text, timeout)
        return None if msg is None else float(msg.param_value)

    def set_param(self, name, value):
        text = name.decode() if isinstance(name, bytes) else name
        self.send(lambda mav: mav.param_set_send(self.link.target_system, self.link.target_component, text.encode(),
                                                 float(value), M.MAV_PARAM_TYPE_REAL32))

    def upload(self, items, timeout=20.0):
        """Görev yükler. Kabul edilmezse RuntimeError.
        item: dict(cmd, lat, lon, alt (eve göre), radius -> param2 (LOITER_TO_ALT: eksi = saat tersi),
                   param1, param3, param4 (LOITER_TO_ALT: 1 = teğet çıkış), frame); verilmeyenler PX4 varsayılanı."""
        self._drain()
        self.send(lambda mav: mav.mission_count_send(self.link.target_system, self.link.target_component, len(items), 0))
        end = time.time() + timeout
        while time.time() < end:
            msg = self._wait_reply(lambda m: m.get_type() in ('MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK'),
                                   end - time.time())
            if msg is None:
                break
            if msg.get_type() == 'MISSION_ACK':
                if msg.type != M.MAV_MISSION_ACCEPTED:
                    raise RuntimeError(f'görev reddedildi (kod {msg.type})')
                return
            item = items[msg.seq]
            self.send(lambda mav, s=msg.seq, it=item: mav.mission_item_int_send(
                self.link.target_system, self.link.target_component, s, it.get('frame', M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT),
                it['cmd'], int(s == 0), 1, it.get('param1', 0), it.get('radius', 0), it.get('param3', 0), it.get('param4', float('nan')),
                int(it['lat'] * 1e7), int(it['lon'] * 1e7), it['alt'], 0))
        raise RuntimeError('görev yüklenemedi: PX4 yanıt vermedi')

    def set_current(self, seq):
        self.send(lambda mav: mav.mission_set_current_send(self.link.target_system, self.link.target_component, seq))
