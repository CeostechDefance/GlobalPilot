"""Offboard komut gönderici: 50 Hz SET_ATTITUDE_TARGET (yatış, yunuslama, gaz) ve bekçi.

Gönderim ayrı iş parçacığında, ana döngüyü beklemez; son komutu yineler.
Bekçi: rehber komutu 0,3 s yenilemezse kanatlar düz, güvenli yunuslama ve gaz; 1 s yenilemezse offboard'dan çıkılır
(AUTO.LOITER) ve gönderim durur. PX4'ün kendi offboard kaybı kuralları (COM_OF_LOSS_T, COM_OBL_RC_ACT) her zaman üstte.
"""
import math
import threading
import time

from pymavlink import mavutil

from .baglanti import PX4_AUTO_LOITER, PX4_MAIN_AUTO, PX4_MAIN_OFFBOARD

RATE_HZ = 50.0
STALE_SAFE_S = 0.3
STALE_EXIT_S = 1.0
SAFE_PITCH_DEG = 5.0
SAFE_THRUST = 0.8
TYPE_MASK_ATTITUDE_THRUST = 0b00000111   # gövde hızları yok sayılır
M = mavutil.mavlink


def quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


class Offboard:
    def __init__(self, baglanti, log=print):
        self.b = baglanti
        self.log = log
        self._lock = threading.Lock()
        self._cmd = None            # (yatış°, yunuslama°, gaz)
        self._cmd_time = 0.0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='offboard-gonderim', daemon=True)
        self.watchdog = None        # None, 'güvenli', 'çıkış'
        self._exiting = False       # çıkış sırasında son komut yinelenir, bekçi kipi değiştirmez

    def set(self, roll_deg, pitch_deg, thrust):
        with self._lock:
            self._cmd = (roll_deg, pitch_deg, thrust)
            self._cmd_time = time.time()

    def start(self, roll_deg, pitch_deg, thrust, stream_s=1.0, timeout=5.0):
        """Akışı başlatır, PX4 offboard'a geçmeden akış ister; sonra kipi ister. Kip görüldüyse True."""
        self.set(roll_deg, pitch_deg, thrust)
        self._thread.start()
        # Giriş sırasında komut bu ilk komuttur: bekçi giriş beklemesini kesinti saymasın diye yenilenir
        end = time.time() + stream_s
        while time.time() < end:
            self.set(roll_deg, pitch_deg, thrust)
            time.sleep(0.05)
        end = time.time() + timeout
        while time.time() < end:
            self.b.command(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_MAIN_OFFBOARD, 0)
            stop = time.time() + 0.5
            while time.time() < stop:
                self.set(roll_deg, pitch_deg, thrust)
                if self.b.main_mode()[0] == PX4_MAIN_OFFBOARD:
                    return True
                time.sleep(0.05)
        return False

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)

    def exit_to(self, main, sub, timeout=5.0):
        """Belirtilen kipe geçer, sonra gönderimi durdurur (geçiş sırasında akış sürer, bekçi devre dışı)."""
        self._exiting = True
        ok = self.b.set_mode(main, sub, timeout)
        self.stop()
        return ok

    def _run(self):
        period = 1.0 / RATE_HZ
        next_t = time.time()
        while not self._stop.is_set():
            with self._lock:
                cmd, age = self._cmd, time.time() - self._cmd_time
            if cmd is not None and self._exiting:
                self._send(*cmd)
            elif cmd is not None:
                if age > STALE_EXIT_S:
                    self._event('çıkış', age)
                    self.b.set_mode(PX4_MAIN_AUTO, PX4_AUTO_LOITER, timeout=2.0)
                    self._stop.set()
                    break
                if age > STALE_SAFE_S:
                    self._event('güvenli', age)
                    cmd = (0.0, SAFE_PITCH_DEG, SAFE_THRUST)
                elif self.watchdog == 'güvenli':
                    self._event(None, age)
                self._send(*cmd)
            next_t += period
            time.sleep(max(0.0, next_t - time.time()))

    def _event(self, kind, age):
        if kind != self.watchdog:
            self.watchdog = kind
            self.log(f'bekçi: {kind or "normal"} (komut yaşı {age:.2f} s)')

    def _send(self, roll_deg, pitch_deg, thrust):
        yaw = self.b.state['yaw']   # sabit kanatlıda yön dönüşle gelir; yalnız biçim için
        q = quaternion(math.radians(roll_deg), math.radians(pitch_deg), yaw)
        thrust = max(0.0, min(1.0, thrust))
        self.b.send(lambda mav: mav.set_attitude_target_send(int(time.time() * 1000) & 0xFFFFFFFF, self.b.link.target_system,
                                                            self.b.link.target_component, TYPE_MASK_ATTITUDE_THRUST, q,
                                                            0.0, 0.0, 0.0, thrust))
