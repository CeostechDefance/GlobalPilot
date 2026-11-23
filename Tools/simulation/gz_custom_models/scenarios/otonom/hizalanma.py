"""Suya hizalanma durum makinesi (offboard): YAKALA → ALÇAL → su+10 m KAPISI → DEĞ → DEVİR; kapı ya da DEĞ bozulursa PAS GEÇ.

Görev (AUTO.MISSION) uçağı Yaklaşma noktasına getirir; Yaklaşma kabul edilince (görev adımı 'Geçiş başı'na geçer) offboard'a
girilir. Suya değip 2 s kararlı kalınca PX4 görevine 'Geçiş sonu' adımından devredilir; su alma ve gaz mantığı water_leg'de sürer.
PAS GEÇ: kanatlar düz, tam gaz, su+150 m'ye tırmanış, görev başına (Yaklaşma) dönüş, yeniden deneme (en çok max_attempts).
Kararlar ve gerekçeleri kayda yazılır.
"""
import math
import time
from dataclasses import dataclass

from . import rehber as r
from .baglanti import PX4_AUTO_LOITER, PX4_AUTO_MISSION, PX4_MAIN_AUTO, PX4_MAIN_OFFBOARD
from .offboard import Offboard


@dataclass(frozen=True)
class Plan:
    hat: r.Hat                 # geçiş hattı: başlangıç noktası geçiş başı, yön geçiş yönü
    approach_seq: int          # 'Yaklaşma' görev adımı
    run_start_seq: int         # 'Geçiş başı'
    handover_seq: int          # 'Geçiş sonu'
    capture_height_m: float = 150.0
    late_touch_m: float = 800.0       # geçiş başından bu kadar sonra hâlâ değilmediyse PAS GEÇ
    max_attempts: int = 3
    touch_stable_s: float = 2.0
    loop_dt: float = 0.05
    timeout_s: float = 900.0
    # Devirde PX4 görevi seyir hızına dönmek için tam gaz ister (gösteri g1: devirde gaz %90, hız 27,3 -> 30 m/s,
    # pencere hız şartını aştı). Devirden önce gaz sınırı bu değere indirilir; geri alma water_leg'in işidir.
    handover_throttle_param: str = None
    handover_throttle: float = None


class Hizalanma:
    def __init__(self, baglanti, height_sensor, mass_source, plan, kayit, ayar=r.Ayar(), log=print, default_mass_kg=800.0):
        self.b, self.hs, self.ms, self.p, self.k, self.a, self.log = baglanti, height_sensor, mass_source, plan, kayit, ayar, log
        self.default_mass = default_mass_kg
        self.attempts = 0
        self.touch = None          # değiş anı ölçümleri
        self.history = []          # (t, yatış, yan sapma) son 20 s ölçütü için

    # --- ölçümler
    def durum(self):
        s = self.b.state
        h, _ = self.hs.read()
        mass = self.ms.read()
        if mass is None:
            mass = self.default_mass
        return r.Durum(lat=s['lat'], lon=s['lon'], course_deg=s['course'], airspeed=s['airspeed'],
                       height=h if h is not None else float('nan'), climb=s['climb'], roll_deg=math.degrees(s['roll']),
                       pitch_deg=math.degrees(s['pitch']), mass_kg=mass)

    def descent_distance(self, d):
        """Şu yükseklikten suya değene dek gereken yol (m): yavaşlama bölgesine kadar 2 m/s, yavaşlama bölgesinde
        decel_sink_max (hız fazlası varken), kapıdan sonra DEĞ 0,5 m/s; hava hızlarıyla. Kötümser tahmin."""
        a = self.a
        air = r.target_speed(d.mass_kg, 'YAKALA', a=a)
        touch = r.target_speed(d.mass_kg, 'DEG', a=a)
        h = d.height
        upper = max(h - a.decel_start_height_m, 0.0)
        middle = max(min(h, a.decel_start_height_m) - a.gate_height_m, 0.0)
        lower = min(h, a.gate_height_m)
        return upper / a.sink_max * air + middle / a.decel_sink_max * (air + touch) / 2 + lower / a.touch_sink_max * touch

    # --- ana döngü
    def run(self):
        """Dönüş: dict(ok, reason, attempts, touch, last20_roll_max, last20_cross_max)."""
        started = time.time()
        while self.attempts < self.p.max_attempts and time.time() - started < self.p.timeout_s:
            if not self._wait_approach(started):
                return self._result(False, 'görev Yaklaşma adımını geçmedi')
            self.attempts += 1
            self.k.karar(f'deneme {self.attempts}: offboard YAKALA')
            outcome, why = self._attempt()
            if outcome == 'DEVIR':
                return self._result(True, None)
            self.k.karar(f'PAS GEÇ: {why}', deneme=self.attempts)
            self.log(f'PAS GEÇ ({self.attempts}/{self.p.max_attempts}): {why}')
            if outcome == 'HATA':
                return self._result(False, why)
        return self._result(False, f'{self.attempts} denemede suya hizalanılamadı')

    def _wait_approach(self, started):
        """Görev Yaklaşma'yı kabul edip Geçiş başı'na geçene dek bekler (AUTO.MISSION).
        PAS GEÇ'ten sonra eski adım numarası bir süre daha gelebilir: önce Yaklaşma adımı görülmeli."""
        if self.attempts > 0:
            while self.b.state['seq'] != self.p.approach_seq and time.time() - started < self.p.timeout_s:
                time.sleep(0.2)
        while time.time() - started < self.p.timeout_s:
            if self.b.state['seq'] == self.p.run_start_seq and self.b.main_mode() == (PX4_MAIN_AUTO, PX4_AUTO_MISSION):
                return True
            time.sleep(0.2)
        return False

    def _attempt(self):
        d = self.durum()
        k = r.Kontrol()
        cmd, k = r.command(d, self.p.hat, 'YAKALA', self.p.capture_height_m, k, self.p.loop_dt, self.a)
        o = Offboard(self.b, self.log)
        if not o.start(cmd.roll_deg, cmd.pitch_deg, cmd.thrust):
            o.stop()
            self.b.set_mode(PX4_MAIN_AUTO, PX4_AUTO_MISSION)
            return 'HATA', 'offboard kipine geçilemedi'
        phase, phase_t, held, wet_since = 'YAKALA', time.time(), 0.0, None
        self.history = []
        last = time.time()
        while True:
            time.sleep(max(0.0, self.p.loop_dt - (time.time() - last)))
            now = time.time()
            dt, last = now - last, now
            if o.watchdog is not None:
                return self._go_around(o, k, f'bekçi: {o.watchdog}')
            if self.b.main_mode()[0] != PX4_MAIN_OFFBOARD:
                return self._go_around(o, k, f'PX4 offboard dışına çıktı: kip {self.b.main_mode()}')
            d = self.durum()
            if math.isnan(d.height):
                return self._go_around(o, k, 'yükseklik ölçümü yok')
            along, cross, terr = r.line_errors(d, self.p.hat)
            self.history.append((now, d.roll_deg, cross))
            self.history = [x for x in self.history if x[0] >= now - 20.0]

            if phase == 'YAKALA':
                ok, why = r.capture_ok(d, self.p.hat, self.a)
                held = held + dt if ok else 0.0
                if along + self.descent_distance(d) > self.p.late_touch_m:
                    return self._go_around(o, k, f'hizalanmaya mesafe kalmadı: geçiş başına {-along:.0f} m, gereken {self.descent_distance(d):.0f} m ({why})')
                if held >= self.a.capture_hold_s:
                    phase, phase_t = 'ALCAL', now
                    self.k.karar(f'ALÇAL: {why}', along=round(along), cross=round(cross, 1))
                    self.log(f'ALÇAL: {why}, geçiş başına {-along:.0f} m')
            elif phase == 'ALCAL':
                if d.height <= self.a.gate_height_m:
                    ok, why = r.gate_ok(d, self.p.hat, self.a, k)
                    if not ok:
                        return self._go_around(o, k, f'KAPI geçmedi: {why}')
                    phase, phase_t = 'DEG', now
                    self.k.karar(f'KAPI geçti, DEĞ: {why}')
                    self.log(f'KAPI geçti: {why}')
            elif phase == 'DEG':
                why = r.touch_abort(d, now - phase_t, self.a, k)
                if why:
                    return self._go_around(o, k, f'DEĞ: {why}')
                if along > self.p.late_touch_m:
                    return self._go_around(o, k, f'geç kalındı: geçiş başını {along:.0f} m geçti, değmedi')
                if d.height <= 0.0:
                    if wet_since is None:
                        wet_since = now
                        last20 = self.history
                        self.touch = dict(speed=round(d.airspeed, 2), pitch=round(d.pitch_deg, 2), vz=round(d.climb, 2),
                                          roll=round(d.roll_deg, 2), cross=round(cross, 1), along=round(along),
                                          last20_roll_max=round(max(abs(x[1]) for x in last20), 2),
                                          last20_cross_max=round(max(abs(x[2]) for x in last20), 1), mass=round(d.mass_kg))
                        self.k.karar('SUYA DEĞDİ', **self.touch)
                        self.log(f'SUYA DEĞDİ: {self.touch}')
                    elif now - wet_since >= self.p.touch_stable_s:
                        if self.p.handover_throttle_param and self.p.handover_throttle is not None:
                            self.b.set_param(self.p.handover_throttle_param, self.p.handover_throttle)
                            got = self.b.read_param(self.p.handover_throttle_param, timeout=1.0)
                            self.k.karar(f'devir öncesi {self.p.handover_throttle_param} {self.p.handover_throttle:g}, okunan {got}')
                        self.b.set_current(self.p.handover_seq)
                        ok = o.exit_to(PX4_MAIN_AUTO, PX4_AUTO_MISSION)
                        self.k.karar(f'DEVİR: Geçiş sonu (adım {self.p.handover_seq}), AUTO.MISSION {ok}')
                        self.log(f'DEVİR: AUTO.MISSION {ok}')
                        return ('DEVIR', None) if ok else ('HATA', 'devirde AUTO.MISSION görülmedi')
                else:
                    wet_since = None

            target = self.p.capture_height_m if phase == 'YAKALA' else 0.0
            cmd, k = r.command(d, self.p.hat, phase, target, k, dt, self.a)
            o.set(cmd.roll_deg, cmd.pitch_deg, cmd.thrust)
            self.k.sample(asama=phase, h=round(d.height, 2), along=round(along), cross=round(cross, 1), terr=round(terr, 1),
                          roll=round(d.roll_deg, 2), pitch=round(d.pitch_deg, 2), vz=round(d.climb, 2), v=round(d.airspeed, 1),
                          m=round(d.mass_kg), roll_sp=round(cmd.roll_deg, 2), pitch_sp=round(cmd.pitch_deg, 2),
                          thr=round(cmd.thrust, 3), v_sp=round(cmd.target_speed, 1), vz_sp=round(cmd.target_climb, 2))

    def _go_around(self, o, k, why):
        """PAS GEÇ: kanatlar düz, tam gaz, tırmanış; su+30 m'yi geçince görev başına (Yaklaşma) dönülür."""
        if o.watchdog == 'çıkış':
            # bekçi zaten AUTO.LOITER'a çıktı; tırmanışı görev üstlensin
            self.b.set_current(self.p.approach_seq)
            self.b.set_mode(PX4_MAIN_AUTO, PX4_AUTO_MISSION)
            return 'PAS', why
        end = time.time() + 30.0
        while time.time() < end:
            d = self.durum()
            cmd, k = r.command(d, self.p.hat, 'PAS', 0.0, k, self.p.loop_dt, self.a)
            o.set(cmd.roll_deg, cmd.pitch_deg, cmd.thrust)
            if not math.isnan(d.height) and d.height > 30.0:
                break
            time.sleep(self.p.loop_dt)
        self.b.set_current(self.p.approach_seq)
        ok = o.exit_to(PX4_MAIN_AUTO, PX4_AUTO_MISSION)
        if not ok:
            self.b.set_mode(PX4_MAIN_AUTO, PX4_AUTO_LOITER)
            return 'HATA', f'{why}; PAS GEÇ sonrası AUTO.MISSION görülmedi'
        return 'PAS', why

    def _result(self, ok, reason):
        return dict(ok=ok, reason=reason, attempts=self.attempts, touch=self.touch)
