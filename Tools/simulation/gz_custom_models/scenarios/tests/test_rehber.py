"""rehber.py için simülasyonsuz testler: saf fonksiyonlar ve basit nokta-kütle modeliyle kapalı döngü."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from otonom import rehber as r  # noqa: E402

A = r.Ayar()
HAT = r.Hat(36.65, 28.79, 178.0)


def durum(**kw):
    base = dict(lat=HAT.lat, lon=HAT.lon, course_deg=HAT.course_deg, airspeed=37.0, height=150.0, climb=0.0,
                roll_deg=0.0, pitch_deg=0.0, mass_kg=800.0)
    base.update(kw)
    return r.Durum(**base)


def offset(lat, lon, course_deg, along, right):
    c, s = math.cos(math.radians(course_deg)), math.sin(math.radians(course_deg))
    n, e = along * c - right * s, along * s + right * c
    return lat + n / 111320.0, lon + e / (111320.0 * math.cos(math.radians(lat)))


def test_stall_speed_matches_airframe_note():
    assert abs(r.stall_speed(1200) - 28.4) < 0.1   # airframe 4022: 1200 kg, temiz
    assert abs(r.stall_speed(800) - 23.2) < 0.1


def test_target_speeds():
    assert abs(r.target_speed(800, 'YAKALA', a=A) - 37.0) < 1e-9          # FW_AIRSPD_MIN baskın
    assert r.target_speed(800, 'DEG', a=A) == A.land_airspeed                          # FW_LND_AIRSPD baskın
    assert abs(r.target_speed(1300, 'DEG', a=A) - 1.3 * r.stall_speed(1300)) < 1e-9     # ağırda 1,3·Vs
    assert abs(r.target_speed(1300, 'YAKALA', a=A) - 1.3 * r.stall_speed(1300)) < 1e-9
    # ALÇAL'da 30 m üstü hava hızı, kapıda değiş hızı, arada doğrusal
    assert r.target_speed(800, 'ALCAL', 50.0, A) == 37.0
    assert abs(r.target_speed(800, 'ALCAL', 10.0, A) - r.target_speed(800, 'DEG', a=A)) < 1e-9
    mid = r.target_speed(800, 'ALCAL', 20.0, A)
    assert r.target_speed(800, 'DEG', a=A) < mid < 37.0


def test_line_errors_sign():
    lat, lon = offset(HAT.lat, HAT.lon, HAT.course_deg, 1000, 200)
    along, cross, terr = r.line_errors(durum(lat=lat, lon=lon, course_deg=190), HAT)
    assert abs(along - 1000) < 2 and abs(cross - 200) < 2 and abs(terr - 12) < 1e-9


def test_lateral_turns_toward_line_and_limits():
    lat, lon = offset(HAT.lat, HAT.lon, HAT.course_deg, 0, 500)        # hattın 500 m sağı
    roll, notes = r.lateral(durum(lat=lat, lon=lon), HAT, A)
    assert roll < 0                                                    # sola dön
    assert notes['wanted_course'] < HAT.course_deg
    lat, lon = offset(HAT.lat, HAT.lon, HAT.course_deg, 0, -5000)
    roll, _ = r.lateral(durum(lat=lat, lon=lon, course_deg=0.0), HAT, A)
    assert abs(roll) <= A.roll_limit_deg


def test_flare_sink_profile():
    assert r.climb_target(8.0, 0.0, 'DEG', A) == -A.touch_sink_max
    assert r.climb_target(0.8, 0.0, 'DEG', A) == -0.8 / A.flare_tau_s
    assert r.climb_target(0.3, 0.0, 'DEG', A) == -A.flare_sink_min
    assert r.climb_target(150.0, 10.0, 'ALCAL', A) == -A.sink_max
    assert r.climb_target(10.0, 150.0, 'YAKALA', A) == A.climb_max


def test_touch_pitch_is_clamped():
    pitch, _ = r.longitudinal(durum(airspeed=45.0, height=3.0, climb=-0.5), -0.5, 'DEG', r.Kontrol(), 0.02, A)
    assert A.touch_pitch_deg[0] <= pitch <= A.touch_pitch_deg[1]


def test_gates_and_abort():
    ok, why = r.capture_ok(durum(roll_deg=6.0), HAT, A)
    assert not ok and 'yatış' in why
    sp = r.target_speed(800, 'ALCAL', 10.0, A)
    ok, why = r.gate_ok(durum(height=10.0, airspeed=sp), HAT, A)
    assert ok, why
    lat, lon = offset(HAT.lat, HAT.lon, HAT.course_deg, 0, 20)
    ok, why = r.gate_ok(durum(lat=lat, lon=lon, height=10.0, airspeed=sp), HAT, A)
    assert not ok and 'yan sapma' in why
    assert r.touch_abort(durum(height=3.0, pitch_deg=4.0, airspeed=30.0), 3.0, A) is None
    assert r.touch_abort(durum(height=7.5, pitch_deg=-1.5, airspeed=34.0), 3.0, A) is None    # yasa 3 flare başında izin verir
    assert r.touch_abort(durum(height=3.0, pitch_deg=4.0, airspeed=26.0), 3.0, A) is None      # 1,05·Vs = 24,4
    assert 'hız' in r.touch_abort(durum(height=3.0, pitch_deg=4.0, airspeed=24.0), 3.0, A)
    assert r.touch_abort(durum(height=0.8, pitch_deg=-3.0, airspeed=20.0, roll_deg=9.0), 3.0, A) is None


class PointMass:
    """Basit uçak: yatış ve yunuslama birinci dereceden gecikmeli, kuvvetler modelin AdvancedLiftDrag değerlerinden."""
    THRUST_MAX, CD0, AR, EFF = 5002.0, 0.04, 5.4, 0.8
    IDLE_THRUST = 220.0

    def __init__(self, lat, lon, course, height, speed, mass=800.0):
        self.lat, self.lon, self.course, self.h, self.v, self.m = lat, lon, course, height, speed, mass
        self.roll, self.pitch, self.climb = 0.0, math.degrees(r.alpha_for(mass, speed)), 0.0

    def state(self):
        return r.Durum(self.lat, self.lon, self.course, self.v, self.h, self.climb, self.roll, self.pitch, self.m)

    def step(self, cmd, dt):
        self.roll += (cmd.roll_deg - self.roll) * dt / 0.8
        self.pitch += (cmd.pitch_deg - self.pitch) * dt / 0.5
        q = 0.5 * r.RHO * self.v ** 2
        # yunuslama ile uçuş yolu açısı arasındaki fark hücum açısıdır; kaldırma dikey dengeyi bozarsa yol eğilir
        alpha_trim = math.degrees(r.alpha_for(self.m, self.v, math.radians(self.roll)))
        gamma = math.radians(self.pitch - alpha_trim)
        cl = r.CL0 + r.CLA * math.radians(self.pitch - math.degrees(gamma))
        drag = q * r.WING_AREA_M2 * (self.CD0 + cl * cl / (math.pi * self.EFF * self.AR))
        # o2 uçuşu: gaz %5'te 35 m/s ve 2 m/s alçalmada yavaşlama yalnız 0,18 m/s² -> pervane ~220 N itki bırakıyor
        thrust = max(self.THRUST_MAX * cmd.thrust ** 2, self.IDLE_THRUST)
        self.v += ((thrust - drag) / self.m - r.G * math.sin(gamma)) * dt
        self.climb = self.v * math.sin(gamma)
        self.h += self.climb * dt
        self.course += math.degrees(r.G * math.tan(math.radians(self.roll)) / self.v) * dt
        dist = self.v * math.cos(gamma) * dt
        self.lat, self.lon = offset(self.lat, self.lon, self.course, dist, 0.0)


def fly(start_right, start_track_offset, mass=800.0, dt=0.05, limit_s=600.0):
    """Gösterideki kötü başlangıç: hattın yanından, açılı. YAKALA → ALÇAL → DEĞ; değişte ölçütleri döndürür."""
    lat, lon = offset(HAT.lat, HAT.lon, HAT.course_deg, -8000, start_right)
    p = PointMass(lat, lon, HAT.course_deg + start_track_offset, 150.0, 45.0, mass)
    k = r.Kontrol()
    phase, held, t, history = 'YAKALA', 0.0, 0.0, []
    while t < limit_s:
        d = p.state()
        if phase == 'YAKALA':
            ok, _ = r.capture_ok(d, HAT, A)
            held = held + dt if ok else 0.0
            if held >= A.capture_hold_s:
                phase = 'ALCAL'
        elif phase == 'ALCAL' and d.height <= A.gate_height_m:
            ok, why = r.gate_ok(d, HAT, A)
            assert ok, why
            gate_speed_err = abs(d.airspeed - r.target_speed(d.mass_kg, 'DEG', a=A))
            assert gate_speed_err < 1.5, f'kapıda hız {d.airspeed:.1f}'
            phase, t_deg = 'DEG', 0.0
        elif phase == 'DEG':
            t_deg += dt
            assert r.touch_abort(d, t_deg, A) is None
        cmd, k = r.command(d, HAT, phase, 150.0 if phase == 'YAKALA' else 0.0, k, dt, A)
        p.step(cmd, dt)
        t += dt
        history.append((t, p.h, p.roll, abs(r.line_errors(p.state(), HAT)[1])))
        if p.h <= 0.0:
            last20 = [x for x in history if x[0] >= t - 20.0]
            return dict(sink=-p.climb, roll=max(abs(x[2]) for x in last20), cross=max(x[3] for x in last20),
                        pitch=p.pitch, speed=p.v)
    raise AssertionError('suya değmedi')


def test_closed_loop_bad_start_reaches_criteria():
    # gösteri 16:45: hattın 560 m yanı, 11° açı
    res = fly(560.0, 11.0)
    # yasa 3 aynen: gaz rölantide, burun son metrede FL_PMIN'de; pistte ölçülen -0,67..-0,91 m/s (49)
    assert res['sink'] <= 0.9, res
    assert res['roll'] < 3.0, res
    assert res['cross'] < 15.0, res
    assert A.fl_pmin_water - 0.1 <= res['pitch'] <= A.fl_pmax_water + 0.1, res   # önce kıç (gövde basamağı)
    assert res['speed'] > A.touch_stall_factor * r.stall_speed(800), res


def test_closed_loop_aligned_and_heavy():
    for mass in (800.0, 1200.0):
        res = fly(0.0, 0.0, mass=mass)
        assert res['sink'] <= 0.9 and res['roll'] < 3.0 and res['cross'] < 15.0, (mass, res)
        assert A.fl_pmin_water - 0.1 <= res['pitch'] <= A.fl_pmax_water + 0.1, (mass, res)


def test_climb_target_is_continuous_in_speed_excess():
    # n1: hız eşiğinde ani anahtar hedef dikey hızı -2,0 ile -0,6 arasında zıplattı ve yunuslama salındı
    prev = None
    for i in range(0, 301):
        excess = -0.5 + i * 0.01
        v = r.climb_target(30.0, 0.0, 'ALCAL', A, excess)
        if prev is not None:
            assert abs(v - prev) < 0.02, (excess, v, prev)
        prev = v
    assert abs(r.climb_target(30.0, 0.0, 'ALCAL', A, 5.0) + A.decel_sink_max) < 1e-9     # hız fazlası çoksa sınır 0,6
    assert abs(r.climb_target(30.0, 0.0, 'ALCAL', A, -1.0) + A.sink_max) < 1e-9          # hız hedefte ya da altındaysa 2,0


def test_flare_law3_matches_px4_equations():
    # FixedWingModeManager.cpp 1497-1571: sink = s_td + (s0 - s_td)·f, f = (h - hold)/(h0 - hold)
    k = r.Kontrol()
    d0 = durum(height=10.0, climb=-1.8, pitch_deg=-1.0, airspeed=35.0)
    pitch, climb_sp, thr_max, k = r.flare_law3(d0, k, 0.02, A)
    assert k.flare and k.fl_start_h == 10.0 and abs(k.fl_start_sink - 1.8) < 1e-12 and k.fl_start_pitch == -1.0
    assert abs(-climb_sp - 1.8) < 1e-3                                  # flare başında başlangıç alçalması
    h = 5.5                                                              # f = (5,5 - 1) / (10 - 1) = 0,5
    _, climb_sp, _, k2 = r.flare_law3(durum(height=h, climb=-1.15, pitch_deg=2.0), k, 0.02, A)
    assert abs(-climb_sp - (0.5 + (1.8 - 0.5) * 0.5)) < 1e-9
    _, climb_sp, _, _ = r.flare_law3(durum(height=0.8, climb=-0.5, pitch_deg=4.0), k, 0.02, A)
    assert abs(-climb_sp - A.fl_sink) < 1e-12                            # hold altında temas alçalması
    # gaz üst sınırı FL_TIME içinde rölantiye (karekök rampası)
    k3 = replace_k(k, fl_t=A.fl_time_s)
    _, _, thr_max, _ = r.flare_law3(durum(height=5.0, climb=-1.0), k3, 0.0, A)
    assert abs(thr_max - A.thr_idle) < 1e-9


def replace_k(k, **kw):
    from dataclasses import replace
    return replace(k, **kw)


def test_flare_law3_pitch_limits_blend_and_antiwindup():
    k = replace_k(r.Kontrol(), flare=True, fl_start_h=10.0, fl_start_sink=2.0, fl_start_pitch=0.0)
    # hold yüksekliğinde sınırlar suya özgü FL_PMIN/PMAX: çok hızlı alçalış burnu üst sınıra taşır
    pitch, _, _, k2 = r.flare_law3(durum(height=1.0, climb=-5.0), k, 0.02, A)
    assert pitch == A.fl_pmax_water
    assert k2.fl_int == k.fl_int                                         # sınırda ve hata aynı yönde: integral birikmez
    pitch, _, _, _ = r.flare_law3(durum(height=1.0, climb=+3.0), k, 0.02, A)
    assert pitch == A.fl_pmin_water                                      # yükselirse alt sınır suya özgü (önce kıç)
    # flare başında sınırlar P_LIM_MIN/MAX
    pitch, _, _, _ = r.flare_law3(durum(height=10.0, climb=+3.0), k, 0.02, A)
    assert pitch < A.fl_pmin_water and pitch >= A.p_lim_min


def test_settings_from_px4():
    p = dict(FW_LND_AIRSPD=35.0, FW_LND_FLALT=10.0, FW_LND_FL_SINK=0.5, FW_LND_FL_HOLD=1.0, FW_LND_FL_KP=5.0,
             FW_LND_FL_KI=2.0, FW_LND_FL_TIME=1.0, FW_P_LIM_MIN=-12.0, FW_P_LIM_MAX=15.0, FW_THR_IDLE=0.0,
             FW_THR_MAX=1.0, FW_AIRSPD_MIN=37.0)
    a = r.ayar_from_px4(p)
    assert a.fl_alt_m == a.gate_height_m == 10.0 and a.fl_kp == 5.0 and a.land_airspeed == 35.0
    assert a.fl_pmin_water == A.fl_pmin_water                            # suya özgü kalır
