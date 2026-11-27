"""Su bırakma geçişi, düz çıkış ve dönüş denetimi (otonom/planlayici/birakma.py). Arazi yapay fonksiyonlarla verilir."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from otonom.planlayici import birakma as PB  # noqa: E402
from otonom.planlayici import pist as PL  # noqa: E402

TARGET = (36.6615, 28.8051)
AYAR = PB.BirakmaAyar()
FLAT = lambda lat, lon: 100.0  # noqa: E731


def _track(p, course, origin=TARGET):
    n = (p[0] - origin[0]) * PB.R_EARTH_LAT
    e = (p[1] - origin[1]) * PB.R_EARTH_LAT * math.cos(math.radians(origin[0]))
    b = math.radians(course)
    return n * math.cos(b) + e * math.sin(b), -n * math.sin(b) + e * math.cos(b)


def always(point, course, hold, actual):
    return True, 'dönüş', None


def test_climb_profile_follows_px4():
    """Bırakma geçişi noktası (hedef + 440 m) geçilene dek tırmanış yok; sonra çıkışa 440 m kala seyre doğrusal,
    uçağın tırmanışıyla sınırlı."""
    run, cruise = 265.0, 349.0
    assert PB.climb_profile(300.0, 3000.0, run, cruise, AYAR) == run
    assert PB.climb_profile(440.0, 3000.0, run, cruise, AYAR) == run
    assert abs(PB.climb_profile(3000.0 - 440.0, 3000.0, run, cruise, AYAR) - cruise) < 1e-6
    mid = PB.climb_profile(1500.0, 3000.0, run, cruise, AYAR)
    assert run < mid < cruise
    steep = PB.climb_profile(1000.0, 1000.0, run, 600.0, AYAR)   # 1000 m'de 335 m tırmanılamaz
    assert abs(steep - (run + AYAR.climb_gradient * (1000.0 - 440.0))) < 1e-6


def test_low_ground_right_after_drop_rejects_exit():
    """2026-11-27 düzeltmesi: bırakmadan 300 m sonra 216 m arazi, uçak 265 m'de (henüz tırmanmıyor): pay 50 m'den az."""
    def ground(lat, lon):
        along, _ = _track((lat, lon), 82.0)
        return 216.0 if 250.0 <= along <= 350.0 else 100.0
    ex = PB.plan_exit(TARGET, 82.0, 265.0, 349.0, ground, always, AYAR)
    assert not ex.ok and all('düz çıkış' in why for _, why in ex.rejected)


def test_exit_shortened_when_return_blocked():
    """Dönüş yalnız hedefe 2000 m'den yakın çıkışta tutuyorsa 3000 ve 2500 m elenir; gerekçe dönüşten."""
    seen = []

    def ret(point, course, hold, actual):
        seen.append((point, hold, actual))
        d = math.hypot(*_track(point, course))
        return (True, d, None) if d <= 2000.0 + 1.0 else (False, None, 'sırt')
    ex = PB.plan_exit(TARGET, 82.0, 265.0, 349.0, FLAT, ret, AYAR)
    assert ex.ok and ex.exit_m == 2000.0
    assert [m for m, _ in ex.rejected] == [3000.0, 2500.0]
    assert all(why.startswith('dönüş') for _, why in ex.rejected)
    assert all(hold == 349.0 and actual <= hold for _, hold, actual in seen)


def test_run_turns_to_a_bearing_with_a_safe_return():
    def ret(point, course, hold, actual):
        return (True, None, None) if 100.0 <= course <= 120.0 else (False, None, 'sırt')
    plan = PB.plan_run(TARGET, 82.0, 265.0, 349.0, FLAT, ret, AYAR)
    assert plan.ok and abs(plan.bearing_deg - 102.0) < 1e-6   # sıra: 82, 72, 92, 62, 102


def test_run_none_when_no_bearing_returns():
    plan = PB.plan_run(TARGET, 82.0, 265.0, 349.0, FLAT, lambda *a: (False, None, 'sırt'), AYAR)
    assert not plan.ok and len(plan.rejected) == 19


def test_ring_return_keeps_old_rule():
    home = (36.726091, 28.796766)
    fn = PB.ring_return(FLAT, home, 6.0, ayar=AYAR)
    ok, first_rel, why = fn(TARGET, 82.0, 349.0, 349.0)
    assert ok and first_rel == 300.0
    ok, _, why = PB.ring_return(lambda la, lo: 320.0, home, 6.0, ayar=AYAR)(TARGET, 82.0, 349.0, 349.0)
    assert not ok and 'halka' in why


def test_exit_with_runway_return_avoids_ridge():
    """Çıkış sonu sağında sırt: pist dönüşü sola döner (yay noktaları), sağa dönen yol hiç seçilmez."""
    N = (36.726091, 28.796766)
    S = (36.701195, 28.788088)
    dirs = [PL.PistYonu('19', N, 195.68, -1.54), PL.PistYonu('01', S, 15.68, -2.25)]
    ayar = PL.PistAyar(airspeed=36.7)

    def ground(lat, lon):
        along, right = _track((lat, lon), 82.0)
        return 400.0 if 2000.0 <= along <= 5000.0 and 600.0 <= right <= 2500.0 else 5.0

    def ret(point, course, hold, actual):
        plan = PL.plan_runway_return(point, hold - 6.0, course, dirs, 6.0, ground, ayar, actual_rel_m=actual - 6.0, early_m=440.0)
        return (True, plan, None) if plan.ok else (False, None, str(plan.rejected[:1]))
    ex = PB.plan_exit(TARGET, 82.0, 265.0, 349.0, ground, ret, AYAR)
    assert ex.ok and ex.ret.turn == 'sola'
    assert all(_track(p, 82.0)[1] < 600.0 for _, p, _ in ex.ret.transition)


def test_short_exit_point_altitude_is_reachable():
    """1000 m çıkışta uçak seyre (349 m) yetişemez: noktanın irtifası yetişebildiği (PX4 irtifası tutmayan noktada döner)."""
    ayar = PB.BirakmaAyar(exit_runs_m=(1000.0,))
    ex = PB.plan_exit(TARGET, 82.0, 265.0, 349.0, FLAT, always, ayar)
    assert ex.ok
    assert abs(ex.point_alt_amsl - (265.0 + ayar.climb_gradient * (1000.0 - ayar.accept_m))) < 1e-6
    assert ex.alt_amsl <= ex.point_alt_amsl
    long = PB.plan_exit(TARGET, 82.0, 265.0, 349.0, FLAT, always, PB.BirakmaAyar())
    assert long.exit_m == 3000.0 and long.point_alt_amsl == 349.0
