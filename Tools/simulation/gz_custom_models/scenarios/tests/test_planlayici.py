"""Planlayıcı (Dubins + arazi) için simülasyonsuz testler. Arazi yapay fonksiyonlarla verilir."""
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from otonom.planlayici import dubins as D  # noqa: E402
from otonom.planlayici import rota as R  # noqa: E402

LAT0, LON0 = 36.65, 28.79
FRAME = R.YerelCerceve(LAT0, LON0)


def at(x, y):
    return FRAME.latlon(x, y)


def sea(lat, lon):
    return 0.0


def hill(cx, cy, height, radius):
    def fn(lat, lon):
        x, y = FRAME.xy(lat, lon)
        return height * max(0.0, 1.0 - math.hypot(x - cx, y - cy) / radius)
    return fn


def test_turn_radius():
    assert abs(D.turn_radius(45.0) - 443.0) < 1.0
    assert abs(D.turn_radius(37.0) - 299.5) < 1.0


def test_all_candidates_reach_goal():
    random.seed(7)
    for _ in range(500):
        s = D.Poz(0, 0, random.uniform(0, 360))
        e = D.Poz(random.uniform(-4000, 4000), random.uniform(-4000, 4000), random.uniform(0, 360))
        for p in D.all_paths(s, e, 500.0):
            _, x, y, c, _ = p.sample(5.0)[-1]
            assert math.hypot(x - e.x, y - e.y) < 1.0, p.word
            assert abs((c - e.course_deg + 180) % 360 - 180) < 0.5, p.word


def test_straight_and_u_turn():
    straight = D.shortest(D.Poz(0, 0, 0), D.Poz(0, 3000, 0), 443.0)
    assert straight.word in ('LSL', 'RSR') and abs(straight.length - 3000) < 1
    back = D.shortest(D.Poz(0, 0, 0), D.Poz(0, 0, 180), 443.0)
    assert back.length >= math.pi * 443.0 - 1   # yerinde dönüş yok: en az yarım çember


def test_leg_over_sea_ok_and_points():
    a = R.PlanAyar()
    start = R.Poz3(*at(0, 0), 0.0, 300.0)
    end = R.Poz3(*at(3000, 6000), 90.0, 300.0)
    leg = R.plan_leg(start, end, sea, a)
    assert leg.ok, leg.reason
    assert leg.min_clearance >= a.clearance_m
    last = leg.items[-1]
    assert abs(last['lat'] - end.lat) < 1e-5 and abs(last['lon'] - end.lon) < 1e-5
    # uzun yaylar ara noktalarla bölünür
    sweep = sum(p.length for p in leg.path.parts if p.kind != 'S') / leg.path.radius
    assert len(leg.items) >= math.ceil(math.degrees(sweep) / a.arc_point_deg)


def test_hill_on_straight_line_forces_other_route_or_fails():
    a = R.PlanAyar()
    start = R.Poz3(*at(0, 0), 0.0, 300.0)
    end = R.Poz3(*at(0, 8000), 0.0, 300.0)
    terrain = hill(0, 4000, 250.0, 600.0)      # tam yolun üstünde, 250 m tepe; 300-150 = 150 m pay yetmez
    leg = R.plan_leg(start, end, terrain, a)
    if leg.ok:
        assert all(alt - h >= a.clearance_m for _, _, _, _, alt, h in leg.samples)
        assert leg.path.word not in ('LSL', 'RSR') or leg.path.length > 8000 + 1
    else:
        assert 'arazi payı' in leg.reason


def test_corridor_catches_side_hill():
    a = R.PlanAyar()
    start = R.Poz3(*at(0, 0), 0.0, 300.0)
    end = R.Poz3(*at(0, 8000), 0.0, 300.0)
    # yolun 170 m yanında dar (60 m) ve yüksek tepe: yolun kendisi temiz, koridor (200 m, 50 m aralık) yakalar
    terrain = hill(170, 4000, 260.0, 60.0)
    assert terrain(*at(0, 4000)) == 0.0
    leg = R.plan_leg(start, end, terrain, a)
    assert not (leg.ok and leg.path.length < 8000 + 1), 'yan tepe kaçırıldı'


def test_altitude_change_limits():
    a = R.PlanAyar()
    start = R.Poz3(*at(0, 0), 0.0, 1000.0)
    end = R.Poz3(*at(0, 3000), 0.0, 160.0)    # 840 m alçalış: 2/45 eğimle 18,9 km ister
    leg = R.plan_leg(start, end, sea, a)
    assert not leg.ok and 'irtifa değişimi' in leg.reason
    end = R.Poz3(*at(0, 3000), 0.0, 1100.0)   # 100 m tırmanış: 1,3 km yeter
    leg = R.plan_leg(start, end, sea, a)
    assert leg.ok, leg.reason
    alts = [alt for _, _, _, _, alt, _ in leg.samples]
    assert alts[0] >= 1000.0 - 1e-6 and abs(alts[-1] - 1100.0) < 1e-6
    assert all(b >= a_ - 1e-6 for a_, b in zip(alts, alts[1:]))   # tırmanışta düşüş yok


def test_descent_happens_late():
    a = R.PlanAyar()
    start = R.Poz3(*at(0, 0), 0.0, 400.0)
    end = R.Poz3(*at(0, 8000), 0.0, 300.0)    # 100 m alçalış 2,25 km ister: ilk 5,7 km'de irtifa korunur
    leg = R.plan_leg(start, end, sea, a)
    assert leg.ok
    early = [alt for s, _, _, _, alt, _ in leg.samples if s < 5000]
    assert min(early) >= 400.0 - 1e-6


def test_loiter_at_end_must_be_clear():
    a = R.PlanAyar()
    start = R.Poz3(*at(0, 0), 0.0, 300.0)
    end = R.Poz3(*at(0, 8000), 0.0, 300.0)
    terrain = hill(600, 8000, 300.0, 300.0)   # bitişin 600 m yanında tepe: çember (500 m) ve çevresi denetlenir
    leg = R.plan_leg(start, end, terrain, a)
    assert not leg.ok and 'bekleme çemberi' in leg.reason
    # çember denetimi kapalıyken yalnız koridor kalır: 600 m yandaki tepe 200 m koridora girmez (500 m koridor yakalar)
    leg = R.plan_leg(start, end, terrain, R.PlanAyar(corridor_m=200.0), require_loiter=False)
    assert leg.ok, leg.reason
    assert not R.plan_leg(start, end, terrain, a, require_loiter=False).ok


def test_missing_terrain_is_not_ok():
    a = R.PlanAyar()
    start = R.Poz3(*at(0, 0), 0.0, 300.0)
    end = R.Poz3(*at(0, 5000), 0.0, 300.0)
    leg = R.plan_leg(start, end, lambda lat, lon: None, a)
    assert not leg.ok


def test_cache_reuses_reads():
    calls = []
    cached = R.OnbellekliArazi(lambda lat, lon: calls.append(1) or 0.0)
    for _ in range(5):
        cached(LAT0, LON0)
    assert len(calls) == 1


def coast(y_shore):
    """y > y_shore kara (300 m), altı deniz."""
    return lambda lat, lon: 300.0 if FRAME.xy(lat, lon)[1] > y_shore else 0.0


def test_loiter_descent_puts_circle_over_water_and_tangent_on_line():
    a = R.PlanAyar()
    terrain = coast(200.0)                       # hat noktası kıyıda, hat güneye (180°) denize gidiyor
    start = R.Poz3(*at(3000, -1000), 0.0, 530.0)
    line = R.Poz3(*at(0, 0), 180.0, 151.0)
    c = R.plan_loiter_descent(start, line, terrain, a, next_along_m=5000.0)
    assert c.ok, c.reason
    cx, cy = FRAME.xy(*c.center)
    tx, ty = FRAME.xy(*c.tangent)
    assert abs(tx) < 1.0 and ty <= 0.0             # teğet hattın üstünde ve kıyının deniz tarafında
    assert abs(abs(cx) - c.radius_m) < 1.0         # merkez hattan yarıçap kadar yanda
    assert cy + 1.7 * c.radius_m <= 200.0          # çember ve denetim çevresi tamamen denizde
    assert c.items[-2]['kind'] == 'loiter_to_alt' and c.items[-2]['exit_tangent']
    nx, ny = FRAME.xy(c.items[-1]['lat'], c.items[-1]['lon'])
    assert abs(nx) < 1.0 and abs(ny + 5000.0) < 1.0   # sonraki nokta hattın üstünde: çıkış teğeti hattın kendisi
    assert (c.items[-2]['radius'] < 0) == c.counter_clockwise


def test_loiter_descent_keeps_straight_before_next_point():
    a = R.PlanAyar()
    start = R.Poz3(*at(3000, -1000), 0.0, 530.0)
    line = R.Poz3(*at(0, 0), 180.0, 151.0)
    c = R.plan_loiter_descent(start, line, sea, a, next_along_m=2000.0, min_straight_m=1500.0)
    assert c.ok
    tx, ty = FRAME.xy(*c.tangent)
    assert -ty <= 2000.0 - 1500.0 + 1.0            # teğetten sonraki noktaya en az 1,5 km düz


def test_loiter_descent_fails_when_all_land():
    a = R.PlanAyar()
    c = R.plan_loiter_descent(R.Poz3(*at(3000, 0), 0.0, 530.0), R.Poz3(*at(0, 0), 180.0, 151.0),
                              lambda lat, lon: 300.0, a, next_along_m=5000.0)
    assert not c.ok and 'çember' in c.reason


def test_settings_from_px4_params():
    p = dict(FW_AIRSPD_TRIM=45.0, FW_R_LIM=25.0, NAV_ACC_RAD=440.0, FW_T_SINK_R_SP=2.0, FW_T_CLMB_R_SP=4.0)
    a = R.ayar_from_px4(p)
    assert abs(D.turn_radius(a.airspeed, a.bank_deg) - 443.0) < 1.0
    assert a.corridor_m == 500.0
    assert abs(a.descent_gradient - 2.0 / 45.0) < 1e-12 and abs(a.climb_gradient - 4.0 / 45.0) < 1e-12
    loiter = D.turn_radius(a.airspeed, a.bank_deg) * a.loiter_turn_factor
    assert abs(math.degrees(math.atan(a.airspeed ** 2 / (9.81 * loiter))) - 16.2) < 0.3   # çemberde ~16° yatış
    # parametre değişince plan uyar: NAV_ACC_RAD 200 -> koridor 260, FW_R_LIM 35 -> yarıçap küçülür
    b = R.ayar_from_px4(dict(p, NAV_ACC_RAD=200.0, FW_R_LIM=35.0))
    assert b.corridor_m == 260.0 and b.radius < a.radius


def test_missing_px4_param_is_an_error():
    import pytest
    with pytest.raises(RuntimeError):
        R.read_px4_params(lambda name: None if name == 'NAV_ACC_RAD' else 1.0)
