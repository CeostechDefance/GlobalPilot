"""Araziye duyarlı, maliyete göre rota (arazi_rota.py) için yapay araziyle testler."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from otonom.planlayici import arazi_rota as AR  # noqa: E402
from otonom.planlayici import rota as R  # noqa: E402

LAT0, LON0 = 36.70, 28.80
F = R.YerelCerceve(LAT0, LON0)
A = AR.RotaAyar()
HOME = 5.0
START = (LAT0, LON0, 16.0)


def at(x, y):
    return F.latlon(x, y)


def xs(route):
    return [F.xy(p[0], p[1])[0] for p in route.points]


def check_clear(route, terrain, a=A, corridor=500.0):
    """Her bacakta nokta nokta: uçak bacak başı irtifasından tırmanış eğimiyle (en çok bacak sonu irtifasına) yükselir;
    her örnekte irtifa >= koridordaki arazi + o noktadaki pay (kalkış payı ve koridoru uzaklıkla büyür)."""
    pts = [F.xy(p[0], p[1]) + (p[2],) for p in route.points]
    for n, ((x1, y1, z1), (x2, y2, z2)) in enumerate(zip(pts, pts[1:])):
        length = math.hypot(x2 - x1, y2 - y1)
        steps = max(1, int(length // 100))
        width = min(corridor, a.climbout_corridor_m + a.climbout_corridor_slope * math.hypot(x1, y1))
        ux, uy = ((x2 - x1) / length, (y2 - y1) / length) if length else (1.0, 0.0)
        for k in range(steps + 1):
            s = length * k / steps
            x, y = x1 + ux * s, y1 + uy * s
            z = min(z2, z1 + a.climb_gradient * s) if z2 >= z1 else max(z2, z1 - a.descent_gradient * s)
            # pay hücre merkezine göre: hücre boyu kadar uzaklık farkı tolere edilir
            margin = min(a.clearance_m, a.climbout_margin_m + 0.5 * a.climb_gradient * max(0.0, math.hypot(x, y) - a.cell_m))
            for off in (-width, 0.0, width):
                h = terrain(*F.latlon(x - uy * off, y + ux * off))
                assert z >= h + margin - 1e-6, (n, round(s), round(z, 1), h, round(margin, 1))


def flat(lat, lon):
    return 0.0


def wall(gap_x, gap_w, wall_y, height, thick=300.0):
    def fn(lat, lon):
        x, y = F.xy(lat, lon)
        return height if abs(y - wall_y) < thick and abs(x - gap_x) > gap_w / 2 else 0.0
    return fn


def test_flat_goes_straight():
    r = AR.plan_route(START, at(0, 20000), HOME, flat, A)
    assert r.ok, r.reason
    assert max(abs(x) for x in xs(r)) < 300 and r.loiter_climb_m == 0.0
    assert r.points[-1][2] >= HOME + A.min_cruise_rel_m


def test_near_gap_cheaper_than_climbing_over():
    # 3 km ötede 1500 m duvar; 2 km yanda geçit: dolanma ~1 km, aşmak ~1650 m tırmanış (çember) -> geçit
    terrain = wall(2000.0, 1600.0, 3000.0, 1500.0)
    r = AR.plan_route(START, at(0, 8000), HOME, terrain, A)
    assert r.ok, r.reason
    assert max(xs(r)) > 1000.0 and r.loiter_climb_m == 0.0
    check_clear(r, terrain)


def test_far_gap_climb_over_is_cheaper():
    # 3 km ötede 400 m duvar, geçit 20 km yanda: dolanma ~+30 km; çemberle ~300 m tırmanış daha ucuz -> üstünden
    terrain = wall(20000.0, 1600.0, 3000.0, 400.0)
    r = AR.plan_route(START, at(0, 8000), HOME, terrain, AR.RotaAyar(margin_m=22000.0, cell_m=400.0))
    assert r.ok, r.reason
    assert max(abs(x) for x in xs(r)) < 3000.0          # dolanmadı
    assert r.loiter_climb_m > 0.0                        # duvarın önünde çember tırmanışı
    margin_3km = min(A.clearance_m, A.climbout_margin_m + 0.5 * A.climb_gradient * 3000.0)
    assert max(p[2] for p in r.points) >= 400.0 + margin_3km - 1e-6
    check_clear(r, terrain, AR.RotaAyar(margin_m=22000.0, cell_m=400.0))


def test_distant_ridge_climbed_on_the_way():
    # 15 km ötede 600 m sırt: yol boyunca tırmanış yeter, çember gerekmez, dolanma yok
    terrain = wall(0.0, 0.0, 15000.0, 600.0)
    r = AR.plan_route(START, at(0, 25000), HOME, terrain, A)
    assert r.ok, r.reason
    assert max(abs(x) for x in xs(r)) < 1000.0
    assert r.loiter_climb_m <= 2 * A.level_m          # ızgara adımından en çok küçük bir çember
    check_clear(r, terrain)


def test_no_data_is_not_ok():
    r = AR.plan_route(START, at(0, 5000), HOME, lambda lat, lon: None, A)
    assert not r.ok
