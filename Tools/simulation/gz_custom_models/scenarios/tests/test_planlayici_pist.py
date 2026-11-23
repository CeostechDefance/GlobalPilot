"""Piste dönüş deseni (otonom/planlayici/pist.py): kısa yön, düz giriş, taban yanı, arazi eleme."""
from otonom.planlayici import pist as PL

N = (36.726091, 28.796766)
S = (36.701195, 28.788088)
DIRS = [PL.PistYonu('19', N, 195.68, -1.54), PL.PistYonu('01', S, 15.68, -2.25)]
FLAT = lambda lat, lon: 5.0  # noqa: E731


def test_from_south_lands_01_straight_in():
    start = PL._along(*S, 195.68, 9000.0)
    plan = PL.plan_runway_return(start, 300.0, 15.0, DIRS, 6.0, FLAT)
    assert plan.runway.name == '01' and plan.entry == 'düz giriş'
    assert [p[0] for p in plan.points][0] == 'Son yaklaşma'


def test_from_north_lands_19_straight_in():
    start = PL._along(*N, 15.68, 9000.0)
    plan = PL.plan_runway_return(start, 300.0, 195.0, DIRS, 6.0, FLAT)
    assert plan.runway.name == '19' and plan.entry == 'düz giriş'


def test_from_side_uses_base_on_own_side():
    start = PL._along(*N, 285.68, 7000.0)   # 19'a göre sağ (batı) yanda, pist ortası hizasında
    plan = PL.plan_runway_return(start, 300.0, 105.0, DIRS, 6.0, FLAT)
    assert plan.entry.startswith('taban')
    assert plan.points[0][0] == 'Taban bacağı'
    assert plan.points[-1][0] == 'İniş'


def test_terrain_rejects_and_none_when_all_blocked():
    start = PL._along(*S, 195.68, 9000.0)
    wall = lambda lat, lon: 1000.0  # noqa: E731
    plan = PL.plan_runway_return(start, 300.0, 15.0, DIRS, 6.0, wall)
    assert not plan.ok and len(plan.rejected) >= 4 and all(why for _, why in plan.rejected)


def test_glide_terrain_checked():
    start = PL._along(*S, 195.68, 9000.0)
    final = PL._along(*S, 15.68, -4500.0)
    land = PL._along(*S, 15.68, 1200.0)

    def hill(lat, lon):   # 01 süzülüşünün başında 140 m tepe (süzülüş ~145 m, pay 15 m)
        mid = PL._along(*S, 15.68, -1500.0)
        return 140.0 if PL._dist((lat, lon), mid) < 200.0 else 5.0
    plan = PL.plan_runway_return(start, 300.0, 15.0, DIRS[1:], 6.0, hill)
    assert not plan.ok


def test_straight_in_needs_room():
    final = PL._along(*S, 15.68, -4500.0)
    start = PL._along(*final, 15.68, -300.0)   # son yaklaşma başının 300 m gerisi: iki yarıçaptan yakın
    plan = PL.plan_runway_return(start, 300.0, 15.0, DIRS[1:], 6.0, FLAT)
    assert plan.entry != 'düz giriş'


def test_shorter_direction_wins():
    start = PL._along(*S, 195.68, 9000.0)
    plan = PL.plan_runway_return(start, 300.0, 15.0, DIRS, 6.0, FLAT)
    other = PL.plan_runway_return(start, 300.0, 15.0, DIRS[:1], 6.0, FLAT)
    assert plan.length_m < other.length_m
