"""Piste dönüş deseni (otonom/planlayici/pist.py): kısa yön, düz giriş, taban yanı, arazi eleme."""
import math
from otonom.planlayici import geo as G
from otonom.planlayici import pist as PL

N = (36.726091, 28.796766)
S = (36.701195, 28.788088)
DIRS = [PL.PistYonu('19', N, 195.68, -1.54), PL.PistYonu('01', S, 15.68, -2.25)]
FLAT = lambda lat, lon: 5.0  # noqa: E731


def test_from_south_lands_01_straight_in():
    start = G.along(*S, 195.68, 9000.0)
    plan = PL.plan_runway_return(start, 300.0, 15.0, DIRS, 6.0, FLAT)
    assert plan.runway.name == '01' and plan.entry == 'düz giriş'
    assert [p[0] for p in plan.points][0] == 'Son yaklaşma'


def test_from_north_lands_19_straight_in():
    start = G.along(*N, 15.68, 9000.0)
    plan = PL.plan_runway_return(start, 300.0, 195.0, DIRS, 6.0, FLAT)
    assert plan.runway.name == '19' and plan.entry == 'düz giriş'


def test_from_side_uses_base_on_own_side():
    start = G.along(*N, 285.68, 7000.0)   # 19'a göre sağ (batı) yanda, pist ortası hizasında
    plan = PL.plan_runway_return(start, 300.0, 105.0, DIRS, 6.0, FLAT)
    assert plan.entry.startswith('taban')
    assert plan.points[0][0] == 'Taban bacağı'
    assert plan.points[-1][0] == 'İniş'


def test_terrain_rejects_and_none_when_all_blocked():
    start = G.along(*S, 195.68, 9000.0)
    wall = lambda lat, lon: 1000.0  # noqa: E731
    plan = PL.plan_runway_return(start, 300.0, 15.0, DIRS, 6.0, wall)
    assert not plan.ok and len(plan.rejected) >= 4 and all(why for _, why in plan.rejected)


def test_glide_terrain_checked():
    start = G.along(*S, 195.68, 9000.0)

    def hill(lat, lon):   # 01 süzülüşünün başında 140 m tepe (süzülüş ~145 m, pay 15 m)
        mid = G.along(*S, 15.68, -1500.0)
        return 140.0 if G.distance_m(lat, lon, *mid) < 200.0 else 5.0
    plan = PL.plan_runway_return(start, 300.0, 15.0, DIRS[1:], 6.0, hill)
    assert not plan.ok


def test_straight_in_needs_room():
    final = G.along(*S, 15.68, -4500.0)
    start = G.along(*final, 15.68, -300.0)   # son yaklaşma başının 300 m gerisi: iki yarıçaptan yakın
    plan = PL.plan_runway_return(start, 300.0, 15.0, DIRS[1:], 6.0, FLAT)
    assert plan.entry != 'düz giriş'


def test_shorter_direction_wins():
    start = G.along(*S, 195.68, 9000.0)
    plan = PL.plan_runway_return(start, 300.0, 15.0, DIRS, 6.0, FLAT)
    other = PL.plan_runway_return(start, 300.0, 15.0, DIRS[:1], 6.0, FLAT)
    assert plan.length_m < other.length_m


# İlk dönüş (2026-11-27): uçağın yönü verilince desenin ilk noktasına Dubins yolu, yay noktaları ve koridorlu arazi.
EMPTY = PL.PistAyar(airspeed=36.7)                      # boş uçak: yarıçap ~370 m
START = G.along(*S, 120.0, 6000.0)                    # 01 pist başının doğu-güneydoğusu
COURSE = 82.0                                           # pistten uzaklaşan yön: dönüş ~180°


def _track(p, origin=START, course=COURSE):
    """p'nin origin'e göre (ileri, sağa) uzaklığı (m)."""
    return G.track_errors(p[0], p[1], origin, course)


def _ridge_right(lat, lon):
    """Başlangıcın sağında (güney) sırt: ileri -1500..2500 m, sağa 600..2000 m, 400 m. 2026-11-27 çarpmasının benzeri."""
    along, right = _track((lat, lon))
    return 400.0 if -1500.0 <= along <= 2500.0 and 600.0 <= right <= 2000.0 else 5.0


def test_u_turn_goes_away_from_ridge():
    plan = PL.plan_runway_return(START, 300.0, COURSE, DIRS, 6.0, _ridge_right, EMPTY, early_m=440.0)
    assert plan.ok, plan.rejected
    assert plan.turn == 'sola'
    assert plan.transition, 'yay noktaları göreve yazılmalı: yön PX4\'e bırakılmaz'
    assert _track(plan.transition[0][1])[1] < 0.0          # ilk yay noktası solda
    assert all(_ridge_right(*p) < 100.0 for _, p, _ in plan.transition)
    assert plan.min_clearance_m >= EMPTY.margin_m


def test_first_arc_holds_altitude():
    plan = PL.plan_runway_return(START, 300.0, COURSE, DIRS, 6.0, FLAT, EMPTY, early_m=440.0)
    assert plan.ok and plan.transition
    arc = [alt for name, _, alt in plan.transition if 'yay' in name][:2]
    assert arc and all(abs(alt - 300.0) < 1e-6 for alt in arc)
    assert all(alt >= plan.points[0][2] - 1e-6 for _, _, alt in plan.transition)


def test_early_turn_behind_start_is_checked():
    """PX4 başlangıç noktasını 440 m erken bırakır: dönüş noktanın gerisinde başlayabilir. Oradaki tepe sola dönüşü de
    kapatınca (sağda sırt) dönüş yoktur; erken pay verilmezse tepe görülmez."""
    def terrain(lat, lon):
        along, right = _track((lat, lon))
        if -440.0 <= along <= -150.0 and -450.0 <= right <= -150.0:
            return 400.0
        return _ridge_right(lat, lon)
    late = PL.plan_runway_return(START, 300.0, COURSE, DIRS, 6.0, terrain, EMPTY, early_m=0.0)
    early = PL.plan_runway_return(START, 300.0, COURSE, DIRS, 6.0, terrain, EMPTY, early_m=440.0)
    assert late.ok and late.turn == 'sola'
    assert not early.ok
    assert any('erken' in why for _, why in early.rejected)


def test_low_start_climb_is_rate_limited():
    """Uçak alçaktaysa (gerçek irtifa) istenen irtifaya tırmanış sınırlı: önündeki tepe görülür."""
    def hill_ahead(lat, lon):
        along, right = _track((lat, lon))
        return 150.0 if 0.0 <= along <= 1200.0 and abs(right) <= 1200.0 else 5.0
    high = PL.plan_runway_return(START, 300.0, COURSE, DIRS, 6.0, hill_ahead, EMPTY, actual_rel_m=300.0)
    low = PL.plan_runway_return(START, 300.0, COURSE, DIRS, 6.0, hill_ahead, EMPTY, actual_rel_m=60.0)
    assert high.ok
    assert not low.ok


def test_low_start_from_sea_climbs_out():
    """Su bölümünden sonra uçak denizin ~35 m üstünde: kalkıştaki gibi pay ve koridor dar başlar, büyür. Kıyıdaki alçak
    arazi (20 m) dönüşü kapatmaz; aynı yükseklikte karadan başlangıçta tam pay istenir."""
    start = G.along(*S, 200.0, 12000.0)            # 01 pist başının güneyi, denizde

    def coast(lat, lon):
        along, _ = _track((lat, lon), start, 0.0)
        return -5.0 if along < 3000.0 else 20.0      # başlangıcın 3 km kuzeyinden sonra alçak kara
    sea = PL.plan_runway_return(start, 300.0, 250.0, DIRS, 6.0, coast, EMPTY, actual_rel_m=28.0)
    assert sea.ok, sea.rejected
    land = PL.plan_runway_return(start, 300.0, 250.0, DIRS, 6.0, lambda la, lo: 20.0, EMPTY, actual_rel_m=28.0)
    assert not land.ok


def test_waypoint_altitudes_are_reachable():
    """PX4 noktayı irtifa farkı NAV_FW_ALT_RAD'den büyükken bırakmaz ve orada döner (2026-11-27 test 2). Alçak başlangıçta
    noktaların irtifası uçağın tırmanabileceğini aşmaz."""
    plan = PL.plan_runway_return(START, 300.0, COURSE, DIRS, 6.0, FLAT, EMPTY, actual_rel_m=60.0, early_m=0.0)
    assert plan.ok, plan.rejected
    frame = PL.R.YerelCerceve(*START)
    sx, sy = frame.xy(*START)
    s = 0.0
    prev = (sx, sy)
    for name, p, alt in plan.transition:
        x, y = frame.xy(*p)
        s += math.hypot(x - prev[0], y - prev[1])     # kiriş: yol boyundan kısa, sınır daha sıkı
        prev = (x, y)
        assert alt <= 60.0 + EMPTY.climb_gradient * s + EMPTY.alt_accept_m, (name, alt, s)


def test_unreachable_waypoint_altitude_rejected():
    """Uçak 60 m'deyken ilk noktası 300 m isteyen yol: PX4 orada döner, denetim yolu kabul etmez."""
    frame = PL.R.YerelCerceve(*START)
    path = PL.D.shortest(PL.D.Poz(0.0, 0.0, 0.0), PL.D.Poz(0.0, 3000.0, 0.0), 370.0)
    wps = [(1000.0, 0.0, 1000.0, 306.0, 'S', False), (3000.0, 0.0, 3000.0, 306.0, 'S', False)]
    ok, _, why = PL._fly(path, wps, 66.0, FLAT, frame, EMPTY)
    assert not ok and 'döner' in why
    reachable = [(1000.0, 0.0, 1000.0, 66.0 + 0.07 * 1000.0, 'S', False), (3000.0, 0.0, 3000.0, 66.0 + 0.07 * 3000.0, 'S', False)]
    assert PL._fly(path, reachable, 66.0, FLAT, frame, EMPTY)[0]
