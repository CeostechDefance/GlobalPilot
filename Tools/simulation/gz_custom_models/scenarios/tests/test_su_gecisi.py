"""Otonom su geçişi (planlayici/su_gecisi.py) için simülasyonsuz testler: yapay arazi ve su."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from otonom.planlayici import rota as R  # noqa: E402
from otonom.planlayici import su_gecisi as SG  # noqa: E402

LAT0, LON0 = 36.65, 28.79
FRAME = R.YerelCerceve(LAT0, LON0)
A = SG.SuGecisAyar()


def at(x, y):
    return FRAME.latlon(x, y)


def coast(shore_y, land_h):
    """y < shore_y kara (land_h), ötesi deniz (0)."""
    def terrain(lat, lon):
        return land_h if FRAME.xy(lat, lon)[1] < shore_y else 0.0

    def water(lat, lon):
        return FRAME.xy(lat, lon)[1] >= shore_y
    return terrain, water


def profile_min_agl(res, start, alt, entry, entry_amsl, terrain):
    """Planlanan yolun her noktasında yerden yükseklik (düz alçalış için)."""
    sx, sy = FRAME.xy(*start)
    ex, ey = FRAME.xy(*entry)
    length = math.hypot(ex - sx, ey - sy)
    worst = float('inf')
    for i in range(201):
        s = length * i / 200
        a = max(entry_amsl, alt - max(0.0, s - res.descent_start_m) * A.descent_gradient)
        worst = min(worst, a - terrain(*at(sx + (ex - sx) * s / length, sy + (ey - sy) * s / length)))
    return worst


def test_straight_descent_waits_for_ground():
    # 50 m kara 8 km boyunca, sonra deniz. 600 m'den 100 m'ye (giriş 20 km'de) düz sığar.
    terrain, water = coast(8000.0, 50.0)
    start, entry = at(0, 0), at(0, 20000)
    res = SG.plan_water_transit(start, 600.0, entry, 0.0, 100.0, terrain, water)
    assert res.ok and res.mode == 'düz alçalış'
    assert res.descent_start_m >= A.min_transit_m
    assert profile_min_agl(res, start, 600.0, entry, 100.0, terrain) >= A.margin_m - 1e-6


def test_high_coast_delays_descent():
    # Kıyıda 450 m tepe: alçalış tepeyi 100 m payla geçecek kadar geç başlamalı.
    def terrain(lat, lon):
        y = FRAME.xy(lat, lon)[1]
        return 450.0 if 9000 <= y < 10000 else (0.0 if y >= 10000 else 20.0)
    start, entry = at(0, 0), at(0, 25000)
    res = SG.plan_water_transit(start, 700.0, entry, 0.0, 100.0, terrain)
    assert res.ok and res.mode == 'düz alçalış'
    # tepe sonunda (10 km) irtifa >= 550 m: alçalış en erken 10000 - 150/g'de başlar
    assert res.descent_start_m >= 10000 - (700.0 - 550.0) / A.descent_gradient - A.step_m
    assert profile_min_agl(res, start, 700.0, entry, 100.0, terrain) >= A.margin_m - 1e-6


def test_too_high_large_water_circles_over_water():
    # Giriş 3 km'de, 1500 m irtifa: düz sığmaz. Deniz geniş: su üstünde çember.
    terrain, water = coast(1000.0, 30.0)
    res = SG.plan_water_transit(at(0, 0), 1500.0, at(0, 3000), 0.0, 100.0, terrain, water)
    assert res.ok and res.mode == 'su üstünde çember'
    assert res.items[0]['kind'] == 'loiter_to_alt' and res.items[0]['alt_amsl'] == 100.0


def test_small_water_loses_height_over_land():
    # Su yalnız 2 km genişlikte bir şerit (göl): çember sığmaz, irtifa karada kaybedilir.
    def terrain(lat, lon):
        y = FRAME.xy(lat, lon)[1]
        return 0.0 if 12000 <= y < 14000 else 40.0

    def water(lat, lon):
        return 12000 <= FRAME.xy(lat, lon)[1] < 14000
    start, entry = at(0, 0), at(0, 12000)
    res = SG.plan_water_transit(start, 1200.0, entry, 0.0, 100.0, terrain, water)
    assert res.ok and res.mode == 'karada çember', res.reason
    it = res.items[0]
    assert it['kind'] == 'loiter_to_alt'
    assert it['alt_amsl'] >= 40.0 + A.margin_m        # karada da yerden en az 100 m
    cx, cy = FRAME.xy(it['lat'], it['lon'])
    assert cy < 12000                                  # çember karada
    assert res.descent_start_m + (it['alt_amsl'] - 100.0) / A.descent_gradient <= 12000 + A.water_slack_m - cy + 1e-6  # çemberden sonra alçalış sığar
    # alçalış boyunca karada yerden >= 100 m
    for k in range(0, 1001):
        y = cy + (12000 + A.water_slack_m - cy) * k / 1000
        alt = max(100.0, it['alt_amsl'] - max(0.0, y - cy - res.descent_start_m) * A.descent_gradient)
        assert alt - terrain(*at(0, y)) >= A.margin_m - 1e-6


def test_refuses_when_terrain_above_altitude():
    terrain, water = coast(8000.0, 550.0)
    res = SG.plan_water_transit(at(0, 0), 600.0, at(0, 20000), 0.0, 100.0, terrain, water)
    assert not res.ok and 'tırmanılmalı' in res.reason


def test_missing_terrain():
    res = SG.plan_water_transit(at(0, 0), 600.0, at(0, 20000), 0.0, 100.0, lambda lat, lon: None)
    assert not res.ok and 'verisi yok' in res.reason
