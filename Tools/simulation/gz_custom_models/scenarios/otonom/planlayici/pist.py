"""Piste dönüş deseni: iki iniş yönünden yolu kısa olanı seçer (kullanıcı, 2026-11-11).

Kural: uçak piste hangi yönden inmek daha kısa yol tutuyorsa o yönden iner; pistin ötesine gidip geri dolaşmaz.
Desen uçağın bulunduğu yerden çizilir:
- düz giriş: uçak son yaklaşma çizgisine az bir açıyla geliyorsa doğrudan son yaklaşma başına,
- taban girişi: değilse uçağın bulunduğu yandaki (ya da öbür yandaki) taban noktasına, oradan son yaklaşmaya.
Her aday için yol uzunluğu ilk dönüşün yayıyla birlikte hesaplanır; arazi payı tutmayan aday elenir.
Hiçbir aday tutmazsa ``None`` döner, çağıran eski deseni kullanır.

Arazi bir fonksiyondur: terrain(lat, lon) -> m (deniz seviyesine göre) ya da None. Simülasyon ya da MAVLink bilmez.
"""
import math
from dataclasses import dataclass, field

__all__ = ['PistAyar', 'PistYonu', 'PistDeseni', 'plan_runway_return']

R_EARTH_LAT = 111320.0


def _offset(lat, lon, north, east):
    return lat + north / R_EARTH_LAT, lon + east / (R_EARTH_LAT * math.cos(math.radians(lat)))


def _along(lat, lon, bearing_deg, distance, right=0.0):
    b = math.radians(bearing_deg)
    return _offset(lat, lon, distance * math.cos(b) - right * math.sin(b), distance * math.sin(b) + right * math.cos(b))


def _dist(a, b):
    return math.hypot((b[0] - a[0]) * R_EARTH_LAT, (b[1] - a[1]) * R_EARTH_LAT * math.cos(math.radians(a[0])))


def _bearing(a, b):
    n = (b[0] - a[0]) * R_EARTH_LAT
    e = (b[1] - a[1]) * R_EARTH_LAT * math.cos(math.radians(a[0]))
    return math.degrees(math.atan2(e, n)) % 360.0


def _turn(a, b):
    """a'dan b'ye en kısa dönüş açısı (derece, 0..180)."""
    return abs((b - a + 180.0) % 360.0 - 180.0)


@dataclass(frozen=True)
class PistAyar:
    land_along_m: float = 1200.0        # iniş noktası, iniş yönündeki pist başından
    final_m: float = 4500.0             # son yaklaşma başı, pist başının bu kadar gerisinde
    base_m: float = 1500.0              # taban noktası son yaklaşma çizgisinin bu kadar yanında
    pattern_rel_m: float = 150.0        # taban ve son yaklaşma yüksekliği (kalkış noktasına göre)
    final_angle_deg: float = 2.99       # süzülüş açısı (FW_LND_ANG altında)
    turn_radius_m: float = 500.0        # 45 m/s, 25° yatış: 443 m; pay ile
    straight_in_max_deg: float = 45.0   # son yaklaşma başına bu açıdan küçük gelişte taban noktası atlanır
    margin_m: float = 50.0              # yol boyunca yerden en az bu kadar yukarıda (ROUTE_MIN_MARGIN_M)
    glide_margin_m: float = 15.0        # süzülüşte yerden en az bu kadar (iniş noktasına glide_check_end_m kalana dek)
    glide_check_end_m: float = 1000.0   # süzülüşün son bu kadarı (pist yaklaşma alanı) denetlenmez
    step_m: float = 100.0               # arazi örnek aralığı


@dataclass(frozen=True)
class PistYonu:
    """Bir iniş yönü: pist başı (iniş yönünde ilk eşik), yön ve iniş noktasının kalkış noktasına göre yüksekliği."""
    name: str
    threshold: tuple                    # (lat, lon)
    heading_deg: float
    land_rel_m: float


@dataclass
class PistDeseni:
    runway: PistYonu
    entry: str                          # 'düz giriş' | 'taban (sağ)' | 'taban (sol)'
    length_m: float
    points: list = field(default_factory=list)   # [(ad, (lat, lon), göreli yükseklik)], iniş noktası en sonda
    rejected: list = field(default_factory=list)  # [(aday, gerekçe)]

    @property
    def ok(self):
        return bool(self.points)


def _leg_clear(terrain, home_amsl, a, b, alt_a, alt_b, ayar, margin=None, end_skip_m=0.0):
    """a'dan b'ye düz yolda yükseklik alt_a'dan alt_b'ye doğrusal; her örnekte yer + pay altında kalmamalı.
    end_skip_m: yolun son bu kadarı denetlenmez."""
    margin = ayar.margin_m if margin is None else margin
    length = _dist(a, b)
    steps = max(1, int(length / ayar.step_m))
    for i in range(steps + 1):
        k = i / steps
        if k * length > length - end_skip_m:
            break
        ground = terrain(a[0] + (b[0] - a[0]) * k, a[1] + (b[1] - a[1]) * k)
        if ground is None:
            return False, 'arazi verisi yok'
        alt = home_amsl + alt_a + (alt_b - alt_a) * k
        if ground + margin > alt:
            return False, f'{k * length:.0f}. m\'de arazi {ground:.0f} m, yükseklik {alt:.0f} m'
    return True, None


def _candidates(start, course_deg, rw, ayar):
    land = _along(*rw.threshold, rw.heading_deg, ayar.land_along_m)
    final = _along(*rw.threshold, rw.heading_deg, -ayar.final_m)
    glide_len = (ayar.pattern_rel_m - rw.land_rel_m) / math.tan(math.radians(ayar.final_angle_deg))
    glide = _along(*rw.threshold, rw.heading_deg, ayar.land_along_m - glide_len)
    tail = [('Son yaklaşma', final, ayar.pattern_rel_m)]
    if _dist(final, land) > glide_len + 1.0:
        tail.append(('Süzülüş başı', glide, ayar.pattern_rel_m))
    tail.append(('İniş', land, rw.land_rel_m))
    final_len = _dist(final, land)
    out = []
    to_final = _bearing(start, final)
    # düz giriş: açı küçük ve son yaklaşma başı en az iki dönüş yarıçapı uzakta (yakınsa yol uçulamaz, 7b)
    if _turn(to_final, rw.heading_deg) <= ayar.straight_in_max_deg and _dist(start, final) >= 2.0 * ayar.turn_radius_m:
        turns = _turn(course_deg, to_final) + _turn(to_final, rw.heading_deg) if course_deg is not None else 0.0
        out.append(('düz giriş', [], _dist(start, final) + final_len + ayar.turn_radius_m * math.radians(turns)))
    for side, label in ((1.0, 'taban (sağ)'), (-1.0, 'taban (sol)')):
        base = _along(*final, rw.heading_deg, 0.0, side * ayar.base_m)
        to_base = _bearing(start, base)
        base_course = _bearing(base, final)
        turns = (_turn(course_deg, to_base) if course_deg is not None else 0.0) + _turn(to_base, base_course) + \
            _turn(base_course, rw.heading_deg)
        length = _dist(start, base) + _dist(base, final) + final_len + ayar.turn_radius_m * math.radians(turns)
        out.append((label, [('Taban bacağı', base, ayar.pattern_rel_m)], length))
    return [(label, head + tail, length) for label, head, length in out]


def plan_runway_return(start, start_rel_m, course_deg, runways, home_amsl, terrain, ayar=PistAyar()):
    """En kısa piste dönüş deseni.

    start: (lat, lon) dönüşün başladığı nokta; start_rel_m: oradaki yükseklik (kalkış noktasına göre);
    course_deg: oradaki uçuş yönü (bilinmiyorsa None, dönüş payı yalnız desen içinde sayılır);
    runways: PistYonu listesi (aynı pistin iki yönü); home_amsl: kalkış noktasının deniz seviyesine göre yüksekliği.
    Dönüş: PistDeseni. Hiçbir aday arazi payını tutmazsa points boş (ok False), rejected her adayın gerekçesini taşır.
    """
    best, rejected = None, []
    for rw in runways:
        for label, points, length in _candidates(start, course_deg, rw, ayar):
            name = f'{rw.name} {label}'
            # ilk bacakta start yüksekliğinden desen yüksekliğine iniş, desen yüksekliğinde süzülüş başına kadar ayar.margin_m;
            # süzülüşte (iniş noktasına doğru) glide_margin_m, son glide_check_end_m hariç
            prev, prev_alt, ok, why = start, start_rel_m, True, None
            for pname, p, alt in points:
                if pname == 'İniş':
                    ok, why = _leg_clear(terrain, home_amsl, prev, p, prev_alt, alt, ayar, ayar.glide_margin_m,
                                         ayar.glide_check_end_m)
                else:
                    ok, why = _leg_clear(terrain, home_amsl, prev, p, prev_alt, alt, ayar)
                if not ok:
                    why = f'{pname}: {why}'
                    break
                prev, prev_alt = p, alt
            if not ok:
                rejected.append((name, why))
                continue
            if best is None or length < best.length_m:
                best = PistDeseni(rw, label, length, points)
    if best is None:
        return PistDeseni(None, None, float('inf'), [], rejected)
    best.rejected = rejected
    return best
