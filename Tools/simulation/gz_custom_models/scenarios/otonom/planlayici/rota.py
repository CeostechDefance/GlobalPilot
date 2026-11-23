"""Bacak planı: iki poz (konum, rota, irtifa) arasında dönüş yarıçapına uyan ve araziyi koridorla temizleyen yol.

- Yatay yol Dubins (dubins.py). Yarıçap = hız ve yatış sınırından, rüzgâr ve PX4 dönüş payıyla büyütülür.
- İrtifa profili ölçülmüş eğimlerle: tırmanış olabildiğince erken, alçalış olabildiğince geç (arazi üstünde yüksek kalınır).
- Arazi yolun üstünde ve iki yanında (koridor) örneklenir; her örnekte irtifa >= arazi + pay olmalı.
- Plan, sonunda güvenli bir bekleme çemberi olan noktada biter (pas geçme ve gecikmede uçak orada döner).
- Çıktı PX4 görev noktalarıdır: yay ve düz parça sınırları ile uzun yayların ara noktaları.
Arazi bir fonksiyon olarak verilir (enlem, boylam → m, ya da None = veri yok); simülasyon ya da dosya bilmez.
"""
import math
from dataclasses import dataclass, field

from . import dubins as D

EARTH_M_PER_DEG = 111320.0


class YerelCerceve:
    """Küçük alanlar için düz yer yaklaşımı: x doğu, y kuzey (m)."""

    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = lat0, lon0
        self.kx = EARTH_M_PER_DEG * math.cos(math.radians(lat0))

    def xy(self, lat, lon):
        return (lon - self.lon0) * self.kx, (lat - self.lat0) * EARTH_M_PER_DEG

    def latlon(self, x, y):
        return self.lat0 + y / EARTH_M_PER_DEG, self.lon0 + x / self.kx


class OnbellekliArazi:
    """Arazi fonksiyonunu hücre ızgarasında önbellekler (aynı hücreye ikinci okuma bedava)."""

    def __init__(self, height_fn, cell_m=30.0):
        self.fn, self.cell = height_fn, cell_m
        self.cache = {}

    def __call__(self, lat, lon):
        key = (round(lat * EARTH_M_PER_DEG / self.cell), round(lon * EARTH_M_PER_DEG * math.cos(math.radians(lat)) / self.cell))
        if key not in self.cache:
            try:
                self.cache[key] = self.fn(lat, lon)
            except (OSError, SystemExit, ValueError):
                self.cache[key] = None
        return self.cache[key]


@dataclass(frozen=True)
class PlanAyar:
    airspeed: float = 45.0
    bank_deg: float = 25.0
    radius_margin: float = 1.25        # rüzgâr ve PX4'ün dönüşe erken/geç girmesi için
    clearance_m: float = 150.0         # yol üstünde arazi payı
    corridor_m: float = 500.0          # yolun iki yanında arazi denetimi: PX4 noktaya NAV_ACC_RAD (440 m) kala döner,
                                       # uçulan yol planlanan Dubins yolundan en çok 451 m saptı (uçuş p1)
    corridor_step_m: float = 30.0      # yan örnek aralığı, aynı nedenle
    climb_gradient: float = 3.45 / 45.0    # ölçülen: seyir tırmanışı 3,45 m/s @45 m/s
    descent_gradient: float = 2.0 / 45.0   # ölçülen: PX4 görevde 2 m/s alçalır @45 m/s
    sample_m: float = 30.0             # arazi ızgarası ~30 m (place_wildfire): daha seyrek örnek dar tepeyi kaçırır
    loiter_radius_m: float = 500.0     # en az; gerçek yarıçap loiter_turn_factor ile hızdan
    loiter_turn_factor: float = 1.6    # PX4 alçalırken en küçük dönüş yarıçapındaki çemberi tutamıyor: 522 m planlandı,
                                       # medyan 846 m uçuldu (p1, yatış 25°'de doymuş). ~1,6 kat ile yatış payı kalır
    loiter_check_factor: float = 1.7   # çember denetimi bu kat yarıçapa kadar (circle_clear ile aynı)
    arc_point_deg: float = 60.0        # bundan uzun yaylara ara görev noktası

    @property
    def radius(self):
        return D.turn_radius(self.airspeed, self.bank_deg) * self.radius_margin


@dataclass(frozen=True)
class Poz3:
    lat: float
    lon: float
    course_deg: float
    alt_amsl: float


@dataclass
class Bacak:
    ok: bool
    reason: str = None
    path: D.DubinsYol = None
    samples: list = field(default_factory=list)   # (s, lat, lon, course, alt, arazi_max)
    min_clearance: float = None
    items: list = field(default_factory=list)     # PX4 görev noktaları: dict(name, lat, lon, alt_amsl)


def altitude_at(s, length, start_alt, end_alt, a):
    """Tırmanışta erken, alçalışta geç: eğimle sınırlı irtifa profili. Ulaşılamıyorsa None döner (çağıran denetler)."""
    if end_alt >= start_alt:
        return min(end_alt, start_alt + a.climb_gradient * s)
    return min(start_alt, end_alt + a.descent_gradient * (length - s))


def feasible_altitude_change(length, start_alt, end_alt, a):
    need = (end_alt - start_alt) / a.climb_gradient if end_alt >= start_alt else (start_alt - end_alt) / a.descent_gradient
    return need <= length + 1e-6, need


def terrain_max(terrain, frame, x, y, course_deg, a):
    """Yol noktası ve iki yanındaki koridor noktalarında en yüksek arazi; veri yoksa None."""
    th = D.course_to_theta(course_deg)
    nx, ny = -math.sin(th), math.cos(th)   # sola dik
    heights = []
    n = max(1, int(math.ceil(a.corridor_m / a.corridor_step_m)))
    for off in [a.corridor_m * k / n for k in range(-n, n + 1)]:
        lat, lon = frame.latlon(x + nx * off, y + ny * off)
        h = terrain(lat, lon)
        if h is None:
            return None
        heights.append(h)
    return max(heights)


def loiter_clear(terrain, frame, x, y, alt_amsl, a):
    """Bekleme çemberi ve çevresi (loiter_check_factor) arazi payını sağlıyor mu?"""
    for r in (0.0, a.loiter_radius_m, a.loiter_check_factor * a.loiter_radius_m):
        for b in range(0, 360, 30):
            lat, lon = frame.latlon(x + r * math.sin(math.radians(b)), y + r * math.cos(math.radians(b)))
            h = terrain(lat, lon)
            if h is None or h + a.clearance_m > alt_amsl:
                return False
    return True


def plan_leg(start, end, terrain, a=PlanAyar(), require_loiter=True):
    """start, end: Poz3. En kısa Dubins adayından başlayarak araziyi ve eğimi sağlayan ilk yolu döner."""
    frame = YerelCerceve(start.lat, start.lon)
    sx, sy = frame.xy(start.lat, start.lon)
    ex, ey = frame.xy(end.lat, end.lon)
    if require_loiter and not loiter_clear(terrain, frame, ex, ey, end.alt_amsl, a):
        return Bacak(False, f'bitiş noktasında {a.loiter_radius_m:.0f} m bekleme çemberi araziyi temizlemiyor')
    reasons = []
    for path in D.all_paths(D.Poz(sx, sy, start.course_deg), D.Poz(ex, ey, end.course_deg), a.radius):
        ok_alt, need = feasible_altitude_change(path.length, start.alt_amsl, end.alt_amsl, a)
        if not ok_alt:
            reasons.append(f'{path.word}: irtifa değişimi {need:.0f} m yol ister, yol {path.length:.0f} m')
            continue
        samples, worst, failed = [], float('inf'), None
        for s, x, y, course, _ in path.sample(a.sample_m):
            alt = altitude_at(s, path.length, start.alt_amsl, end.alt_amsl, a)
            hmax = terrain_max(terrain, frame, x, y, course, a)
            lat, lon = frame.latlon(x, y)
            if hmax is None:
                failed = f'{path.word}: {s:.0f} m\'de arazi verisi yok'
                break
            margin = alt - hmax
            worst = min(worst, margin)
            if margin < a.clearance_m:
                failed = f'{path.word}: {s:.0f} m\'de arazi payı {margin:.0f} m < {a.clearance_m:.0f}'
                break
            samples.append((s, lat, lon, course, alt, hmax))
        if failed:
            reasons.append(failed)
            continue
        leg = Bacak(True, None, path, samples, worst)
        leg.items = mission_points(leg, frame, start, end, a)
        return leg
    return Bacak(False, '; '.join(reasons[:3]) or 'aday yol yok')


def mission_points(leg, frame, start, end, a):
    """PX4 görev noktaları: parça sınırları, uzun yaylarda arc_point_deg aralıklı ara noktalar, bitiş.
    İrtifa profilden okunur; başlangıç noktası (uçağın bulunduğu yer) görev noktası değildir."""
    path = leg.path
    points = []
    s_acc = 0.0
    x, y, th = path.start.x, path.start.y, D.course_to_theta(path.start.course_deg)
    for part in path.parts:
        sign = {'L': 1, 'R': -1, 'S': 0}[part.kind]
        if sign != 0:
            sweep = math.degrees(part.length / path.radius)
            n = max(1, int(math.ceil(sweep / a.arc_point_deg)))
        else:
            n = 1
        ds = part.length / n
        for k in range(n):
            x, y, th = D._advance(x, y, th, sign, ds, path.radius)
            s_acc += ds
            lat, lon = frame.latlon(x, y)
            alt = altitude_at(s_acc, path.length, start.alt_amsl, end.alt_amsl, a)
            name = {'L': 'sol yay', 'R': 'sağ yay', 'S': 'düz'}[part.kind]
            points.append(dict(name=f'{name} {k + 1}/{n}', lat=lat, lon=lon, alt_amsl=alt, course_deg=D.theta_to_course(th)))
    points[-1]['name'] = 'bacak sonu'
    return points


@dataclass
class CemberliAlcalis:
    """Uzun irtifa farkı için: hatta teğet bir çemberde alçalış, sonra hatta teğet çıkış (PX4 LOITER_TO_ALT, param4 > 0)."""
    ok: bool
    reason: str = None
    approach: Bacak = None        # çembere giriş bacağı
    center: tuple = None          # (lat, lon)
    radius_m: float = None
    counter_clockwise: bool = None
    tangent: tuple = None         # çıkış teğet noktası (lat, lon), hattın üstünde
    items: list = field(default_factory=list)


def plan_loiter_descent(start, line, terrain, a=PlanAyar(), next_along_m=0.0, min_straight_m=1500.0,
                        tangent_offsets=(-3500.0, -2500.0, -1500.0, 0.0, 1000.0, 2000.0, 3000.0)):
    """start: uçağın pozu. line: hattın noktası (Poz3: konum, hat yönü, hat irtifası).
    next_along_m: çemberden sonraki görev noktasının hat boyunca uzaklığı (0 = line noktasının kendisi, ör. 5000 = geçiş başı).
    Çember hatta teğettir; teğet noktası hat noktasından 'offset' kadar ileride (- ise geride). PX4 irtifaya inince
    çemberden sonraki noktaya teğet çıkar: o nokta hattın üstünde olduğu için çıkış yolu hattın kendisidir.
    Teğetten sonraki noktaya en az min_straight_m düz hat kalmalı (hizalanma). Çember ve çevresi hat irtifasında
    araziyi temizlemeli; çembere giriş bacağı plan_leg'den. Teğet noktası hat noktasına en yakın olan adaylar önce."""
    frame = YerelCerceve(line.lat, line.lon)
    th = D.course_to_theta(line.course_deg)
    ux, uy = math.cos(th), math.sin(th)          # hat yönü
    lx, ly = -uy, ux                             # sola dik
    r = max(a.loiter_radius_m, D.turn_radius(a.airspeed, a.bank_deg) * a.loiter_turn_factor)
    nx_, ny_ = ux * next_along_m, uy * next_along_m
    nlat, nlon = frame.latlon(nx_, ny_)
    reasons = []
    for off in sorted(tangent_offsets, key=abs):
        if next_along_m - off < min_straight_m:
            continue
        for ccw in (True, False):
            tag = f'teğet {off:+.0f} m {"sol" if ccw else "sağ"}'
            tx, ty = ux * off, uy * off
            side = 1.0 if ccw else -1.0              # saat tersinde merkez hattın solunda
            cx, cy = tx + side * lx * r, ty + side * ly * r
            if not loiter_clear(terrain, frame, cx, cy, line.alt_amsl, replace_radius(a, r)):
                reasons.append(f'{tag}: çember araziyi temizlemiyor')
                continue
            sx, sy = frame.xy(start.lat, start.lon)
            ang = math.atan2(sy - cy, sx - cx)
            px, py = cx + r * math.cos(ang), cy + r * math.sin(ang)
            entry_theta = ang + (math.pi / 2 if ccw else -math.pi / 2)
            plat, plon = frame.latlon(px, py)
            entry = Poz3(plat, plon, D.theta_to_course(entry_theta), start.alt_amsl)
            approach = plan_leg(start, entry, terrain, a, require_loiter=False)
            if not approach.ok:
                reasons.append(f'{tag}: giriş {approach.reason}')
                continue
            clat, clon = frame.latlon(cx, cy)
            tlat, tlon = frame.latlon(tx, ty)
            items = list(approach.items) + [dict(name='Çember atarak alçalış', kind='loiter_to_alt', lat=clat, lon=clon,
                                                 alt_amsl=line.alt_amsl, radius=-r if ccw else r, exit_tangent=True),
                                            dict(name='Hat', lat=nlat, lon=nlon, alt_amsl=line.alt_amsl,
                                                 course_deg=line.course_deg)]
            return CemberliAlcalis(True, None, approach, (clat, clon), r, ccw, (tlat, tlon), items)
    return CemberliAlcalis(False, '; '.join(reasons[:4]) or 'aday yok')


def replace_radius(a, r):
    from dataclasses import replace
    return replace(a, loiter_radius_m=r)


def to_mission(items, home_amsl, mavlink):
    """Planlayıcı noktalarını Baglanti.upload biçimine çevirir (irtifa eve göre). mavlink: pymavlink.mavutil.mavlink."""
    out = []
    for it in items:
        rel = it['alt_amsl'] - home_amsl
        if it.get('kind') == 'loiter_to_alt':
            out.append(dict(name=it['name'], cmd=mavlink.MAV_CMD_NAV_LOITER_TO_ALT, lat=it['lat'], lon=it['lon'], alt=rel,
                            # param1 = 1 (heading required): PX4 çıkışı uçak sonraki noktaya dönünce yapar (mission_block.cpp
                            # 441-462). 0 iken irtifaya iner inmez nerede olursa çıkıyordu (p1-p4: teğetin ~700 m gerisi, 74-125°)
                            radius=it['radius'], param1=1 if it.get('exit_tangent') else 0,
                            param4=1 if it.get('exit_tangent') else 0))
        else:
            out.append(dict(name=it['name'], cmd=mavlink.MAV_CMD_NAV_WAYPOINT, lat=it['lat'], lon=it['lon'], alt=rel))
    return out


PX4_PARAMS = ('FW_AIRSPD_TRIM', 'FW_R_LIM', 'NAV_ACC_RAD', 'FW_T_SINK_R_SP', 'FW_T_CLMB_R_SP')


def ayar_from_px4(params, corridor_extra_m=60.0, loiter_bank_tan_ratio=1.0 / 1.6, **overrides):
    """PlanAyar'ı PX4 parametrelerinden hesaplar; parametre değişirse plan kendiliğinden uyar.

    hız              = FW_AIRSPD_TRIM
    dönüş yarıçapı   = v² / (g·tan FW_R_LIM)                 (Dubins; radius_margin ayrıca)
    koridor          = NAV_ACC_RAD + corridor_extra_m        (PX4 noktaya bu kadar kala döner; p1: 451 m sapma)
    alçalış eğimi    = FW_T_SINK_R_SP / v                    (p1: 2,06 m/s ölçüldü, parametre 2,0)
    tırmanış eğimi   = FW_T_CLMB_R_SP / v
    çember yarıçapı  = v² / (g·tan FW_R_LIM · loiter_bank_tan_ratio): çemberde yatış payı kalsın
                       (p1: tan oranı 0,86 ile 522 m çember tutulamadı, 846 m uçuldu; 1/1,6 → yatış ~16°)
    params: {ad: değer}. Eksik parametre KeyError verir: plan sessizce varsayılana düşmesin."""
    v = float(params['FW_AIRSPD_TRIM'])
    bank = float(params['FW_R_LIM'])
    a = PlanAyar(airspeed=v, bank_deg=bank,
                 corridor_m=float(params['NAV_ACC_RAD']) + corridor_extra_m,
                 descent_gradient=float(params['FW_T_SINK_R_SP']) / v,
                 climb_gradient=float(params['FW_T_CLMB_R_SP']) / v,
                 loiter_turn_factor=1.0 / loiter_bank_tan_ratio)
    from dataclasses import replace
    return replace(a, **overrides) if overrides else a


def read_px4_params(read_param, names=PX4_PARAMS):
    """read_param(ad) -> değer ya da None (ör. Baglanti.read_param). Okunamayan varsa RuntimeError."""
    values = {n: read_param(n) for n in names}
    missing = [n for n, v in values.items() if v is None]
    if missing:
        raise RuntimeError(f'PX4 parametreleri okunamadı: {", ".join(missing)}')
    return values
