"""Piste dönüş deseni: iki iniş yönünden yolu kısa olanı seçer (kullanıcı, 2026-11-11).

Kural: uçak piste hangi yönden inmek daha kısa yol tutuyorsa o yönden iner; pistin ötesine gidip geri dolaşmaz.
Desen uçağın bulunduğu yerden çizilir:
- düz giriş: uçak son yaklaşma çizgisine az bir açıyla geliyorsa doğrudan son yaklaşma başına,
- taban girişi: değilse uçağın bulunduğu yandaki (ya da öbür yandaki) taban noktasına, oradan son yaklaşmaya.
Arazi payı tutmayan aday elenir. Hiçbir aday tutmazsa desen boş döner (ok False), çağıran eski deseni kullanır.

İlk dönüş (2026-11-27): uçağın bulunduğu yerden ve yönünden (course_deg) desenin ilk noktasına (taban noktası ya da
son yaklaşma başı) giden yol Dubins yoludur ve PX4'ün uçacağı gibi denetlenir. 2026-11-27 gösterisinde yalnız düz çizgi
denetlenmişti; uçak ~180° dönüşte kısa yönü (sağ) seçti ve denetlenmeyen sırta çarptı.
- Yay noktaları göreve yazılır (transition): PX4 dönüşü planlanan yönde yapar.
- Arazi yolun iki yanında koridorla (corridor_m) örneklenir: PX4 noktaya accept_m kala sonrakine geçer ve köşeleri keser.
  Başlangıç bir görev noktasıysa (early_m) dönüş o kadar erken de başlayabilir: erken başlayan aynı yönlü yol da denetlenir.
- İrtifa ilk yay boyunca korunur (start_rel_m), sonra geç alçalış. Denetimde PX4'ün irtifa davranışı kullanılır: iki nokta
  arası doğrusal ve sonraki noktanın irtifasına ona accept_m kala varılır; noktalar accept_m'den yakınsa doğrudan
  sonrakinin irtifası istenir. Uçak en çok climb_gradient ile tırmanır, en çok sink_max_gradient ile alçalır (en kötü durum).
Desenin geri kalanı (taban → son yaklaşma → iniş) düz bacaklarla denetlenir.

Arazi bir fonksiyondur: terrain(lat, lon) -> m (deniz seviyesine göre) ya da None. Simülasyon ya da MAVLink bilmez.
"""
import math
from dataclasses import dataclass, field

from . import dubins as D
from . import rota as R
from .geo import along, bearing_deg, distance_m, turn_deg

__all__ = ['PistAyar', 'PistYonu', 'PistDeseni', 'plan_runway_return']

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
    # İlk dönüş (Dubins). Yarıçap = airspeed ve bank_deg'den, radius_margin ile (rota.PlanAyar ile aynı yöntem).
    airspeed: float = 45.0              # dönüş hızı; boş uçakta PX4 hızları ağırlıkla küçültür (800 kg: ~37 m/s)
    bank_deg: float = 25.0              # FW_R_LIM
    radius_margin: float = 1.25
    accept_m: float = 440.0             # NAV_ACC_RAD: PX4 noktaya bu kadar kala sonrakine geçer
    corridor_m: float = 500.0           # NAV_ACC_RAD + 60: uçulan yol Dubins yolundan en çok 451 m saptı (uçuş p1)
    corridor_step_m: float = 30.0       # arazi ızgarası ~30 m: daha seyrek örnek dar tepeyi kaçırır
    sample_m: float = 30.0              # yol boyunca örnek aralığı, aynı nedenle
    climb_gradient: float = 3.45 / 45.0     # ölçülen seyir tırmanışı (m/m)
    descent_gradient: float = 2.0 / 45.0    # FW_T_SINK_R_SP / hız: planlanan geç alçalışın eğimi
    sink_max_gradient: float = 5.0 / 37.0   # FW_T_SINK_MAX / boş uçak hızı: PX4'ün en hızlı alçalışı (denetimde en kötü)
    descent_max_gradient: float = 4.0 / 37.0  # planlanan alçalışın en diki: uçak noktaların irtifasına yetişsin
    alt_accept_m: float = 10.0          # NAV_FW_ALT_RAD: PX4 noktayı irtifa farkı bundan büyükken bırakmaz, orada döner
                                        # (2026-11-27 test 2: 300 m istenen ilk yay noktasına ~100 m'de varan uçak çember attı)
    arc_point_deg: float = 60.0         # dönüş yayında bu açıdan bir görev noktası
    # Denizden alçak başlayan dönüş (su bölümünden sonra): kalkış gibi, pay ve koridor dar başlar, uzaklıkla büyür
    # (arazi_rota.RotaAyar ile aynı kural). Yalnız başlangıç denizdeyse ve uçak istenen irtifanın altındaysa.
    sea_max_m: float = 5.0              # bunun altı deniz (LAND_MIN_AMSL_M)
    climbout_margin_m: float = 10.0     # başlangıçta pay; yol boyunca tırmanış eğiminin yarısıyla margin_m'ye büyür
    climbout_corridor_m: float = 60.0   # başlangıçta koridor yarı genişliği; climbout_corridor_slope ile corridor_m'ye
    climbout_corridor_slope: float = 0.3

    @property
    def radius(self):
        return D.turn_radius(self.airspeed, self.bank_deg) * self.radius_margin


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
    transition: list = field(default_factory=list)  # ilk dönüşün görev noktaları, points'ten önce: [(ad, (lat, lon), göreli)]
    turn: str = None                    # ilk dönüşün yönü: 'sola' | 'sağa' | None (dönüş yok)
    min_clearance_m: float = None       # ilk dönüşte (koridorla) en az arazi payı

    @property
    def ok(self):
        return bool(self.points)


def _leg_clear(terrain, home_amsl, a, b, alt_a, alt_b, ayar, margin=None, end_skip_m=0.0):
    """a'dan b'ye düz yolda yükseklik alt_a'dan alt_b'ye doğrusal; her örnekte yer + pay altında kalmamalı.
    end_skip_m: yolun son bu kadarı denetlenmez."""
    margin = ayar.margin_m if margin is None else margin
    length = distance_m(*a, *b)
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


def _first_turn(path):
    kind = next((p.kind for p in path.parts if p.kind != 'S'), None)
    return {'L': 'sola', 'R': 'sağa'}.get(kind)


def _profile(path, actual, hold, end, ayar):
    """Görev noktalarının irtifası (deniz seviyesine göre) yol boyunca; uçağın yetişebileceği biçimde, çünkü PX4 noktayı
    irtifa farkı alt_accept_m'den büyükken bırakmaz. Dönüş: (irtifa(s), alçalış başı s ya da None, gerekçe ya da None).
    Uçağın gerçek irtifasından (actual) climb_gradient ile hold'a tırmanış; ilk yay boyunca alçalış yok; sonra end'e geç
    alçalış (descent_gradient; yol yetmezse descent_max_gradient'e kadar dik). Sığmazsa irtifa None, gerekçe dolu."""
    length = path.length
    top_alt = max(hold, end, actual)

    def top(s):
        return min(top_alt, actual + ayar.climb_gradient * s)
    if top(length) < end - 1e-6:
        return None, None, f'tırmanış yetişmiyor: sonda {end - top(length):.0f} m eksik'
    if end >= top_alt - 1e-6:
        return top, None, None
    first_arc = min(length, path.parts[0].length) if path.parts and path.parts[0].kind != 'S' else 0.0
    rest = length - first_arc
    need = (top(first_arc) - end) / rest if rest > 1e-6 else math.inf
    gradient = max(ayar.descent_gradient, need)
    if gradient > ayar.descent_max_gradient + 1e-9:
        return None, None, f'alçalış sığmıyor: {top(first_arc) - end:.0f} m için {rest:.0f} m yol'

    def profile(s):
        return min(top(s), end + gradient * (length - s))
    # alçalış başı: tırmanış/tutma çizgisinin alçalış çizgisiyle kesiştiği yer
    s_flat = (top_alt - actual) / ayar.climb_gradient if top_alt > actual else 0.0
    s_d = length - (top_alt - end) / gradient
    if s_d < s_flat:
        s_d = (end + gradient * length - actual) / (ayar.climb_gradient + gradient)
    return profile, max(first_arc, s_d), None


def _waypoints(path, profile, s_d, ayar):
    """Görev noktaları (s, x, y, irtifa): yaylarda en çok arc_point_deg aralıkla, parça sonlarında, düz parçada alçalış
    başında. Son nokta yolun sonudur (desenin ilk noktası)."""
    pts = []
    x, y, th = path.start.x, path.start.y, D.course_to_theta(path.start.course_deg)
    s0 = 0.0
    for part in path.parts:
        sign = D.SIGN[part.kind]
        if sign:
            n = max(1, int(math.ceil(math.degrees(part.length / path.radius) / ayar.arc_point_deg - 1e-9)))
            cuts = [part.length * k / n for k in range(1, n + 1)]
        elif s_d is not None and s0 + ayar.sample_m < s_d < s0 + part.length - ayar.sample_m:
            cuts = [s_d - s0, part.length]
        else:
            cuts = [part.length]
        done = 0.0
        for c in cuts:
            x, y, th = D.advance(x, y, th, sign, c - done, path.radius)
            done = c
            pts.append((s0 + c, x, y, profile(s0 + c), part.kind, s_d is not None and abs(s0 + c - s_d) < 1e-6))
        s0 += part.length
    return pts


def _fly(path, wps, actual_amsl, terrain, frame, ayar, climbout=False):
    """PX4'ün uçacağı irtifayla yol boyunca (iki yanda koridorla) arazi payı. (ok, en az pay, gerekçe).
    actual_amsl: uçağın başlangıçtaki gerçek irtifası; PX4 ilk noktaya irtifayı oradan değiştirmeye başlar (en alçak durum).
    climbout: denizden alçak başlangıç; pay ve koridor kalkıştaki gibi dar başlar, yol boyunca büyür.
    PX4 bir noktayı ancak yatayda accept_m'ye girince (ya da geçince) ve irtifa farkı alt_accept_m içindeyken bırakır;
    irtifası tutmadan geçilen noktada uçak döner: bu yol kabul edilmez."""
    k = 0
    px, py, palt = path.start.x, path.start.y, actual_amsl
    z, s_last, min_d = actual_amsl, 0.0, None
    worst = math.inf
    margin, width = ayar.margin_m, ayar.corridor_m
    samples = path.sample(ayar.sample_m)
    for s, x, y, course, _ in samples:
        while k < len(wps) - 1:
            _, cx, cy, calt = wps[k][:4]
            near = math.hypot(x - cx, y - cy) <= ayar.accept_m
            passed = (cx - px) * (x - cx) + (cy - py) * (y - cy) > 0.0
            if not (near or passed):
                break
            if abs(z - calt) > ayar.alt_accept_m:
                if passed:
                    return False, worst, (f'{k + 1}. noktada irtifa farkı {calt - z:+.0f} m: PX4 noktayı bırakmaz, '
                                          f'orada döner')
                break
            px, py, palt = cx, cy, calt
            k += 1
            min_d = None
        _, cx, cy, calt = wps[k][:4]
        chord = math.hypot(cx - px, cy - py)
        if chord > ayar.accept_m:
            # PX4 iki nokta arası (FixedwingPositionControl, irtifa birinci derece tutma): en küçük uzaklıkla doğrusal
            d = math.hypot(x - cx, y - cy)
            min_d = min(chord, d) if min_d is None else min(min_d, d)
            sp = calt if min_d <= ayar.accept_m else palt + (calt - palt) * (chord - min_d) / (chord - ayar.accept_m)
        else:
            sp = calt
        ds = s - s_last
        s_last = s
        z += max(-ayar.sink_max_gradient * ds, min(ayar.climb_gradient * ds, sp - z))
        if climbout:
            margin = min(ayar.margin_m, ayar.climbout_margin_m + 0.5 * ayar.climb_gradient * s)
            width = min(ayar.corridor_m, ayar.climbout_corridor_m + ayar.climbout_corridor_slope * s)
        h = R.corridor_max(terrain, frame, x, y, course, width, ayar.corridor_step_m)
        if h is None:
            return False, worst, f'{s:.0f}. m\'de arazi verisi yok'
        worst = min(worst, z - h)
        if z - h < margin:
            return False, worst, f'{s:.0f}. m\'de arazi {h:.0f} m, yükseklik {z:.0f} m'
    if abs(z - wps[-1][3]) > ayar.alt_accept_m:
        return False, worst, f'desenin ilk noktasında irtifa farkı {wps[-1][3] - z:+.0f} m: PX4 noktayı bırakmaz, orada döner'
    return True, worst, None


def _transition(start, course_deg, hold_amsl, actual_amsl, end, end_course, end_amsl, terrain, ayar, early_m=0.0,
                climbout=False):
    """Başlangıç pozundan desenin ilk noktasına (end, end_course) arazi payını tutan en kısa Dubins yolu.
    early_m > 0: başlangıç bir görev noktasıdır, PX4 onu early_m erken bırakır; 0: başlangıç uçağın kendi konumudur.
    Noktaların irtifası uçağın gerçek irtifasından (actual_amsl) yetişilebilir biçimde (_profile).
    Dönüş: (yol, görev noktaları [(ad, (lat, lon), irtifa amsl)] (son nokta hariç), en az pay, gerekçe).
    Yol yoksa yol None, gerekçe dolu."""
    frame = R.YerelCerceve(*start)
    sx, sy = frame.xy(*start)
    ex, ey = frame.xy(*end)
    end_pose = D.Poz(ex, ey, end_course)
    early = {}
    if early_m > 0.0:
        th = D.course_to_theta(course_deg)
        bx, by = sx - early_m * math.cos(th), sy - early_m * math.sin(th)
        early = {p.word: p for p in D.all_paths(D.Poz(bx, by, course_deg), end_pose, ayar.radius)}
    reasons = []
    for path in D.all_paths(D.Poz(sx, sy, course_deg), end_pose, ayar.radius):
        profile, s_d, why = _profile(path, actual_amsl, hold_amsl, end_amsl, ayar)
        if profile is None:
            reasons.append(f'{_first_turn(path) or "düz"} ({path.word}): {why}')
            continue
        wps = _waypoints(path, profile, s_d, ayar)
        ok, worst, why = _fly(path, wps, actual_amsl, terrain, frame, ayar, climbout)
        if ok and early:
            twin = early.get(path.word)
            tprofile, ts_d, twhy = _profile(twin, actual_amsl, hold_amsl, end_amsl, ayar) if twin else (None, None, None)
            if twin is None:
                ok, why = False, 'erken başlayan aynı yönlü dönüş yok'
            elif tprofile is None:
                ok, why = False, f'{early_m:.0f} m erken dönüşte {twhy}'
            else:
                ok, eworst, why = _fly(twin, _waypoints(twin, tprofile, ts_d, ayar), actual_amsl, terrain, frame, ayar,
                                       climbout)
                worst = min(worst, eworst)
                why = f'{early_m:.0f} m erken dönüşte {why}' if why else None
        if not ok:
            reasons.append(f'{_first_turn(path) or "düz"} ({path.word}): {why}')
            continue
        names = {'L': 'sol yay', 'R': 'sağ yay'}
        arcs = {}
        for wp in wps[:-1]:
            arcs[wp[4]] = arcs.get(wp[4], 0) + 1
        seen, points = {}, []
        for s, x, y, alt, kind, descent in wps[:-1]:
            seen[kind] = seen.get(kind, 0) + 1
            if kind == 'S':
                name = 'Piste dönüş: alçalış başı' if descent else 'Piste dönüş: düz'
            else:
                name = f'Piste dönüş: {names[kind]} {seen[kind]}/{arcs[kind]}' + (' (alçalış başı)' if descent else '')
            points.append((name, frame.latlon(x, y), alt))
        return path, points, worst, None
    return None, [], None, '; '.join(reasons[:2]) or 'aday yol yok'


def _candidates(start, rw, ayar):
    """Adaylar: (ad, desen noktaları, ilk noktadaki yön, ilk noktadan sonraki uzunluk)."""
    land = along(*rw.threshold, rw.heading_deg, ayar.land_along_m)
    final = along(*rw.threshold, rw.heading_deg, -ayar.final_m)
    glide_len = (ayar.pattern_rel_m - rw.land_rel_m) / math.tan(math.radians(ayar.final_angle_deg))
    glide = along(*rw.threshold, rw.heading_deg, ayar.land_along_m - glide_len)
    tail = [('Son yaklaşma', final, ayar.pattern_rel_m)]
    if distance_m(*final, *land) > glide_len + 1.0:
        tail.append(('Süzülüş başı', glide, ayar.pattern_rel_m))
    tail.append(('İniş', land, rw.land_rel_m))
    final_len = distance_m(*final, *land)
    out = []
    # düz giriş: açı küçük ve son yaklaşma başı en az iki dönüş yarıçapı uzakta (yakınsa yol uçulamaz, 7b)
    if turn_deg(bearing_deg(*start, *final), rw.heading_deg) <= ayar.straight_in_max_deg and \
            distance_m(*start, *final) >= 2.0 * ayar.turn_radius_m:
        out.append(('düz giriş', tail, rw.heading_deg, final_len))
    for side, label in ((1.0, 'taban (sağ)'), (-1.0, 'taban (sol)')):
        base = along(*final, rw.heading_deg, 0.0, side * ayar.base_m)
        base_course = bearing_deg(*base, *final)
        rest = distance_m(*base, *final) + final_len + ayar.turn_radius_m * math.radians(turn_deg(base_course, rw.heading_deg))
        out.append((label, [('Taban bacağı', base, ayar.pattern_rel_m)] + tail, base_course, rest))
    return out


def _pattern_clear(terrain, home_amsl, points, ayar):
    """Desenin ilk noktasından sonraki bacaklar: desen yüksekliğinde ayar.margin_m, süzülüşte glide_margin_m (son
    glide_check_end_m hariç). (ok, gerekçe)."""
    prev, prev_alt = points[0][1], points[0][2]
    for pname, p, alt in points[1:]:
        if pname == 'İniş':
            ok, why = _leg_clear(terrain, home_amsl, prev, p, prev_alt, alt, ayar, ayar.glide_margin_m, ayar.glide_check_end_m)
        else:
            ok, why = _leg_clear(terrain, home_amsl, prev, p, prev_alt, alt, ayar)
        if not ok:
            return False, f'{pname}: {why}'
        prev, prev_alt = p, alt
    return True, None


def plan_runway_return(start, start_rel_m, course_deg, runways, home_amsl, terrain, ayar=PistAyar(), actual_rel_m=None,
                       early_m=0.0):
    """En kısa piste dönüş deseni.

    start: (lat, lon) dönüşün başladığı nokta; start_rel_m: oradaki yükseklik (kalkış noktasına göre; ilk dönüş boyunca
    bu yükseklik istenir); course_deg: oradaki uçuş yönü; runways: PistYonu listesi (aynı pistin iki yönü); home_amsl: kalkış noktasının deniz
    seviyesine göre yüksekliği. actual_rel_m: uçağın başlangıçtaki gerçek yüksekliği (verilmezse start_rel_m; düşükse
    tırmanış climb_gradient ile sınırlı sayılır). early_m: başlangıç bir görev noktasıysa PX4 onu bu kadar erken bırakır
    (NAV_ACC_RAD); uçağın kendi konumundan planlanan dönüşte 0.
    Dönüş: PistDeseni. Hiçbir aday arazi payını tutmazsa points boş (ok False), rejected her adayın gerekçesini taşır.
    """
    hold = home_amsl + start_rel_m
    actual = home_amsl + (start_rel_m if actual_rel_m is None else actual_rel_m)
    here = terrain(*start)
    climbout = actual < hold and here is not None and here < ayar.sea_max_m   # denizden alçak başlangıç (su sonrası)
    frame = R.YerelCerceve(*start)
    sx, sy = frame.xy(*start)
    cands = []
    for rw in runways:
        for label, points, first_course, rest in _candidates(start, rw, ayar):
            ex, ey = frame.xy(*points[0][1])
            shortest = D.shortest(D.Poz(sx, sy, course_deg), D.Poz(ex, ey, first_course), ayar.radius).length
            cands.append((shortest + rest, rw, label, points, first_course, rest))
    cands.sort(key=lambda c: c[0])
    best, rejected = None, []
    for bound, rw, label, points, first_course, rest in cands:
        if best is not None and bound >= best.length_m:
            break                                         # kalan adayların en kısa yolu bile daha uzun
        name = f'{rw.name} {label}'
        ok, why = _pattern_clear(terrain, home_amsl, points, ayar)
        if not ok:
            rejected.append((name, why))
            continue
        path, transition, worst, why = _transition(start, course_deg, hold, actual, points[0][1], first_course,
                                                   home_amsl + points[0][2], terrain, ayar, early_m, climbout)
        if path is None:
            rejected.append((name, f'{points[0][0]}: ilk dönüş {why}'))
            continue
        length = path.length + rest
        if best is None or length < best.length_m:
            best = PistDeseni(rw, label, length, points, transition=[(n, p, alt - home_amsl) for n, p, alt in transition],
                              turn=_first_turn(path) if transition else None, min_clearance_m=worst)
    if best is None:
        return PistDeseni(None, None, float('inf'), [], rejected)
    best.rejected = rejected
    return best
