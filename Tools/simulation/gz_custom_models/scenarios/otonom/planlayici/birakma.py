"""Su bırakma geçişi ve sonrası: yangının üstünden hangi yönde geçilir, bırakmadan sonra ne kadar düz gidilir, piste
nasıl dönülür.

fire_attack.py'deki plan_run ve plan_exit buraya taşındı (2026-11-27; kural: plan kuralları planlayıcıda). Kurallar:
- Yaklaşma: geçiş yönünde commit_range_m geriden, seyir irtifasından bırakma irtifasına doğrusal alçalış; her örnekte
  yerden en az margin_m.
- Düz çıkış: bırakmadan sonra exit_runs_m uzunluklarıyla düz ve tırmanarak. İrtifa PX4'ün uçacağı gibi (climb_profile):
  bırakma geçişi noktası hedefin accept_m ötesindedir; istenen irtifa o noktadan sonra çıkış noktasına accept_m kala
  noktanın irtifasına doğrusal yükselir, uçak en çok climb_gradient ile tırmanır. Her örnekte yerden en az margin_m.
  Çıkış noktasının irtifası seyirdir, uçak oraya yetişemiyorsa yetişebildiği: PX4 noktayı irtifa farkı NAV_FW_ALT_RAD
  içinde değilse bırakmaz ve orada kendi seçtiği yönde döner.
- Dönüş: çıkış noktasından piste dönüş return_fn ile, uçağın dönüş başındaki (çıkış noktasına accept_m kala) tahmini
  irtifasıyla denetlenir. Dalaman'da return_fn pist.plan_runway_return'dür: ilk dönüş Dubins yolu, iki yanda koridor, PX4'ün
  noktayı erken bırakması. Dönüş yoksa daha kısa çıkış, o da yoksa başka geçiş yönü (tercih edilen yöne yakından uzağa)
  denenir.
2026-11-27 gösterisinde çıkıştan sonraki ~180° dönüş denetlenmiyordu (yalnız çıkış noktası çevresinde 500 m halka);
uçak kısa yönden döndü ve sırta çarptı.

Arazi bir fonksiyondur: terrain(lat, lon) -> m (deniz seviyesine göre) ya da None. Simülasyon ya da MAVLink bilmez.
return_fn(nokta, yön, istenen irtifa amsl, gerçek irtifa amsl) -> (ok, dönüş planı, gerekçe).
"""
import math
from dataclasses import dataclass, field

__all__ = ['BirakmaAyar', 'Cikis', 'Gecis', 'climb_profile', 'plan_exit', 'plan_run', 'ring_return']

R_EARTH_LAT = 111320.0


def _along(lat, lon, bearing_deg, distance, right=0.0):
    b = math.radians(bearing_deg)
    north, east = distance * math.cos(b) - right * math.sin(b), distance * math.sin(b) + right * math.cos(b)
    return lat + north / R_EARTH_LAT, lon + east / (R_EARTH_LAT * math.cos(math.radians(lat)))


def _dist(a, b):
    return math.hypot((b[0] - a[0]) * R_EARTH_LAT, (b[1] - a[1]) * R_EARTH_LAT * math.cos(math.radians(a[0])))


@dataclass(frozen=True)
class BirakmaAyar:
    commit_range_m: float = 3000.0      # COMMIT_RANGE_M: bırakma irtifasına alçalış bu uzaklıktan başlar
    accept_m: float = 440.0             # NAV_ACC_RAD: PX4 noktaya bu kadar kala sonrakine geçer
    margin_m: float = 50.0              # ROUTE_MIN_MARGIN_M
    step_m: float = 100.0               # ROUTE_STEP_M
    climb_gradient: float = 3.45 / 45.0  # ölçülen seyir tırmanışı (m/m)
    exit_runs_m: tuple = (3000.0, 2500.0, 2000.0, 1500.0, 1000.0)
    max_offset_deg: int = 90            # geçiş yönü tercih edilenden en çok bu kadar sapar
    offset_step_deg: int = 10


@dataclass
class Cikis:
    ok: bool
    exit_m: float = None
    point: tuple = None                 # çıkış noktası (lat, lon)
    alt_amsl: float = None              # dönüş başında (çıkış noktasına accept_m kala) beklenen irtifa
    point_alt_amsl: float = None        # çıkış noktasının görev irtifası: seyir, uçak yetişemiyorsa yetişebildiği
    ret: object = None                  # return_fn'in döndürdüğü dönüş planı
    reason: str = None
    rejected: list = field(default_factory=list)   # [(uzunluk, gerekçe)]


@dataclass
class Gecis:
    ok: bool
    bearing_deg: float = None
    exit: Cikis = None
    reason: str = None
    rejected: list = field(default_factory=list)   # [(yön, gerekçe)]


def climb_profile(x, exit_m, run_amsl, cruise_amsl, ayar=BirakmaAyar()):
    """Hedeften x metre ötede (geçiş yönünde) PX4'ün uçacağı irtifa, deniz seviyesine göre.

    Uçak hedefte bırakma geçişi noktasını (hedef + accept_m) bırakıp çıkış noktasına yönelir. İki nokta arasında PX4
    istenen irtifayı doğrusal değiştirir: önceki noktayı geçince başlar, sonraki noktaya accept_m kala biter; noktalar
    accept_m'den yakınsa sonraki noktanın irtifası hemen istenir. Uçak en çok climb_gradient ile tırmanır."""
    a = ayar.accept_m
    if exit_m - a > a:
        start = a
        sp = run_amsl + (cruise_amsl - run_amsl) * min(1.0, max(0.0, (x - a) / (exit_m - 2.0 * a)))
    else:
        start = 0.0
        sp = cruise_amsl
    if cruise_amsl <= run_amsl:
        return sp
    return min(sp, run_amsl + ayar.climb_gradient * max(0.0, x - start))


def _run_clear(target, bearing_deg, exit_m, run_amsl, cruise_amsl, terrain, ayar):
    """Bırakmadan çıkış noktasına düz yol: her örnekte yerden en az margin_m. Sorun yoksa None, varsa gerekçe."""
    steps = max(1, int(exit_m / ayar.step_m))
    for i in range(1, steps + 1):
        x = exit_m * i / steps
        ground = terrain(*_along(target[0], target[1], bearing_deg, x))
        if ground is None:
            return 'arazi verisi yok'
        z = climb_profile(x, exit_m, run_amsl, cruise_amsl, ayar)
        if ground + ayar.margin_m > z:
            return f'düz çıkışın {x:.0f}. m\'sinde arazi {ground:.0f} m, yükseklik {z:.0f} m'
    return None


def _approach_clear(target, bearing_deg, run_amsl, cruise_amsl, terrain, ayar):
    """Yaklaşma: commit_range_m geriden seyir irtifasından hedefte bırakma irtifasına doğrusal. None ya da gerekçe."""
    start = _along(target[0], target[1], bearing_deg, -ayar.commit_range_m)
    length = _dist(start, target)
    steps = max(1, int(length / ayar.step_m))
    for i in range(1, steps + 1):
        k = i / steps
        ground = terrain(start[0] + (target[0] - start[0]) * k, start[1] + (target[1] - start[1]) * k)
        if ground is None:
            return 'arazi verisi yok'
        z = cruise_amsl + (run_amsl - cruise_amsl) * k * length / ayar.commit_range_m
        if ground + ayar.margin_m > z:
            return f'yaklaşmanın {k * length:.0f}. m\'sinde arazi {ground:.0f} m, yükseklik {z:.0f} m'
    return None


def plan_exit(target, bearing_deg, run_amsl, cruise_amsl, terrain, return_fn, ayar=BirakmaAyar()):
    """Bırakmadan sonra en uzun güvenli düz çıkış ve oradan piste dönüş.

    target: (lat, lon[, ...]) hedef; bearing_deg: geçiş yönü; run_amsl: bırakma irtifası; cruise_amsl: çıkışta istenen
    seyir irtifası. Dönüş: Cikis. Düz yol ya da dönüş hiçbir uzunlukta tutmazsa ok False, rejected her uzunluğun gerekçesi."""
    rejected = []
    for exit_m in ayar.exit_runs_m:
        # PX4 çıkış noktasını irtifa farkı NAV_FW_ALT_RAD içinde değilse bırakmaz, orada döner: noktanın irtifası uçağın
        # oraya kadar tırmanabileceğini aşmaz (tırmanış bırakma geçişi noktasını geçince başlar)
        point_alt = min(cruise_amsl, run_amsl + ayar.climb_gradient * max(0.0, exit_m - ayar.accept_m))
        why = _run_clear(target, bearing_deg, exit_m, run_amsl, point_alt, terrain, ayar)
        if why:
            rejected.append((exit_m, why))
            continue
        point = _along(target[0], target[1], bearing_deg, exit_m)
        alt = climb_profile(exit_m - ayar.accept_m, exit_m, run_amsl, point_alt, ayar)
        ok, ret, why = return_fn(point, bearing_deg, cruise_amsl, alt)
        if not ok:
            rejected.append((exit_m, f'dönüş: {why}'))
            continue
        return Cikis(True, exit_m, point, alt, point_alt, ret, rejected=rejected)
    return Cikis(False, reason='; '.join(f'{m:.0f} m: {w}' for m, w in rejected), rejected=rejected)


def plan_run(target, preferred_deg, run_amsl, cruise_amsl, terrain, return_fn, ayar=BirakmaAyar()):
    """Tercih edilen yöne en yakın, yaklaşması, düz çıkışı ve piste dönüşü güvenli geçiş yönü. Dönüş: Gecis."""
    rejected = []
    step = ayar.offset_step_deg
    for offset in sorted(range(-ayar.max_offset_deg, ayar.max_offset_deg + 1, step), key=abs):
        bearing = (preferred_deg + offset) % 360.0
        why = _approach_clear(target, bearing, run_amsl, cruise_amsl, terrain, ayar)
        if why:
            rejected.append((bearing, why))
            continue
        exit_plan = plan_exit(target, bearing, run_amsl, cruise_amsl, terrain, return_fn, ayar)
        if exit_plan.ok:
            return Gecis(True, bearing, exit_plan, rejected=rejected)
        rejected.append((bearing, exit_plan.reason))
    return Gecis(False, reason=f'{len(rejected)} yönün hiçbiri tutmadı', rejected=rejected)


def ring_return(terrain, return_point, home_amsl, ring_m=500.0, min_rel_m=300.0, above_terrain_m=200.0,
                ayar=BirakmaAyar()):
    """Pist yönleri tanımsız dünyalar (sabit dönüş deseni) için eski dönüş denetimi, return_fn olarak.

    Çıkış noktası çevresinde ring_m halkada (30° aralık) arazi + margin_m istenen irtifanın altında kalmalı; dönüşün ilk
    noktasının yüksekliği (kalkış noktasına göre) yol üstündeki arazinin above_terrain_m üstü, en az min_rel_m. Dönüş
    yönünü ve yayını denetlemez. Plan olarak bu yüksekliği döner."""
    def check(point, course_deg, hold_amsl, actual_amsl):
        ring = [terrain(*_along(point[0], point[1], b, ring_m)) for b in range(0, 360, 30)]
        if any(h is None for h in ring):
            return False, None, 'arazi verisi yok'
        if max(ring) + ayar.margin_m > hold_amsl:
            return False, None, f'dönüş halkasında arazi {max(ring):.0f} m'
        length = _dist(point, return_point)
        steps = max(1, int(length / ayar.step_m))
        back = [terrain(point[0] + (return_point[0] - point[0]) * i / steps, point[1] + (return_point[1] - point[1]) * i / steps)
                for i in range(1, steps + 1)]
        if any(h is None for h in back):
            return False, None, 'arazi verisi yok'
        return True, max(min_rel_m, max(back) + above_terrain_m - home_amsl), None
    return check
