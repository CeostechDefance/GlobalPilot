"""Su bırakma geçişi ve sonrası: yangının üstünden hangi yönde geçilir, bırakmadan sonra ne kadar düz gidilir, piste
nasıl dönülür.

fire_attack.py'deki plan_run ve plan_exit buraya taşındı (2026-11-27; kural: plan kuralları planlayıcıda). Kurallar:
- Yaklaşma: geçiş yönünde commit_range_m geriden, seyir irtifasından bırakma irtifasına doğrusal alçalış; her örnekte
  yerden en az margin_m.
- Düz çıkış (kullanıcı kararı 2026-11-27, yangın söndürme uçaklarındaki gibi): bırakmadan sonra düz ve tırmanarak,
  güvenli irtifaya tırmanmaya yetecek kadar. Güvenli irtifa yereldir: çıkış yolunun ve dönüş alanının en yüksek
  arazisinin safe_above_m üstü, seyirden yüksek değil (exit_lengths); uzaktaki bir sırt için gereken irtifaya dönüş
  yolunda tırmanılır. Dönüş tutmazsa çıkış exit_step_m adımlarla exit_max_m'ye kadar uzar. İrtifa PX4'ün uçacağı gibi
  (climb_profile): bırakma geçişi noktası hedefin accept_m ötesindedir; istenen irtifa o noktadan sonra çıkış noktasına
  accept_m kala noktanın irtifasına doğrusal yükselir, uçak en çok climb_gradient ile tırmanır. Her örnekte yerden en az
  margin_m. Çıkış noktasının irtifası güvenli irtifadır: PX4 noktayı irtifa farkı NAV_FW_ALT_RAD içinde değilse
  bırakmaz, yani uçak güvenli irtifaya çıkmadan dönmez.
- Dönüş: çıkış noktasından piste dönüş return_fn ile, uçağın dönüş başındaki (çıkış noktasına accept_m kala) tahmini
  irtifasıyla denetlenir. return_fn pist.plan_runway_return'dür (fire_attack): ilk dönüş Dubins yolu, iki yanda koridor, PX4'ün
  noktayı erken bırakması. Dönüş yoksa daha uzun çıkış, o da yoksa başka geçiş yönü (tercih edilen yöne yakından uzağa)
  denenir.
2026-11-27 gösterisinde çıkıştan sonraki ~180° dönüş denetlenmiyordu (yalnız çıkış noktası çevresinde 500 m halka);
uçak kısa yönden döndü ve sırta çarptı.

Arazi bir fonksiyondur: terrain(lat, lon) -> m (deniz seviyesine göre) ya da None. Simülasyon ya da MAVLink bilmez.
return_fn(nokta, yön, istenen irtifa amsl, gerçek irtifa amsl) -> (ok, dönüş planı, gerekçe).
"""
import math
from dataclasses import dataclass, field

from . import rota as R
from .geo import along, distance_m

__all__ = ['BirakmaAyar', 'Cikis', 'Gecis', 'climb_profile', 'exit_lengths', 'plan_exit', 'plan_run']

@dataclass(frozen=True)
class BirakmaAyar:
    commit_range_m: float = 3000.0      # COMMIT_RANGE_M: bırakma irtifasına alçalış bu uzaklıktan başlar
    accept_m: float = 440.0             # NAV_ACC_RAD: PX4 noktaya bu kadar kala sonrakine geçer
    margin_m: float = 50.0              # ROUTE_MIN_MARGIN_M
    step_m: float = 100.0               # ROUTE_STEP_M
    climb_gradient: float = 3.45 / 45.0  # ölçülen seyir tırmanışı (m/m)
    safe_above_m: float = 200.0         # güvenli irtifa yerel arazinin bu kadar üstü (CRUISE_ABOVE_TERRAIN_M)
    safe_width_m: float = 500.0         # yerel arazi: çıkış yolunun iki yanında ve ötesinde (dönüş koridoru)
    safe_step_m: float = 30.0           # yerel arazi enine bu aralıkla (dönüş koridoru gibi: seyrek örnek dar tepeyi kaçırır)
    exit_step_m: float = 250.0          # dönüş tutmazsa düz çıkış bu adımlarla uzar
    exit_max_m: float = 3000.0          # düz çıkış dönüş için en çok bu kadar uzar
    max_offset_deg: int = 90            # geçiş yönü tercih edilenden en çok bu kadar sapar
    offset_step_deg: int = 10


@dataclass
class Cikis:
    ok: bool
    exit_m: float = None
    point: tuple = None                 # çıkış noktası (lat, lon)
    alt_amsl: float = None              # dönüş başında (çıkış noktasına accept_m kala) beklenen irtifa
    point_alt_amsl: float = None        # çıkış noktasının görev irtifası: güvenli irtifa
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


def climb_profile(x, exit_m, run_amsl, point_amsl, ayar=BirakmaAyar()):
    """Hedeften x metre ötede (geçiş yönünde) PX4'ün uçacağı irtifa, deniz seviyesine göre.

    Uçak hedefte bırakma geçişi noktasını (hedef + accept_m) bırakıp çıkış noktasına yönelir. İki nokta arasında PX4
    istenen irtifayı doğrusal değiştirir: önceki noktayı geçince başlar, sonraki noktaya accept_m kala biter; noktalar
    accept_m'den yakınsa sonraki noktanın irtifası hemen istenir. Uçak en çok climb_gradient ile tırmanır."""
    a = ayar.accept_m
    if exit_m - a > a:
        start = a
        sp = run_amsl + (point_amsl - run_amsl) * min(1.0, max(0.0, (x - a) / (exit_m - 2.0 * a)))
    else:
        start = 0.0
        sp = point_amsl
    return min(sp, run_amsl + ayar.climb_gradient * max(0.0, x - start))


def _run_clear(target, bearing_deg, exit_m, run_amsl, point_amsl, terrain, ayar):
    """Bırakmadan çıkış noktasına düz yol: her örnekte yerden en az margin_m. Sorun yoksa None, varsa gerekçe."""
    steps = max(1, int(exit_m / ayar.step_m))
    for i in range(1, steps + 1):
        x = exit_m * i / steps
        ground = terrain(*along(target[0], target[1], bearing_deg, x))
        if ground is None:
            return 'arazi verisi yok'
        z = climb_profile(x, exit_m, run_amsl, point_amsl, ayar)
        if ground + ayar.margin_m > z:
            return f'düz çıkışın {x:.0f}. m\'sinde arazi {ground:.0f} m, yükseklik {z:.0f} m'
    return None


def _approach_clear(target, bearing_deg, run_amsl, cruise_amsl, terrain, ayar):
    """Yaklaşma: commit_range_m geriden seyir irtifasından hedefte bırakma irtifasına doğrusal. None ya da gerekçe."""
    start = along(target[0], target[1], bearing_deg, -ayar.commit_range_m)
    length = distance_m(start[0], start[1], target[0], target[1])
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


def exit_lengths(target, bearing_deg, run_amsl, cruise_amsl, terrain, ayar=BirakmaAyar()):
    """Denenecek düz çıkışlar kısadan uzuna, gerektikçe: (hedeften uzunluk, güvenli irtifa); arazi verisi biterse durur.

    Güvenli irtifa: düz çıkış yolunun ve dönüş alanının (yolun iki yanında ve çıkış noktasının ötesinde safe_width_m,
    rota.corridor_max) en yüksek arazisinin safe_above_m üstü, seyirden yüksek, bırakma irtifasından düşük değil
    (bırakmadan sonra alçalış yok). İlk uzunluk uçağın güvenli irtifaya yetiştiği en kısa uzunluk (step_m katı, en az
    2 * accept_m; tırmanış climb_profile'daki gibi). Sonrakiler exit_step_m adımlarla exit_max_m'ye kadar, güvenli
    irtifaya yetişilenler. Güvenli irtifa seyri aşmadığından ilk uzunluk her zaman bulunur."""
    frame, step = R.YerelCerceve(target[0], target[1]), ayar.step_m
    tops = []       # tops[i]: hedeften i * step_m'ye kadar yolun iki yanındaki en yüksek arazi
    length, found = math.ceil(2.0 * ayar.accept_m / step) * step, False
    while not found or length <= ayar.exit_max_m:
        k = math.ceil((length + ayar.safe_width_m) / step)
        while len(tops) <= k:
            x, y = frame.xy(*along(target[0], target[1], bearing_deg, len(tops) * step))
            top = R.corridor_max(terrain, frame, x, y, bearing_deg, ayar.safe_width_m, ayar.safe_step_m)
            if top is None:
                return
            tops.append(max(tops[-1], top) if tops else top)
        alt = max(run_amsl, min(cruise_amsl, tops[k] + ayar.safe_above_m))
        if climb_profile(length, length, run_amsl, alt, ayar) >= alt - 1e-6:
            found = True
            yield length, alt
        length += ayar.exit_step_m if found else step


def plan_exit(target, bearing_deg, run_amsl, cruise_amsl, terrain, return_fn, ayar=BirakmaAyar()):
    """Bırakmadan sonra güvenli irtifaya tırmanmaya yeten en kısa düz çıkış (exit_lengths) ve oradan piste dönüş.

    target: (lat, lon[, ...]) hedef; bearing_deg: geçiş yönü; run_amsl: bırakma irtifası; cruise_amsl: çıkışta istenen
    seyir irtifası. Dönüş: Cikis. Düz yol ya da dönüş hiçbir uzunlukta tutmazsa ok False, rejected her uzunluğun gerekçesi
    (arazi verisi yoksa boş)."""
    rejected = []
    for exit_m, point_alt in exit_lengths(target, bearing_deg, run_amsl, cruise_amsl, terrain, ayar):
        # noktanın irtifası güvenli irtifa, uçak oraya bu uzunlukta yetişir: PX4 noktayı irtifa farkı NAV_FW_ALT_RAD
        # içinde değilse bırakmaz, yani uçak güvenli irtifaya çıkmadan dönmez
        why = _run_clear(target, bearing_deg, exit_m, run_amsl, point_alt, terrain, ayar)
        if why:
            rejected.append((exit_m, why))
            continue
        point = along(target[0], target[1], bearing_deg, exit_m)
        alt = climb_profile(exit_m - ayar.accept_m, exit_m, run_amsl, point_alt, ayar)
        ok, ret, why = return_fn(point, bearing_deg, cruise_amsl, alt)
        if not ok:
            rejected.append((exit_m, f'dönüş: {why}'))
            continue
        return Cikis(True, exit_m, point, alt, point_alt, ret, rejected=rejected)
    return Cikis(False, reason='; '.join(f'{m:.0f} m: {w}' for m, w in rejected) or 'arazi verisi yok', rejected=rejected)


def plan_run(target, preferred_deg, run_amsl, cruise_amsl, terrain, return_fn, ayar=BirakmaAyar(), bearings=None):
    """Tercih edilen yöne en yakın, yaklaşması, düz çıkışı ve piste dönüşü güvenli geçiş yönü. Dönüş: Gecis; reason ilk
    üç elenen yönün kısa gerekçesi. bearings verilirse yalnız bu yönler sırayla denenir ve yaklaşma denetlenmez
    (bırakmaya yaklaşırken düzeltme: yeni yön, olmazsa uçulan yön; uçak zaten yaklaşmada)."""
    check_approach = bearings is None
    if check_approach:
        offsets = sorted(range(-ayar.max_offset_deg, ayar.max_offset_deg + 1, ayar.offset_step_deg), key=abs)
        bearings = [(preferred_deg + offset) % 360.0 for offset in offsets]
    rejected = []
    for bearing in bearings:
        why = _approach_clear(target, bearing, run_amsl, cruise_amsl, terrain, ayar) if check_approach else None
        if why:
            rejected.append((bearing, why))
            continue
        exit_plan = plan_exit(target, bearing, run_amsl, cruise_amsl, terrain, return_fn, ayar)
        if exit_plan.ok:
            return Gecis(True, bearing, exit_plan, rejected=rejected)
        rejected.append((bearing, exit_plan.reason))
    return Gecis(False, reason='; '.join(f'{b:.0f}°: {w[:160]}' for b, w in rejected[:3]), rejected=rejected)
