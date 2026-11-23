"""Otonom su geçişi modu: yangın bulunduktan sonra suya gidiş ve alçalış planı.

Kurallar (kullanıcı, 2026-11-10):
- Alçalış ancak yangın bulunduktan sonra, suya gidişte başlar.
- Uçak yerden her zaman en az ``margin_m`` (100 m) yukarıda kalır. Kıyı tepeleri de bu kurala dahildir.
- Alçalış, bu payı yolun tamamında koruyabildiği ilk noktada başlar. Yükseklik henüz fazlaysa (alçalırsa önündeki
  araziye 100 m'den fazla yaklaşacaksa) alçalışa başlamaz, düz devam eder.
- Düz alçalış suya kadar sığmıyorsa irtifa çemberde kaybedilir:
    * su yüzeyi çemberi ve çevresini (loiter_check_factor × yarıçap) tümüyle alacak kadar büyükse su üstünde,
    * değilse karada, çevresindeki en yüksek arazinin margin_m üstünde kalan en alçak irtifaya kadar.
  Karadaki çemberden sonra da suya giden düz yol aynı payla denetlenir.
- Su üstünde son yaklaşma irtifası entry_amsl'dir (su + APPROACH_ABOVE_WATER_M, 100 m).

Arazi ve su birer fonksiyondur: terrain(lat, lon) -> m (deniz seviyesine göre) ya da None (veri yok),
is_water(lat, lon) -> bool. Simülasyon ya da dosya bilmez; PX4'e çevirme rota.to_mission ile yapılır.
Bu modülün ürettiği son nokta, su bacağının (water_leg) devraldığı noktadır: su bacağı yüklenince hemen alçalmaya başlar.
"""
import math
from dataclasses import dataclass, field

from .rota import YerelCerceve, OnbellekliArazi

__all__ = ['SuGecisAyar', 'SuGecisi', 'plan_water_transit', 'OnbellekliArazi']


@dataclass(frozen=True)
class SuGecisAyar:
    margin_m: float = 100.0              # yerden en az bu kadar yukarıda (kullanıcı kuralı)
    descent_gradient: float = 2.0 / 45.0  # ölçülen: su bacağı yüklenince PX4 2 m/s @45 m/s alçalır
    step_m: float = 100.0                # yol örnek aralığı
    corridor_m: float = 0.0              # düz yolun iki yanında da arazi denetimi (0 = yalnız orta çizgi)
    min_transit_m: float = 1500.0        # su bacağı devralmadan önce suya dönüş için yer
    accept_m: float = 440.0              # NAV_ACC_RAD: PX4 noktayı bu kadar erken kabul eder
    circle_radius_m: float = 500.0       # 45 m/s, 25° yatış: 443 m; pay ile
    check_factor: float = 1.7            # çember denetimi bu kat yarıçapa kadar
    water_circle_along_m: float = 1500.0  # su üstü çember merkezi: giriş noktasından koridor boyunca
    land_circle_step_m: float = 500.0    # kara çemberi adayları bu aralıkla, suya yakından uzağa
    land_circle_search_m: float = 8000.0  # giriş yakınsa adaylar uçağın gerisinde ve yanlarında da, girişten bu kadar
    water_slack_m: float = 1000.0        # karada çemberden sonra alçalış girişten bu kadar sonra (su üstünde) bitebilir:
                                         # kıyıda arazi + pay giriş irtifasından yüksekse son metreler suyun üstünde inilir.
                                         # Su bacağının yaklaşması 5 km (100 m için ~2,3 km yeter), fazlası sığar


@dataclass
class SuGecisi:
    ok: bool
    mode: str = None                     # 'düz alçalış' | 'su üstünde çember' | 'karada çember'
    reason: str = None
    descent_start_m: float = None        # düz alçalış: uçaktan bu kadar sonra
    floor_amsl: float = None             # çemberde inilen irtifa
    items: list = field(default_factory=list)  # dict(name, lat, lon, alt_amsl[, kind='loiter_to_alt', radius])


def _bearing(frame, a, b):
    ax, ay = frame.xy(*a)
    bx, by = frame.xy(*b)
    return math.degrees(math.atan2(bx - ax, by - ay)) % 360.0


def _along(frame, p, bearing_deg, dist, right=0.0):
    x, y = frame.xy(*p)
    br = math.radians(bearing_deg)
    return frame.latlon(x + dist * math.sin(br) + right * math.cos(br), y + dist * math.cos(br) - right * math.sin(br))


def _samples(terrain, frame, a, b, ayar):
    """(uzaklık, en yüksek arazi) a'dan b'ye her step_m'de; veri yoksa None."""
    ax, ay = frame.xy(*a)
    bx, by = frame.xy(*b)
    length = math.hypot(bx - ax, by - ay)
    n = max(1, int(math.ceil(length / ayar.step_m)))
    ux, uy = ((bx - ax) / length, (by - ay) / length) if length > 0 else (0.0, 0.0)
    offsets = [0.0] if ayar.corridor_m <= 0 else [-ayar.corridor_m, 0.0, ayar.corridor_m]
    out = []
    for i in range(n + 1):
        s = length * i / n
        best = None
        for off in offsets:
            h = terrain(*frame.latlon(ax + ux * s - uy * off, ay + uy * s + ux * off))
            if h is None:
                return None, length
            best = h if best is None else max(best, h)
        out.append((s, best))
    return out, length


def _profile_ok(samples, start_alt, floor_alt, start_s, ayar):
    """start_s'e kadar düz, sonra eğimle floor_alt'a alçalış: her örnekte yerden >= margin_m mi?"""
    for s, ground in samples:
        alt = max(floor_alt, start_alt - max(0.0, s - start_s) * ayar.descent_gradient)
        if alt - ground < ayar.margin_m:
            return False
    return True


def _circle_points(frame, center, r, ayar):
    for k in (0.0, 1.0, ayar.check_factor):
        for b in range(0, 360, 30):
            yield _along(frame, center, b, k * r)


def _circle_ground(terrain, frame, center, r, ayar):
    heights = [terrain(*p) for p in _circle_points(frame, center, r, ayar)]
    return None if any(h is None for h in heights) else max(heights)


def _land_candidates(frame, position, entry, to_entry, length, ayar):
    """Kara çemberi merkezleri, girişe yakından uzağa: önce uçak→giriş hattı üstünde, sonra girişin çevresinde
    halkalar (uçağın gerisi ve yanları dahil). Girişe yakın çember suya kısa yol demektir."""
    out = []
    d = length - ayar.land_circle_step_m
    while d >= 0.0:
        out.append((length - d, _along(frame, position, to_entry, d)))
        d -= ayar.land_circle_step_m
    ring = ayar.land_circle_step_m
    while ring <= ayar.land_circle_search_m:
        n = max(8, int(2 * math.pi * ring / ayar.land_circle_step_m))
        for k in range(n):
            out.append((ring, _along(frame, entry, 360.0 * k / n, ring)))
        ring += ayar.land_circle_step_m
    out.sort(key=lambda t: t[0])
    return [c for _, c in out]


def plan_water_transit(position, alt_amsl, entry, course_deg, entry_amsl, terrain, is_water=None, ayar=SuGecisAyar()):
    """position: uçak (lat, lon). alt_amsl: suya gidiş irtifası (en az şimdiki irtifa ya da rota üstündeki seyir).
    entry: su koridorunun yaklaşma başı (lat, lon), course_deg: koridor yönü. entry_amsl: su + yaklaşma payı.
    is_water: su maskesi; verilmezse su üstü çember yalnız araziyle denetlenir.
    Dönen SuGecisi.items: su bacağı devralmadan önce uçulacak noktalar (irtifalar deniz seviyesine göre)."""
    frame = YerelCerceve(*position)
    samples, length = _samples(terrain, frame, position, entry, ayar)
    if samples is None:
        return SuGecisi(False, reason='suya giden yolda arazi verisi yok')
    if max(g for _, g in samples) + ayar.margin_m > alt_amsl:
        return SuGecisi(False, reason=f'suya giden yolda arazi {ayar.margin_m:.0f} m payla bu irtifanın üstünde; önce tırmanılmalı')
    to_entry = _bearing(frame, position, entry)

    # 1) Düz alçalış: payı koruyan ilk başlangıç noktası, alçalış girişe kadar bitmeli.
    latest = length - (alt_amsl - entry_amsl) / ayar.descent_gradient
    s = ayar.min_transit_m
    while s <= latest + 1e-6:
        if _profile_ok(samples, alt_amsl, entry_amsl, s, ayar):
            handoff = _along(frame, position, to_entry, s + ayar.accept_m)
            return SuGecisi(True, 'düz alçalış', descent_start_m=s, floor_amsl=entry_amsl,
                            items=[dict(name='Suya yönelme', lat=handoff[0], lon=handoff[1], alt_amsl=alt_amsl)])
        s += ayar.step_m
    reasons = ['düz alçalış suya kadar sığmıyor ya da arazi payı yok']

    r = ayar.circle_radius_m
    # 2) Su yeterince büyükse su üstünde çember: çember ve çevresi tümüyle su olmalı.
    center = _along(frame, entry, course_deg, ayar.water_circle_along_m)
    big = is_water is None or all(is_water(*p) for p in _circle_points(frame, center, r, ayar))
    if big:
        ground = _circle_ground(terrain, frame, center, r, ayar)
        path, _ = _samples(terrain, frame, position, center, ayar)
        if ground is not None and ground + ayar.margin_m <= entry_amsl and path is not None \
                and max(g for _, g in path) + ayar.margin_m <= alt_amsl:
            return SuGecisi(True, 'su üstünde çember', floor_amsl=entry_amsl,
                            items=[dict(name='Su üstünde çember atarak alçalış', kind='loiter_to_alt', lat=center[0], lon=center[1],
                                        alt_amsl=entry_amsl, radius=r)])
        reasons.append('su üstü çemberinde arazi payı yok')
    else:
        reasons.append('su yüzeyi çember için yeterince büyük değil')

    # 3) Karada çember: suya yakından uzağa adaylar; çemberin tabanı çevresindeki arazinin margin_m üstü.
    #    Su bacağı son noktada devralır ve hemen alçalır: kıyı karadaysa çemberden sonra kıyıya kadar düz bir nokta eklenir.
    #    descent_start_m bu durumda çemberden sonraki düz yoldur.
    for center in _land_candidates(frame, position, entry, to_entry, length, ayar):
        path, _ = _samples(terrain, frame, position, center, ayar)
        if path is None or max(g for _, g in path) + ayar.margin_m > alt_amsl:
            continue
        ground = _circle_ground(terrain, frame, center, r, ayar)
        if ground is not None:
            floor = min(alt_amsl, max(entry_amsl, ground + ayar.margin_m))
            beyond = _along(frame, entry, course_deg, ayar.water_slack_m)
            rest, rest_len = _samples(terrain, frame, center, beyond, ayar)
            # Çemberden çıkınca kıyıya kadar düz (karada pay), sonra alçalış: payı koruyan ilk başlangıç
            s0 = 0.0
            while rest is not None and s0 + (floor - entry_amsl) / ayar.descent_gradient <= rest_len + 1e-6:
                if _profile_ok(rest, floor, entry_amsl, s0, ayar):
                    wet = is_water is not None and is_water(*center)
                    items = [dict(name=('Kıyıda' if wet else 'Karada') + ' çember atarak alçalış', kind='loiter_to_alt', lat=center[0], lon=center[1],
                                  alt_amsl=floor, radius=r)]
                    if s0 > 0.0:
                        p = _along(frame, center, _bearing(frame, center, beyond), s0 + ayar.accept_m)
                        items.append(dict(name='Kıyıya düz', lat=p[0], lon=p[1], alt_amsl=floor))
                    return SuGecisi(True, 'karada çember', descent_start_m=s0, floor_amsl=floor, items=items)
                s0 += ayar.step_m
    reasons.append('karada uygun çember yok')
    return SuGecisi(False, reason='; '.join(reasons))
