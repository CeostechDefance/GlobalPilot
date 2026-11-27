"""Dubins yolu: başlangıç ve bitiş konum + rotası verilen, en küçük dönüş yarıçapına uyan en kısa yatay yol.

Yerel düzlem: x doğu, y kuzey (m). Rota (course) kuzeyden saat yönünde derece; içeride matematik açısı kullanılır.
En kısa yol en çok üç parçadır: L (sola yay), R (sağa yay), S (düz). Altı aday: LSL, RSR, LSR, RSL, RLR, LRL.
Saf fonksiyonlar; MAVLink ya da simülasyon bilmez.
"""
import math
from dataclasses import dataclass

TWO_PI = 2.0 * math.pi
SIGN = {'L': 1, 'R': -1, 'S': 0}   # parça türü → eğrilik işareti: sola yay +1, sağa yay -1, düz 0


def _mod(a):
    return a % TWO_PI


def course_to_theta(course_deg):
    """Kuzeyden saat yönü derece → x eksenine göre saat tersi radyan."""
    return _mod(math.radians(90.0 - course_deg))


def theta_to_course(theta):
    return (90.0 - math.degrees(theta)) % 360.0


@dataclass(frozen=True)
class Poz:
    x: float
    y: float
    course_deg: float


@dataclass(frozen=True)
class Parca:
    kind: str       # 'L', 'R', 'S'
    length: float   # m (yaylarda yay uzunluğu)


@dataclass(frozen=True)
class DubinsYol:
    start: Poz
    radius: float
    parts: tuple    # Parca dizisi
    word: str

    @property
    def length(self):
        return sum(p.length for p in self.parts)

    def sample(self, step_m):
        """Yol boyunca (s, x, y, course_deg, eğrilik işareti: +1 sol, -1 sağ, 0 düz) örnekleri; son nokta dahil."""
        out = []
        x, y, th = self.start.x, self.start.y, course_to_theta(self.start.course_deg)
        s_total = 0.0
        out.append((0.0, x, y, theta_to_course(th), 0))
        for part in self.parts:
            n = max(1, int(math.ceil(part.length / step_m)))
            ds = part.length / n
            sign = SIGN[part.kind]
            for _ in range(n):
                x, y, th = advance(x, y, th, sign, ds, self.radius)
                s_total += ds
                out.append((s_total, x, y, theta_to_course(th), sign))
        return out


def advance(x, y, th, sign, ds, r):
    """(x, y, th) pozundan sign yönünde (SIGN) r yarıçapla ds yol ilerle."""
    if sign == 0:
        return x + ds * math.cos(th), y + ds * math.sin(th), th
    dth = sign * ds / r
    cx, cy = x - sign * r * math.sin(th), y + sign * r * math.cos(th)
    th2 = th + dth
    return cx + sign * r * math.sin(th2), cy - sign * r * math.cos(th2), _mod(th2)


def _candidates(alpha, beta, d):
    """Normalleştirilmiş Dubins aday uzunlukları (t, p, q), yarıçap 1. Geçersizse None."""
    sa, sb, ca, cb = math.sin(alpha), math.sin(beta), math.cos(alpha), math.cos(beta)
    cab = math.cos(alpha - beta)
    res = {}
    p2 = 2 + d * d - 2 * cab + 2 * d * (sa - sb)
    if p2 >= 0:
        tmp = math.atan2(cb - ca, d + sa - sb)
        res['LSL'] = (_mod(-alpha + tmp), math.sqrt(p2), _mod(beta - tmp))
    p2 = 2 + d * d - 2 * cab + 2 * d * (sb - sa)
    if p2 >= 0:
        tmp = math.atan2(ca - cb, d - sa + sb)
        res['RSR'] = (_mod(alpha - tmp), math.sqrt(p2), _mod(-beta + tmp))
    p2 = -2 + d * d + 2 * cab + 2 * d * (sa + sb)
    if p2 >= 0:
        p = math.sqrt(p2)
        tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
        res['LSR'] = (_mod(-alpha + tmp), p, _mod(-_mod(beta) + tmp))
    p2 = -2 + d * d + 2 * cab - 2 * d * (sa + sb)
    if p2 >= 0:
        p = math.sqrt(p2)
        tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
        res['RSL'] = (_mod(alpha - tmp), p, _mod(beta - tmp))
    tmp = (6.0 - d * d + 2 * cab + 2 * d * (sa - sb)) / 8.0
    if abs(tmp) <= 1:
        p = _mod(TWO_PI - math.acos(tmp))
        t = _mod(alpha - math.atan2(ca - cb, d - sa + sb) + p / 2.0)
        res['RLR'] = (t, p, _mod(alpha - beta - t + p))
    tmp = (6.0 - d * d + 2 * cab + 2 * d * (sb - sa)) / 8.0
    if abs(tmp) <= 1:
        p = _mod(TWO_PI - math.acos(tmp))
        t = _mod(-alpha - math.atan2(ca - cb, d + sa - sb) + p / 2.0)
        res['LRL'] = (t, p, _mod(_mod(beta) - alpha - t + p))
    return res


def shortest(start, end, radius):
    """En kısa Dubins yolu (DubinsYol)."""
    return min(all_paths(start, end, radius), key=lambda p: p.length)


def all_paths(start, end, radius):
    """Geçerli tüm aday yollar, kısadan uzuna. Planlayıcı arazi tutmayınca bir sonrakini dener."""
    dx, dy = end.x - start.x, end.y - start.y
    d = math.hypot(dx, dy) / radius
    phi = math.atan2(dy, dx)
    alpha = _mod(course_to_theta(start.course_deg) - phi)
    beta = _mod(course_to_theta(end.course_deg) - phi)
    paths = []
    for word, (t, p, q) in _candidates(alpha, beta, d).items():
        parts = tuple(Parca(k, v * radius) for k, v in zip(word, (t, p, q)) if v * radius > 1e-6)
        paths.append(DubinsYol(start, radius, parts, word))
    paths.sort(key=lambda p: p.length)
    return paths


def turn_radius(airspeed, bank_deg=25.0, g=9.81):
    """Koordineli dönüşte en küçük yarıçap (rüzgârsız)."""
    return airspeed * airspeed / (g * math.tan(math.radians(bank_deg)))
