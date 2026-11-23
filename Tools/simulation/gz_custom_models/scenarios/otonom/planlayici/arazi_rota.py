"""Araziye duyarlı, maliyete göre rota: kalkıştan hedefe en ucuz yol ve irtifa profili.

Dağı dolanmak zorunlu değil: üstünden geçmek ucuzsa uçak tırmanır, gerekirse yerinde çember atarak tırmanır.

Arama uzayı: (ızgara hücresi, irtifa seviyesi). Hücre cell_m, seviye level_m. Hücre değeri = hücredeki en yüksek arazi
(3x3 örnek). Bir duruma ancak irtifa >= arazi + pay ise girilir. Pay kalkıştan uzaklaştıkça climbout_margin_m'den
clearance_m'ye büyür (uçak pistte zaten alçak; pay yolun yarı tırmanış eğimiyle artar).

Hareketler ve maliyet (metre cinsinden; 1 m tırmanış ≈ climb_cost_per_m metre düz uçuş: süzülme oranı L/D, enerji):
  - komşu hücreye uç: irtifa aynı, ya da yol boyunca tırmanış eğimiyle yükselir, ya da alçalış eğimiyle iner
      maliyet = yol + climb_cost_per_m · tırmanılan
  - yerinde çember tırmanışı (yalnız çember ve çevresi o irtifada arazi payını sağlıyorsa): bir seviye
      maliyet = çemberde uçulan yol (seviye / tırmanış eğimi) + climb_cost_per_m · seviye
A*: sezgisel = hedefe düz uzaklık (maliyet her zaman yoldan büyük ya da eşit: kabul edilebilir).
Çıktı: düzleştirilmiş noktalar (koridor ve irtifa denetimiyle), her noktada irtifa, gerekiyorsa çember tırmanışları.
"""
import heapq
import math
from dataclasses import dataclass, field

from . import rota as R


@dataclass(frozen=True)
class RotaAyar:
    cell_m: float = 200.0
    level_m: float = 10.0             # 200 m hücrede tırmanış ~18 m: seviye bundan küçük olmalı
    margin_m: float = 4000.0          # ızgara başlangıç-hedef kutusunun dışına taşma (dolanma payı)
    clearance_m: float = 150.0        # arazi üstü tam pay
    climbout_margin_m: float = 10.0   # kalkış noktasında pay; uzaklıkla clearance_m'ye büyür
    climbout_corridor_m: float = 60.0  # kalkış noktasında koridor yarı genişliği; uzaklıkla corridor_m'ye büyür
    climbout_corridor_slope: float = 0.3  # (kalkış engel yüzeyi gibi: pistte dar, açıldıkça geniş)
    min_cruise_rel_m: float = 300.0   # hedefte en az bu irtifa (eve göre)
    climb_gradient: float = 4.0 / 45.0     # FW_T_CLMB_MAX / FW_AIRSPD_TRIM
    descent_gradient: float = 2.0 / 45.0   # FW_T_SINK_R_SP / FW_AIRSPD_TRIM
    climb_cost_per_m: float = 10.0    # 1 m tırmanışın düz uçuş karşılığı (≈ L/D)
    descent_cost_per_m: float = 0.5   # gereksiz alçalıp yeniden tırmanmasın (eşit maliyetli yollarda testere dişi)
    loiter_radius_m: float = 700.0
    max_alt_rel_m: float = 3000.0     # aranan en yüksek irtifa (eve göre)


@dataclass
class AraziRota:
    ok: bool
    reason: str = None
    points: list = field(default_factory=list)   # (lat, lon, alt_amsl, çember_tırmanışı_mı)
    cost_m: float = None
    length_m: float = None
    climbed_m: float = None
    loiter_climb_m: float = 0.0
    crest_amsl: float = None
    expanded: int = 0


class Izgara:
    def __init__(self, terrain, frame, x0, y0, nx, ny, cell):
        self.terrain, self.frame, self.x0, self.y0, self.nx, self.ny, self.cell = terrain, frame, x0, y0, nx, ny, cell
        self.h = {}

    def center(self, i, j):
        return self.x0 + (i + 0.5) * self.cell, self.y0 + (j + 0.5) * self.cell

    def index(self, x, y):
        return int((x - self.x0) // self.cell), int((y - self.y0) // self.cell)

    def inside(self, i, j):
        return 0 <= i < self.nx and 0 <= j < self.ny

    def corridor_height(self, i, j, r):
        """Hücre ve r hücre çevresindeki en yüksek arazi: bacaklar koridorla denetlendiği için arama da öyle."""
        key = ('c', i, j, r)
        if key not in self.h:
            self.h[key] = max(self.height(a, b) for a in range(i - r, i + r + 1) for b in range(j - r, j + r + 1)
                              if self.inside(a, b))
        return self.h[key]

    def height(self, i, j):
        """Hücrenin en yüksek arazisi (3x3 örnek); veri yoksa +sonsuz (girilmez)."""
        key = (i, j)
        if key not in self.h:
            cx, cy = self.center(i, j)
            best = -math.inf
            for dx in (-0.5, 0.0, 0.5):
                for dy in (-0.5, 0.0, 0.5):
                    lat, lon = self.frame.latlon(cx + dx * self.cell, cy + dy * self.cell)
                    v = self.terrain(lat, lon)
                    if v is None:
                        self.h[key] = math.inf
                        return math.inf
                    best = max(best, v)
            self.h[key] = best
        return self.h[key]


def line_max_terrain(terrain, frame, a, b, step, corridor):
    """a, b (x, y) arasındaki düz çizgi ve iki yanındaki koridorda en yüksek arazi (yoksa +sonsuz)."""
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(1, int(math.ceil(length / step)))
    ux, uy = ((b[0] - a[0]) / length, (b[1] - a[1]) / length) if length > 0 else (1.0, 0.0)
    best = -math.inf
    for k in range(n + 1):
        x, y = a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n
        for off in (-corridor, -corridor / 2, 0.0, corridor / 2, corridor):
            lat, lon = frame.latlon(x - uy * off, y + ux * off)
            v = terrain(lat, lon)
            if v is None:
                return math.inf
            best = max(best, v)
    return best


def plan_route(start, goal, home_amsl, terrain, a=RotaAyar(), corridor_m=500.0, max_expand=400000):
    """start: (lat, lon, alt_amsl) kalkış adımının bittiği nokta; goal: (lat, lon). En ucuz rota ve irtifaları."""
    frame = R.YerelCerceve(start[0], start[1])
    sx, sy = frame.xy(start[0], start[1])
    gx, gy = frame.xy(goal[0], goal[1])
    x0, y0 = min(sx, gx) - a.margin_m, min(sy, gy) - a.margin_m
    nx = int(math.ceil((abs(gx - sx) + 2 * a.margin_m) / a.cell_m))
    ny = int(math.ceil((abs(gy - sy) + 2 * a.margin_m) / a.cell_m))
    grid = Izgara(terrain, frame, x0, y0, nx, ny, a.cell_m)
    s, g = grid.index(sx, sy), grid.index(gx, gy)
    if grid.height(*s) == math.inf or grid.height(*g) == math.inf:
        return AraziRota(False, 'başlangıç ya da hedefte arazi verisi yok')
    base = start[2]
    nlev = int(math.ceil((home_amsl + a.max_alt_rel_m - base) / a.level_m)) + 1
    alt = lambda k: base + k * a.level_m
    goal_min = home_amsl + a.min_cruise_rel_m
    loiter_cfg = R.PlanAyar(loiter_radius_m=a.loiter_radius_m, clearance_m=a.clearance_m)
    loiter_ok = {}

    def clearance_at(i, j):
        d = math.hypot((i - s[0]) * a.cell_m, (j - s[1]) * a.cell_m)
        return min(a.clearance_m, a.climbout_margin_m + 0.5 * a.climb_gradient * d)

    def r_cells(i, j):
        d = math.hypot((i - s[0]) * a.cell_m, (j - s[1]) * a.cell_m)
        width = min(corridor_m, a.climbout_corridor_m + a.climbout_corridor_slope * d)
        return int(math.ceil(width / a.cell_m))

    def ok(i, j, k):
        return alt(k) >= grid.corridor_height(i, j, r_cells(i, j)) + clearance_at(i, j)

    def can_loiter(i, j, k):
        key = (i, j, k)
        if key not in loiter_ok:
            cx, cy = grid.center(i, j)
            loiter_ok[key] = R.loiter_clear(terrain, frame, cx, cy, alt(k), loiter_cfg)
        return loiter_ok[key]

    steps = [(dx, dy, math.hypot(dx, dy) * a.cell_m) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy]
    h = lambda i, j: math.hypot((i - g[0]) * a.cell_m, (j - g[1]) * a.cell_m)
    start_state = (s[0], s[1], 0)
    best = {start_state: 0.0}
    parent = {start_state: None}
    open_ = [(h(*s), 0.0, start_state)]
    expanded = 0
    found = None
    loiter_cost = a.level_m / a.climb_gradient + a.climb_cost_per_m * a.level_m
    while open_ and expanded < max_expand:
        f, c, st = heapq.heappop(open_)
        if c > best.get(st, math.inf):
            continue
        expanded += 1
        i, j, k = st
        if (i, j) == g and alt(k) >= goal_min:
            found = st
            break
        moves = []
        for dx, dy, dist in steps:
            ni, nj = i + dx, j + dy
            if not grid.inside(ni, nj):
                continue
            up = int(a.climb_gradient * dist // a.level_m)
            down = int(a.descent_gradient * dist // a.level_m)
            for nk in {k, min(k + up, nlev - 1), max(k - down, 0)}:
                # varış hücresi varış irtifasıyla; alçalışta bulunulan hücre de yeni irtifayla geçilmeli;
                # köşegende yan hücreler iki uçtaki düşük irtifayla
                if not ok(ni, nj, nk) or (nk < k and not ok(i, j, nk)):
                    continue
                low = min(k, nk)
                if dx and dy and not (ok(i + dx, j, low) and ok(i, j + dy, low)):
                    continue
                moves.append(((ni, nj, nk), dist + (a.climb_cost_per_m * max(0, nk - k)
                                                    + a.descent_cost_per_m * max(0, k - nk)) * a.level_m))
        if k + 1 < nlev and can_loiter(i, j, k + 1):
            moves.append(((i, j, k + 1), loiter_cost))
        for nst, cost in moves:
            nc = c + cost
            if nc < best.get(nst, math.inf):
                best[nst] = nc
                parent[nst] = st
                heapq.heappush(open_, (nc + h(nst[0], nst[1]), nc, nst))
    if found is None:
        return AraziRota(False, 'hedefe arazi payını sağlayan rota bulunamadı', expanded=expanded)
    path = []
    st = found
    while st is not None:
        path.append(st)
        st = parent[st]
    path.reverse()
    corridor_at = lambda x, y: min(corridor_m, a.climbout_corridor_m + a.climbout_corridor_slope * math.hypot(x - sx, y - sy))
    return _assemble(path, grid, frame, terrain, a, (sx, sy), (gx, gy), alt, clearance_at, corridor_at, best[found], expanded)


def _assemble(path, grid, frame, terrain, a, sxy, gxy, alt, clearance_at, corridor, cost, expanded):
    """Izgara yolunu noktalara çevirir. Düzleştirme irtifayı uçulan yoldan yeniden hesaplar: birleştirilen bacakta
    uçak en çok tırmanış eğimiyle yükselir (en çok noktanın hedef irtifasına), araziyi bu gerçek irtifayla geçmeli.
    Böylece kalkışta en hızlı tırmanırken ızgaranın zikzakı kalmaz. Izgara adımından doğan küçük çember tırmanışları
    (en çok 2 seviye) yol boyu tırmanışa bırakılır; yetmezse orada gerçekten çember atılır."""
    nodes = []   # [x, y, hedef irtifa, çember]
    for idx, (i, j, k) in enumerate(path):
        xy = sxy if idx == 0 else (gxy if idx == len(path) - 1 else grid.center(i, j))
        if nodes and (i, j) == path[idx - 1][:2]:
            climb = alt(k) - nodes[-1][4]
            nodes[-1] = [nodes[-1][0], nodes[-1][1], alt(k), True, nodes[-1][4]]
        else:
            nodes.append([xy[0], xy[1], alt(k), False, alt(k)])   # son alan: çember öncesi irtifa
    for n in nodes:
        if n[3] and n[2] - n[4] <= 2 * a.level_m:
            n[3] = False                                            # küçük çember: yol boyu tırmanışa
    margin_at = lambda x, y: clearance_at(*grid.index(x, y))

    def end_alt(z0, target, length):
        return min(target, z0 + a.climb_gradient * length) if target >= z0 else max(target, z0 - a.descent_gradient * length)

    def leg_ok(p, q, z0):
        """Bacak boyunca her örnekte: o noktadaki irtifa >= koridordaki arazi + o noktadaki pay."""
        x0, y0, x1, y1 = nodes[p][0], nodes[p][1], nodes[q][0], nodes[q][1]
        length = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(math.ceil(length / (a.cell_m / 2))))
        ux, uy = ((x1 - x0) / length, (y1 - y0) / length) if length > 0 else (1.0, 0.0)
        width = corridor(x0, y0)
        worst = math.inf
        for k in range(n + 1):
            s = length * k / n
            x, y = x0 + ux * s, y0 + uy * s
            z = end_alt(z0, nodes[q][2], s)
            for off in (-width, -width / 2, 0.0, width / 2, width):
                lat, lon = frame.latlon(x - uy * off, y + ux * off)
                h = terrain(lat, lon)
                if h is None:
                    return False, z0, math.inf
                worst = min(worst, z - (h + margin_at(x, y)))
        z1 = end_alt(z0, nodes[q][2], length)
        return worst >= 0.0, z1, z0 - worst      # need: başta bu irtifa olsaydı geçerdi

    out = []           # (x, y, gerçek irtifa, çember)
    loiter_climb = 0.0
    p, zp = 0, nodes[0][2]
    out.append((nodes[0][0], nodes[0][1], zp, False))
    while p < len(nodes) - 1:
        chosen = None
        for q in range(len(nodes) - 1, p, -1):
            if any(n[3] for n in nodes[p + 1:q]):
                continue                                            # gerçek çember noktası atlanmaz
            ok, z1, need = leg_ok(p, q, zp)
            if ok:
                chosen = (q, z1)
                break
        if chosen is None:
            # komşu noktaya bile gerçek irtifayla geçilemiyor: burada gereken irtifaya çember atarak tırman
            ok, z1, need = leg_ok(p, p + 1, zp)
            loiter_climb += need - zp
            zp = need
            out[-1] = (out[-1][0], out[-1][1], zp, True)
            z1 = end_alt(zp, max(nodes[p + 1][2], zp), math.hypot(nodes[p + 1][0] - nodes[p][0], nodes[p + 1][1] - nodes[p][1]))
            chosen = (p + 1, z1)
        q, z1 = chosen
        if nodes[q][3]:
            loiter_climb += max(0.0, nodes[q][2] - z1)
            z1 = max(z1, nodes[q][2])
        out.append((nodes[q][0], nodes[q][1], z1, nodes[q][3]))
        p, zp = q, z1
    pts, length, climbed = [], 0.0, loiter_climb
    for n, (x, y, z, loiter) in enumerate(out):
        if n:
            px, py, pz, _ = out[n - 1]
            length += math.hypot(x - px, y - py)
            climbed += max(0.0, min(z, pz + a.climb_gradient * math.hypot(x - px, y - py)) - pz)
        lat, lon = frame.latlon(x, y)
        pts.append((lat, lon, z, loiter))
    crest = max(line_max_terrain(terrain, frame, o1[:2], o2[:2], a.cell_m / 2, corridor(*o1[:2])) for o1, o2 in zip(out, out[1:]))
    return AraziRota(True, None, pts, cost, length, climbed, loiter_climb, crest, expanded)
