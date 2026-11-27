"""Suya hizalanma rehberi: saf fonksiyonlar. MAVLink, simülasyon ya da saat bilmez; zaman adımı dışarıdan verilir.

Girdi: anlık durum (Durum), hedef hat (Hat), aşama ve ayarlar. Çıktı: yatış, yunuslama (derece) ve gaz (0-1).
Yanal: hattan yan sapma ve yön hatası → istenen rota → koordineli dönüş yatışı.
Boyuna: istenen dikey hız → yunuslama (aerodinamik ön besleme + PI). Hız → gaz (PI).
Aerodinamik değerler GASI FFA modelinin AdvancedLiftDrag ayarlarından (gasi_ffa/model.sdf): CL = CL0 + CLa·α.
"""
import math
import os
from dataclasses import dataclass, field, replace

from .planlayici.geo import track_errors

G = 9.81
RHO = 1.2041          # model.sdf air_density
WING_AREA_M2 = 15.0
CL0 = 0.7095          # gövde ekseninde α = 0 için
CLA = 4.693           # 1/rad
ALPHA_STALL_RAD = 0.1927
CL_MAX_CLEAN = CL0 + CLA * ALPHA_STALL_RAD   # 1,614; 1200 kg'da 28,4 m/s (airframe 4022 notuyla aynı). Tasarım değeri:
# model.sdf'teki alpha_stall gz sigmoidi için ayrıca ayarlanabilir (49, 2026-11-08), bu sabit modelden okunmaz
CD0, ASPECT, OSWALD = 0.04, 5.4, 0.8
THRUST_MAX_N = 5002.0  # itki gazın karesiyle orantılı (motorConstant · maxRotVelocity²)


def stall_speed(mass_kg, cl_max=CL_MAX_CLEAN):
    """Düz uçuşta tutunma kaybı hızı, m/s."""
    return math.sqrt(2.0 * mass_kg * G / (RHO * WING_AREA_M2 * cl_max))


def alpha_for(mass_kg, airspeed, roll_rad=0.0):
    """Bu hız ve yatışta yükü taşımak için gereken hücum açısı (rad), doğrusal bölgede."""
    q = 0.5 * RHO * max(airspeed, 1.0) ** 2
    cl = mass_kg * G / (q * WING_AREA_M2 * max(math.cos(roll_rad), 0.3))
    return (cl - CL0) / CLA


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class Ayar:
    # hızlar (Vs katı); havada alt sınır PX4'ün FW_AIRSPD_MIN değeri
    speed_factor_air: float = 1.3
    airspeed_min_air: float = 37.0
    speed_factor_touch: float = 1.3     # DEĞ: burun yukarı değiş ancak ~1,3·Vs'de dengelenir
    # Suya değiş, yasa 3 alanları (PX4 FW_LND_FL_LAW 3'ün kopyası; fl_sink ve fl_hold_m yasa 6'da da). Değerler
    # ayar_from_px4 ile PX4'ten; yalnız burun sınırları suya göre (önce kıç: gövde basamağı +2..+7°, kıç ~8,9°'de değer)
    land_airspeed: float = 35.0         # FW_LND_AIRSPD; yaklaşma hızı max(bu, 1,3·Vs)
    fl_sink: float = 0.5                # FW_LND_FL_SINK: temas alçalması
    fl_hold_m: float = 1.0              # FW_LND_FL_HOLD: bu yükseklikte ve altında hedef = fl_sink
    fl_kp: float = 5.0                  # FW_LND_FL_KP: derece / (m/s)
    fl_ki: float = 2.0                  # FW_LND_FL_KI: derece / (m/s·s)
    fl_time_s: float = 1.0              # FW_LND_FL_TIME: gaz bu sürede rölantiye
    # suya özgü FL_PMIN (pistte 0; kullanıcı kararı 1,5): ön gövde ancak < 0°'de önce değer. GASI_FL_PMIN_WATER yalnız deneme
    # içindir (2026-11-10, değme açısı denemesi); verilmezse 1,5
    fl_pmin_water: float = float(os.environ.get('GASI_FL_PMIN_WATER', '1.5'))
    fl_pmax_water: float = 7.0          # suya özgü FL_PMAX (pistte 15): kıç basamaktan çok önce değmesin
    p_lim_min: float = -12.0            # FW_P_LIM_MIN
    p_lim_max: float = 15.0             # FW_P_LIM_MAX
    thr_idle: float = 0.0               # FW_THR_IDLE
    thr_max: float = 1.0                # FW_THR_MAX
    touch_stall_factor: float = 1.05    # flare'de hız bunun·Vs altına inerse ve h > 1 m: PAS GEÇ
    # Yasa 3'ten tek fark (kullanıcı kararı 2026-11-08): suda uçak durmayacak, kayacak. 'hold': flare'de gaz rölantiye
    # inmez, hız speed_factor_touch·Vs'de tutulur. 'idle': PX4 pistteki gibi (n12 boş: temas -1,14 m/s, 26 m/s, burun 6,7°;
    # n13 dolu: -0,81 m/s, 31 m/s, burun 7,5°)
    water_flare_throttle: str = 'hold'
    # Suya değiş yasası: 'yasa3' (PX4 flare yasası 3 kopyası, burun alçalmayı tutar) ya da 'yasa4' (globalpilot-79'un pist
    # yasası 4'ün su karşılığı, 2026-11-10: burun fl_att_water'a 1 s'de gelip sabit kalır, gaz alçalmayı PI ile
    # tutar; FAA-H-8083-23 bölüm 6 glassy water: sabit iniş açısı, ≤0,76 m/s alçalma, suya kadar flare yok, gaz değince
    # kesilmez) ya da 'yasa6' (pistteki yasa 6, kütleden bağımsız). Varsayılan yasa6 (kullanıcı kararı 2026-11-27, pistte
    # de FW_LND_FL_LAW 6); GASI_WATER_LAW ile başka yasa denenebilir.
    water_touch_law: str = os.environ.get('GASI_WATER_LAW', 'yasa6')
    fl_att_water: float = float(os.environ.get('GASI_FL_ATT_WATER', '6.0'))   # yasa4 burun (gövde basamağı 2-7° içinde)
    fl_snk1: float = 0.75               # yasa4: hold yüksekliğinin üstünde alçalma (FAA ≤150 ft/dk)
    fl_hold4_m: float = 3.0             # yasa4: bunun altında alçalma fl_sink
    fl_tff: float = 0.45                # yasa4: gaz ileri besleme
    fl_tkp: float = 0.15                # yasa4: gaz / (m/s) alçalma hatası
    fl_tki: float = 0.10                # yasa4: gaz / (m/s·s)
    fl_vmin: float = 27.7               # yasa4: 1200 kg'da hız koruması; ağırlıkla sqrt(m/1200) ölçeklenir
    fl_vmin_pitch_max: float = 2.0      # yasa4: hız korumasında burun en çok bu kadar iner
    touch_stall_factor_law4: float = 1.0  # yasa4 ve yasa6 PAS GEÇ hız sınırı (·Vs): denge hızı ~1,03·Vs, 1,05 her denemede kesti
    # Yasa 6'nın su karşılığı (globalpilot-6b, PX4 FW_LND_FL_LAW 6 ile aynı ilke, 2026-11-11): tutunma hızı kütleden değil
    # yaklaşmada sentetik hücum açısından (burun - uçuş yolu açısı) tahmin edilir; yaklaşma l6_vapp_k·Vs, flare'de burun
    # alçalmayı PI ile tutar (fl_snk1, fl_hold_m altında fl_sink), gaz yükseklikle l6_vtd_k·Vs'ye inen hızı tutar, giriş
    # gazından başlar, en çok l6_tslw/s değişir, suya değince kesilmez. Flapsız: suya yaklaşmada flap 0 (par_y4b kayıtları).
    l6_cl0: float = 0.73                # FW_WE_CL0 (kayıtlardan, flapsız)
    l6_cla: float = 4.7                 # FW_WE_CLA, 1/rad
    l6_vapp_k: float = 1.30             # FW_LND_VAPP_K
    l6_vtd_k: float = 1.15              # suda 1,15 (pistte FW_LND_VTD_K 1,12): değme burnu ~5°; kıç konisi basamak 5-10 cm batıkken 6-7°'de değer (79)
    l6_kp: float = 2.0                  # derece / (m/s); 4/4 ile Dalaman tam görevinde burun -1,7..+6,5° salındı (2026-11-11)
    l6_ki: float = 1.0                  # derece / (m/s·s)
    l6_tatt_s: float = 4.0              # FW_LND_FL_TATT: alçalma hedefi bu sürede giriş alçalmasından fl_snk1'e iner
    l6_prate: float = 3.0               # FW_LND_FL_PRT: burun komutu en çok bu kadar derece/s değişir (kullanıcı: ani burun
                                        # kaldırma olmasın)
    l6_tkp: float = 0.15                # FW_LND_FL_TKP: gaz / (m/s) hız hatası
    l6_tki: float = 0.10                # FW_LND_FL_TKI
    l6_tslw: float = 0.10               # FW_LND_FL_TSLW: gaz değişimi / s (itki ekseni CG üstünde: ani gaz burnu düşürür)
    decel_start_height_m: float = 45.0  # ALÇAL'ın son kısmı: hava hızı buradan decel_end_height_m'ye kadar değiş hızına iner
    decel_end_height_m: float = 15.0    # kapı (FW_LND_FLALT) 10 m iken ondan 5 m önce biterdi; 15 m ile (yasa 6 seti, 11 ve 27
                                        # Kasım uçuşları) kapıda biter
    decel_sink_max: float = 0.6         # hız hedefin üstündeyken alçalma sınırı: alçalma hızı geri beslemesin (uçuş o2)
    decel_speed_margin: float = 1.5     # m/s: hız fazlası 0'dan bu değere çıkarken alçalma sınırı sink_max'tan decel_sink_max'a
                                        # doğrusal iner. Ani anahtar (n1) hedef dikey hızı -2,0/-0,6 arasında zıplatıp salınım yaptı
    # yanal
    roll_limit_deg: float = 25.0
    lookahead_s: float = 6.0
    lookahead_min_m: float = 60.0
    intercept_max_deg: float = 45.0
    course_tau_s: float = 4.0
    # boyuna
    height_tau_s: float = 8.0           # ALÇAL: yükseklik hatası → dikey hız
    sink_max: float = 2.0               # m/s
    climb_max: float = 5.0
    flare_height_m: float = 5.0
    flare_tau_s: float = 1.75
    flare_sink_min: float = 0.4
    touch_sink_max: float = 0.5         # DEĞ: +2..+7° yunuslama ve 1,3·Vs ile ulaşılabilen en hızlı alçalma ~0,4-0,5 m/s
    touch_pitch_margin_deg: float = 0.5  # komut aralığı ölçüt aralığının bu kadar içinde: ölçüm sınırda kalınca PAS GEÇ olmasın
    pitch_kp: float = 1.5               # derece / (m/s) dikey hız hatası (3,0 yeni modelde ±12° salınım yaptı, n1)
    pitch_ki: float = 0.6
    pitch_int_limit_deg: float = 4.0
    pitch_limit_deg: tuple = (-12.0, 15.0)
    touch_pitch_deg: tuple = (2.0, 7.0)  # gövde basamağı: altında burun, üstünde kıç önce değer
    # gaz: ön besleme (sürükleme + m·g·sinγ) ve PI düzeltme
    thr_kp: float = 0.08                # (m/s) başına
    thr_ki: float = 0.02
    thr_int_limit: float = 0.25
    thr_limits: tuple = (0.05, 1.0)
    # kapılar
    capture_cross_m: float = 30.0
    capture_track_deg: float = 5.0
    capture_roll_deg: float = 5.0
    capture_hold_s: float = 5.0
    gate_height_m: float = 10.0         # FW_LND_FLALT (ayar_from_px4): kapı flare başlangıcıdır
    gate_cross_m: float = 15.0
    gate_track_deg: float = 3.0
    gate_roll_deg: float = 3.0
    gate_speed_tol: float = 2.0
    touch_roll_abort_deg: float = 5.0


@dataclass(frozen=True)
class Durum:
    lat: float
    lon: float
    course_deg: float       # yer izi
    airspeed: float
    height: float           # gövde altının su/yer üstü yüksekliği (sensor)
    climb: float            # yukarı +, m/s
    roll_deg: float
    pitch_deg: float
    mass_kg: float


@dataclass(frozen=True)
class Hat:
    lat: float
    lon: float
    course_deg: float


@dataclass(frozen=True)
class Kontrol:
    """Aşamalar arası taşınan integratörler ve flare durumu. İlk değerler giriş anındaki komuttan (sarsıntısız geçiş)."""
    pitch_int: float = 0.0
    thr_int: float = 0.0
    flare: bool = False             # flare başladı mı (PX4: _flare_states.flaring, geri dönüşsüz)
    fl_start_h: float = 0.0
    fl_start_sink: float = 0.0
    fl_start_pitch: float = 0.0
    fl_int: float = 0.0             # derece
    fl_t: float = 0.0               # flare başından beri süre
    fl_stage: int = 0               # yasa4: 0 yavaşlama (gaz rölanti, burun alçalmayı tutar), 1 sabit açı (gaz alçalmayı tutar)
    fl_pint: float = 0.0            # yasa4 yavaşlama: burun PI tümlevi (derece)
    fl_t1: float = 0.0              # yasa4: sabit açıya geçiş anı (fl_t)
    fl_p1: float = 0.0              # yasa4: geçişteki burun
    vs_est: float = float('nan')    # yasa6: sentetik hücum açısından tutunma hızı (m/s), flare'de donar
    vs_t: float = 0.0               # yasa6: tahminin süzüldüğü süre (s)
    level_t: float = 0.0            # yasa6: kanatlar düz geçen süre (s)
    last_thr: float = float('nan')  # son gaz komutu (yasa6 flare'i bundan sarsıntısız başlar)
    fl_v0: float = 0.0              # yasa6: flare başı hava hızı
    fl_thr0: float = 0.0            # yasa6: flare başı gaz
    fl_thr: float = 0.0             # yasa6: hız sınırlı gaz komutu
    fl_tint: float = 0.0            # yasa6: gaz PI tümlevi
    fl_pcmd: float = 0.0            # yasa6: hız sınırlı burun komutu (derece)


@dataclass(frozen=True)
class Komut:
    roll_deg: float
    pitch_deg: float
    thrust: float
    target_speed: float
    target_climb: float
    notes: dict = field(default_factory=dict)


def line_errors(d, hat):
    """(boyuna ilerleme, yan sapma sağa +, yön hatası) m, m, derece. Küçük alan için düz yer yaklaşımı."""
    along, cross = track_errors(d.lat, d.lon, (hat.lat, hat.lon), hat.course_deg)
    return along, cross, wrap_deg(d.course_deg - hat.course_deg)


def vs_of(k, mass_kg, a=Ayar()):
    """Yasa 6'da tahmin edilen tutunma hızı (oturduysa), yoksa kütleden."""
    if a.water_touch_law == 'yasa6' and k is not None and k.vs_t >= 5.0 and not math.isnan(k.vs_est):
        return k.vs_est
    return stall_speed(mass_kg)


def target_speed(mass_kg, phase, height=None, a=Ayar(), vs=None):
    """Hedef hava hızı. YAKALA/ALÇAL: max(1,3·Vs, FW_AIRSPD_MIN). Yaklaşma/DEĞ: max(FW_LND_AIRSPD, 1,3·Vs).
    ALÇAL'ın son kısmında (decel_start_height_m → decel_end_height_m) doğrusal geçiş. Flare'de gaz rölantiye iner
    (yasa 3), hız hedefi yalnız kapı denetimi ve gaz rampası öncesi içindir.
    Yasa 6: değiş hızı l6_vapp_k·Vs (FW_LND_AIRSPD tabanı yok); vs verilirse kütle yerine o kullanılır."""
    vs = stall_speed(mass_kg) if vs is None else vs
    air = max(a.speed_factor_air * vs, a.airspeed_min_air)
    touch = max(a.land_airspeed, a.speed_factor_touch * vs)
    if a.water_touch_law == 'yasa6':
        touch = a.l6_vapp_k * vs
    if phase == 'DEG':
        return touch
    if phase == 'ALCAL' and height is not None and height < a.decel_start_height_m:
        f = (a.decel_start_height_m - height) / (a.decel_start_height_m - a.decel_end_height_m)
        return air + (touch - air) * max(0.0, min(1.0, f))
    return air


def lateral(d, hat, a=Ayar()):
    """Yatış komutu (derece) ve ara değerler."""
    _, cross, _ = line_errors(d, hat)
    look = max(a.lookahead_s * max(d.airspeed, 10.0), a.lookahead_min_m)
    intercept = math.degrees(math.atan2(cross, look))
    intercept = max(-a.intercept_max_deg, min(a.intercept_max_deg, intercept))
    wanted = hat.course_deg - intercept
    err = wrap_deg(wanted - d.course_deg)
    turn_rate = math.radians(err) / a.course_tau_s
    roll = math.degrees(math.atan(max(d.airspeed, 10.0) * turn_rate / G))
    roll = max(-a.roll_limit_deg, min(a.roll_limit_deg, roll))
    return roll, dict(cross=cross, wanted_course=wanted, course_err=err)


def climb_target(height, target_height, phase, a=Ayar(), speed_excess=0.0):
    """İstenen dikey hız (yukarı +). speed_excess: hava hızı - hedef hız.
    Gaz rölantideyken 2 m/s alçalma yavaşlamayı neredeyse durdurur (o2: 0,18 m/s²); hız fazlaysa alçalma sınırlanır."""
    if phase == 'DEG':
        # Yunuslama +2..+7° ve 1,3·Vs ile alçalma ~0,5 m/s'yi geçemez: asıl burun kaldırma kapıda (su+10 m) olur.
        # Flare (su+5 m altı) alçalmayı yüksekliğe göre azaltır, alt sınır flare_sink_min.
        sink = a.touch_sink_max
        if height <= a.flare_height_m:
            sink = min(sink, max(height / a.flare_tau_s, a.flare_sink_min))
        return -sink
    rate = (target_height - height) / a.height_tau_s
    blend = max(0.0, min(1.0, speed_excess / a.decel_speed_margin))
    sink = a.sink_max - (a.sink_max - a.decel_sink_max) * blend
    return max(-sink, min(a.climb_max, rate))


def longitudinal(d, climb_sp, phase, k, dt, a=Ayar()):
    """Yunuslama (derece) ve yeni yunuslama integratörü."""
    alpha = math.degrees(alpha_for(d.mass_kg, d.airspeed, math.radians(d.roll_deg)))
    gamma = math.degrees(math.atan2(climb_sp, max(d.airspeed, 10.0)))
    err = climb_sp - d.climb
    integ = max(-a.pitch_int_limit_deg, min(a.pitch_int_limit_deg, k.pitch_int + a.pitch_ki * err * dt))
    pitch = alpha + gamma + a.pitch_kp * err + integ
    if phase == 'DEG':
        low, high = a.touch_pitch_deg[0] + a.touch_pitch_margin_deg, a.touch_pitch_deg[1] - a.touch_pitch_margin_deg
    else:
        low, high = a.pitch_limit_deg
    clipped = max(low, min(high, pitch))
    if clipped != pitch:   # doyumda integratör büyümesin
        integ = k.pitch_int
    return clipped, integ


def thrust_feedforward(mass_kg, airspeed, climb_sp, roll_deg=0.0):
    """Hızı korumak için gereken gaz: itki = sürükleme + m·g·sinγ, gaz = sqrt(itki / en büyük itki)."""
    v = max(airspeed, 10.0)
    q = 0.5 * RHO * v * v
    cl = mass_kg * G / (q * WING_AREA_M2 * max(math.cos(math.radians(roll_deg)), 0.3))
    drag = q * WING_AREA_M2 * (CD0 + cl * cl / (math.pi * OSWALD * ASPECT))
    need = drag + mass_kg * G * climb_sp / v
    return math.sqrt(max(need, 0.0) / THRUST_MAX_N)


def throttle(d, speed_sp, climb_sp, k, dt, a=Ayar()):
    err = speed_sp - d.airspeed
    low, high = a.thr_limits
    ff = thrust_feedforward(d.mass_kg, speed_sp, climb_sp, d.roll_deg)
    integ = max(-a.thr_int_limit, min(a.thr_int_limit, k.thr_int + a.thr_ki * err * dt))
    thr = ff + integ + a.thr_kp * err
    clipped = max(low, min(high, thr))
    if clipped != thr:
        integ = k.thr_int
    return clipped, integ


def flare_law3(d, k, dt, a=Ayar()):
    """PX4 iniş flare'i yasa 3'ün kopyası; h = gövde altının su üstü yüksekliği.
    Dönüş: (yunuslama°, hedef dikey hız (yukarı +), gaz üst sınırı, yeni Kontrol)."""
    if not k.flare:
        # flare başlangıcı: geri dönüşsüz; başlangıç alçalması, yüksekliği ve burun açısı saklanır
        k = replace(k, flare=True, fl_start_h=d.height, fl_start_sink=-d.climb, fl_start_pitch=d.pitch_deg, fl_int=0.0, fl_t=0.0)
    t = k.fl_t + dt
    ramp = math.sqrt(max(0.0, min(1.0, t / a.fl_time_s)))
    sink_td = max(a.fl_sink, 0.0)
    sink_start = max(k.fl_start_sink, sink_td)
    hold = max(a.fl_hold_m, 0.0)
    span = max(k.fl_start_h - hold, 0.1)
    f = max(0.0, min(1.0, (d.height - hold) / span))          # 1 flare başında, 0 hold yüksekliğinde ve altında
    sink = sink_td + (sink_start - sink_td) * f
    interp = 1.0 - f
    pmin = interp * a.fl_pmin_water + (1.0 - interp) * a.p_lim_min
    pmax = interp * a.fl_pmax_water + (1.0 - interp) * a.p_lim_max
    err = -d.climb - sink                                     # > 0: istenenden hızlı alçalıyor
    unsat = k.fl_start_pitch + a.fl_kp * err + k.fl_int
    integ = k.fl_int
    if not ((unsat >= pmax and err > 0.0) or (unsat <= pmin and err < 0.0)):
        integ += a.fl_ki * err * dt
    pitch = max(pmin, min(pmax, unsat))
    thr_max = ramp * a.thr_idle + (1.0 - ramp) * a.thr_max
    return pitch, -sink, thr_max, replace(k, fl_int=integ, fl_t=t)


def flare_law4_water(d, k, dt, a=Ayar()):
    """Yasa 4'ün su karşılığı, iki aşama (globalpilot-79'un PX4 pist yasasıyla aynı; FAA: 200 ft'ten önce Vs+10 kt'ye
    yavaşla, sonra sabit iniş açısı). Dönüş: (yunuslama°, hedef dikey hız (yukarı +), gaz, yeni Kontrol).
    0 YAVAŞLAMA: gaz rölanti; burun alçalma hedefini (fl_snk1) tutan PI (yasa 3 kazançları, başlangıç burnu etrafında),
      üst sınır fl_att_water: uçak tırmanamaz, hız düştükçe gereken burun kendiliğinden artar.
    1 SABİT AÇI: PI çıkışı fl_att_water'a ulaşınca ya da h < fl_hold4_m olunca; burun 1 s'de fl_att_water'a gelip sabit,
      gaz alçalmayı PI ile tutar (tümlev sıfırdan); hız koruması burnu en çok fl_vmin_pitch_max indirir."""
    if not k.flare:
        k = replace(k, flare=True, fl_start_h=d.height, fl_start_sink=-d.climb, fl_start_pitch=d.pitch_deg, fl_int=0.0,
                    fl_t=0.0, fl_stage=0, fl_pint=0.0)
    t = k.fl_t + dt
    low, high = a.touch_pitch_deg[0] + a.touch_pitch_margin_deg, a.touch_pitch_deg[1] - a.touch_pitch_margin_deg
    sink = a.fl_snk1 if d.height > a.fl_hold4_m else a.fl_sink
    err = -d.climb - sink                                     # > 0: istenenden hızlı alçalıyor
    if k.fl_stage == 0:
        unsat = k.fl_start_pitch + a.fl_kp * err + k.fl_pint
        pint = k.fl_pint
        pmin, pmax = a.p_lim_min, a.fl_att_water
        if not ((unsat >= pmax and err > 0.0) or (unsat <= pmin and err < 0.0)):
            pint += a.fl_ki * err * dt
        pitch = max(pmin, min(pmax, unsat))
        if unsat >= a.fl_att_water or d.height < a.fl_hold4_m:
            k = replace(k, fl_stage=1, fl_int=0.0, fl_t1=t, fl_p1=pitch)
        else:
            return pitch, -sink, a.thr_limits[0], replace(k, fl_pint=pint, fl_t=t)
    f = max(0.0, min(1.0, (t - k.fl_t1) / 1.0))
    pitch = k.fl_p1 + (a.fl_att_water - k.fl_p1) * f
    vmin = a.fl_vmin * math.sqrt(max(d.mass_kg, 1.0) / 1200.0)
    if d.airspeed < vmin:   # hız koruması: burun en çok fl_vmin_pitch_max iner
        pitch -= min(a.fl_vmin_pitch_max, 0.5 * (vmin - d.airspeed))
    pitch = max(low, min(high, pitch))
    integ = max(-0.4, min(0.4, k.fl_int + a.fl_tki * err * dt))
    thr = a.fl_tff + a.fl_tkp * err + integ
    lo, hi = a.thr_limits
    if not lo <= thr <= hi:
        integ = k.fl_int
    thr = max(lo, min(hi, thr))
    return pitch, -sink, thr, replace(k, fl_int=integ, fl_t=t)


def update_vs_estimate(d, k, dt, a=Ayar()):
    """Yasa 6 tutunma hızı tahmini: kanatlar 2 s düz (yatış < 5°), su+20 m üstü, dikey hız < 4 m/s iken
    Vs = V·sqrt(CL/CL_MAX_CLEAN), CL = l6_cl0 + l6_cla·(burun - uçuş yolu açısı); 5 s zaman sabitiyle süzülür."""
    level_t = k.level_t + dt if abs(d.roll_deg) < 5.0 else 0.0
    k = replace(k, level_t=level_t)
    if level_t < 2.0 or math.isnan(d.height) or d.height < 20.0 or abs(d.climb) > 4.0 or d.airspeed < 10.0:
        return k
    alpha = math.radians(d.pitch_deg) - math.atan2(d.climb, d.airspeed)
    cl = max(0.1, min(CL_MAX_CLEAN, a.l6_cl0 + a.l6_cla * alpha))
    sample = d.airspeed * math.sqrt(cl * math.cos(math.radians(d.roll_deg)) / CL_MAX_CLEAN)
    if math.isnan(k.vs_est):
        return replace(k, vs_est=sample)
    return replace(k, vs_est=k.vs_est + (sample - k.vs_est) * min(dt / 5.0, 1.0), vs_t=k.vs_t + dt)


def flare_law6_water(d, k, dt, a=Ayar()):
    """Yasa 6'nın su karşılığı (PX4 FW_LND_FL_LAW 6). Dönüş: (yunuslama°, hedef dikey hız (yukarı +), gaz, hız hedefi,
    yeni Kontrol). Burun: alçalma hatası PI, flare başı burnu etrafında; sınır flare boyunca P_LIM_MIN'den gövde
    basamağına (touch_pitch_deg) daralır. Gaz: yükseklikle Vtd = l6_vtd_k·Vs'ye inen hız hedefi PI, merkez flare başı gazı,
    en çok l6_tslw/s; suda kesilmez."""
    vs = vs_of(k, d.mass_kg, a)
    if not k.flare:
        thr0 = k.last_thr if not math.isnan(k.last_thr) else thrust_feedforward(d.mass_kg, d.airspeed, d.climb, d.roll_deg)
        k = replace(k, flare=True, fl_start_h=d.height, fl_start_sink=-d.climb, fl_start_pitch=d.pitch_deg, fl_int=0.0,
                    fl_t=0.0, fl_v0=d.airspeed, fl_thr0=thr0, fl_thr=thr0, fl_tint=0.0, fl_pcmd=d.pitch_deg)
    t = k.fl_t + dt
    hold = a.fl_hold_m
    trans = max(0.0, min(1.0, t / max(a.l6_tatt_s, 0.1)))
    sink = trans * (a.fl_snk1 if d.height > hold else a.fl_sink) + (1.0 - trans) * max(k.fl_start_sink, 0.0)
    f = max(0.0, min(1.0, (d.height - hold) / max(k.fl_start_h - hold, 0.1)))   # 1 flare başı, 0 hold altı
    low, high = a.touch_pitch_deg[0] + a.touch_pitch_margin_deg, a.touch_pitch_deg[1] - a.touch_pitch_margin_deg
    pmin = f * a.p_lim_min + (1.0 - f) * low
    pmax = high
    err = -d.climb - sink                                      # > 0: istenenden hızlı alçalıyor
    unsat = k.fl_start_pitch + a.l6_kp * err + k.fl_int
    want = max(pmin, min(pmax, unsat))
    pstep = a.l6_prate * dt
    pitch = k.fl_pcmd + max(-pstep, min(pstep, want - k.fl_pcmd))
    plimited = abs(pitch - want) > 1e-6
    integ = k.fl_int
    up_held = unsat >= pmax or (plimited and want > pitch)
    down_held = unsat <= pmin or (plimited and want < pitch)
    if not ((up_held and err > 0.0) or (down_held and err < 0.0)):
        integ += a.l6_ki * err * dt
    vtd = a.l6_vtd_k * vs
    speed_sp = vtd + (max(k.fl_v0, vtd) - vtd) * f
    serr = speed_sp - d.airspeed                               # > 0: yavaş
    lo, hi = a.thr_limits
    tun = k.fl_thr0 + a.l6_tkp * serr + k.fl_tint
    target = max(lo, min(hi, tun))
    step = a.l6_tslw * dt
    thr = k.fl_thr + max(-step, min(step, target - k.fl_thr))
    limited = abs(thr - target) > 1e-4
    held_up = tun >= hi or (limited and target > thr)
    held_down = tun <= lo or (limited and target < thr)
    tint = k.fl_tint
    if not ((held_up and serr > 0.0) or (held_down and serr < 0.0)):
        tint += a.l6_tki * serr * dt
    return pitch, -sink, thr, speed_sp, replace(k, fl_int=integ, fl_t=t, fl_thr=thr, fl_tint=tint, fl_pcmd=pitch)


def command(d, hat, phase, target_height, k, dt, a=Ayar()):
    """Tek adım: (Komut, yeni Kontrol). phase: 'YAKALA', 'ALCAL', 'DEG', 'PAS'."""
    cmd, k = _command(d, hat, phase, target_height, k, dt, a)
    return cmd, replace(k, last_thr=cmd.thrust)


def _command(d, hat, phase, target_height, k, dt, a=Ayar()):
    """Tek adım: (Komut, yeni Kontrol). phase: 'YAKALA', 'ALCAL', 'DEG', 'PAS'."""
    if phase == 'PAS':
        # kanatlar düz, tırmanış, tam gaz; hat takibi yok
        sp = target_speed(d.mass_kg, 'YAKALA', a=a)
        pitch = max(a.pitch_limit_deg[0], min(a.pitch_limit_deg[1], 8.0))
        return Komut(0.0, pitch, a.thr_limits[1], sp, a.climb_max), replace(k, pitch_int=0.0, thr_int=0.0)
    roll, lat_notes = lateral(d, hat, a)
    if phase == 'DEG':
        roll = max(-3.0, min(3.0, roll))   # değişte kanatlar düz; ince hizalama yalnız küçük yatışla
    if a.water_touch_law == 'yasa6':
        if phase in ('YAKALA', 'ALCAL'):
            k = update_vs_estimate(d, k, dt, a)
        if phase == 'DEG':
            pitch, climb_sp, thr, speed_sp, k = flare_law6_water(d, k, dt, a)
            return Komut(roll, pitch, thr, speed_sp, climb_sp, dict(lat_notes, vs=round(vs_of(k, d.mass_kg, a), 2))), k
    speed_sp = target_speed(d.mass_kg, phase, d.height, a, vs=vs_of(k, d.mass_kg, a))
    if phase == 'DEG' and a.water_touch_law == 'yasa4':
        pitch, climb_sp, thr, k = flare_law4_water(d, k, dt, a)
        return Komut(roll, pitch, thr, speed_sp, climb_sp, lat_notes), k
    if phase == 'DEG':
        pitch, climb_sp, thr_max, k = flare_law3(d, k, dt, a)
        if a.water_flare_throttle == 'hold':
            speed_sp = a.speed_factor_touch * stall_speed(d.mass_kg)   # kayma hızı: gaz hızı tutar, rölantiye inmez
            thr_max = a.thr_max
        thr, t_int = throttle(d, speed_sp, climb_sp, k, dt, a)    # PX4'te TECS gazı, flare üst sınırı altında
        thr = min(thr, thr_max)
        return Komut(roll, pitch, thr, speed_sp, climb_sp, lat_notes), replace(k, thr_int=t_int)
    climb_sp = climb_target(d.height, target_height, phase, a, d.airspeed - speed_sp)
    pitch, p_int = longitudinal(d, climb_sp, phase, k, dt, a)
    thr, t_int = throttle(d, speed_sp, climb_sp, k, dt, a)
    return Komut(roll, pitch, thr, speed_sp, climb_sp, lat_notes), replace(k, pitch_int=p_int, thr_int=t_int)


def capture_ok(d, hat, a=Ayar()):
    """YAKALA → ALÇAL koşulu (süre şartını durum makinesi sayar). (bool, gerekçe)"""
    _, cross, terr = line_errors(d, hat)
    checks = [(abs(cross) < a.capture_cross_m, f'yan sapma {cross:.1f} m'),
              (abs(terr) < a.capture_track_deg, f'yön hatası {terr:.1f}°'),
              (abs(d.roll_deg) < a.capture_roll_deg, f'yatış {d.roll_deg:.1f}°')]
    failed = [why for ok, why in checks if not ok]
    return not failed, ', '.join(failed) if failed else f'yan {cross:.1f} m, yön {terr:.1f}°, yatış {d.roll_deg:.1f}°'


def gate_ok(d, hat, a=Ayar(), k=None):
    """Su+10 m KAPISI. (bool, gerekçe). k: yasa 6'da tahmin edilen tutunma hızı için."""
    _, cross, terr = line_errors(d, hat)
    sp = target_speed(d.mass_kg, 'ALCAL', d.height, a, vs=vs_of(k, d.mass_kg, a))
    checks = [(abs(cross) < a.gate_cross_m, f'yan sapma {cross:.1f} m'),
              (abs(terr) < a.gate_track_deg, f'yön hatası {terr:.1f}°'),
              (abs(d.roll_deg) < a.gate_roll_deg, f'yatış {d.roll_deg:.1f}°'),
              (abs(d.airspeed - sp) < a.gate_speed_tol, f'hız {d.airspeed:.1f} (hedef {sp:.1f})')]
    failed = [why for ok, why in checks if not ok]
    return not failed, ', '.join(failed) if failed else f'yan {cross:.1f} m, yön {terr:.1f}°, yatış {d.roll_deg:.1f}°, hız {d.airspeed:.1f}'


def touch_abort(d, time_in_phase, a=Ayar(), k=None):
    """DEĞ (flare) sırasında PAS GEÇ gerekçesi ya da None. Yükseklik 1 m altına inince artık geri dönülmez.
    Burun sınırlarını yasa uygular: burada yalnız yatış ve tutunma kaybı yakınlığı denetlenir (yasa 3'te gaz flare'de
    rölantide, hız düşer)."""
    if d.height <= 1.0:
        return None
    if abs(d.roll_deg) > a.touch_roll_abort_deg:
        return f'yatış {d.roll_deg:.1f}°, yükseklik {d.height:.1f} m'
    factor = a.touch_stall_factor_law4 if a.water_touch_law in ('yasa4', 'yasa6') else a.touch_stall_factor
    guard = factor * vs_of(k, d.mass_kg, a)
    if d.airspeed < guard:
        return f'hız {d.airspeed:.1f} < {guard:.1f} ({factor:g}·Vs), yükseklik {d.height:.1f} m'
    return None


REHBER_PX4_PARAMS = ('FW_LND_AIRSPD', 'FW_LND_FLALT', 'FW_LND_FL_SINK', 'FW_LND_FL_HOLD', 'FW_LND_FL_KP', 'FW_LND_FL_KI',
                     'FW_LND_FL_TIME', 'FW_P_LIM_MIN', 'FW_P_LIM_MAX', 'FW_THR_IDLE', 'FW_THR_MAX', 'FW_AIRSPD_MIN')


def ayar_from_px4(params, base=None):
    """Rehber ayarlarını PX4'ün iniş parametrelerinden alır (suya değiş pistteki flare ile aynı olsun).
    Su için yalnız burun sınırları (fl_pmin_water/fl_pmax_water) değişmez: gövde basamağından."""
    a = base or Ayar()
    return replace(a, land_airspeed=float(params['FW_LND_AIRSPD']), gate_height_m=float(params['FW_LND_FLALT']),
                   fl_sink=float(params['FW_LND_FL_SINK']),
                   fl_hold_m=float(params['FW_LND_FL_HOLD']), fl_kp=float(params['FW_LND_FL_KP']),
                   fl_ki=float(params['FW_LND_FL_KI']), fl_time_s=float(params['FW_LND_FL_TIME']),
                   p_lim_min=float(params['FW_P_LIM_MIN']), p_lim_max=float(params['FW_P_LIM_MAX']),
                   thr_idle=float(params['FW_THR_IDLE']), thr_max=float(params['FW_THR_MAX']),
                   airspeed_min_air=float(params['FW_AIRSPD_MIN']))
