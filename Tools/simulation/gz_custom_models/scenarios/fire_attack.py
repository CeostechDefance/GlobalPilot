#!/usr/bin/env python3
"""Yangına müdahale senaryosu: arama, kamerayla konum hesabı, hesaplanan noktaya su bırakma, dönüş ve iniş.

Önce simülasyonu başlatın, sonra bu betiği çalıştırın:

    make px4_sitl gz_gasi_ffa_antalya
    python3 Tools/simulation/gz_custom_models/scenarios/fire_attack.py

Uçak yangının yerini bilmez: arama bölgesine gider, kamerada alevi bulur ve konumu kendisi hesaplar.
--fire-lat ve --fire-lon yalnızca sonunda isabeti puanlamak için kullanılır.
Su yalnızca görseldir: uçağın kütlesi ve ağırlık merkezi bırakmadan sonra değişmez.
"""
import argparse
import math
import os
import select
import subprocess
import sys
import termios
import threading
import time

os.environ.setdefault('GZ_IP', '127.0.0.1')

import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
from pymavlink import mavutil

from fire_locator import find_fire, pixel_to_ned, ray_to_ground
from place_wildfire import data_root_dir, ground_height  # fire_locator adds its folder to the path
from otonom.planlayici import rota as R, su_gecisi as SG, pist as PL, birakma as PB, su_alma as SA  # noqa: E402  plan kuralları planlayıcıda (kullanıcı 2026-11-10)

M = mavutil.mavlink
# Per world: runway heading, where the search goes, and the fire's true position (only for scoring the drop).
SITES = {
    'antalya': dict(runway_heading=6.27, land_rel=-0.42, search=(36.9892, 30.6708), fire=(36.9759, 30.6779)),
    # Runway 19, from its north end. 300 m'ye çıkınca doğrudan arama noktasına; tepeler yangını kuzeyden kısmen gizler,
    # görülmezse tarama bacakları arar.
    # land_rel: the runway surface at the landing point against the takeoff point, from the installed world.
    # pattern: the descent to the pattern height ends on the downwind leg, ahead of the base turn, and the final comes in straight at final_angle, just under FW_LND_ANG.
    # search_via (denizden yaklaşma, 36.6250 28.7550) kaldırıldı (kullanıcı 2026-11-09): rota 17,7 km'den ~8 km'ye iner.
    # runway_end: pistin öbür başı (gz_geo_terrain regions/dalaman.json); verilirse planlayıcı (otonom/planlayici/pist.py) iki
    # iniş yönünden (19, 01) yolu kısa olanı seçer, pistin ötesinden dolaşmaz (kullanıcı 2026-11-11).
    'dalaman': dict(runway_heading=195.68, land_rel=-1.54, pattern=dict(right=1500.0, base=-4500.0, rel=150.0, final_angle=2.99), search=(36.6650, 28.7980), fire=(36.66098120, 28.80584463),
                    runway_end=(36.701195, 28.788088), designators=('19', '01')),
}
RUNWAY_HEADING_DEG = SITES['antalya']['runway_heading']  # set from --world in main()
LAND_REL_M = 0.0  # set from --world in main()
PATTERN = dict(right=1200.0, base=-1800.0, rel=120.0)  # the default; set from --world in main()
RUNWAY_DIRS = None   # [PL.PistYonu]: iki iniş yönü; main() runway_end olan dünyada kurar. None: sabit desen
PIST_AYAR = None
PIST_TERRAIN = None  # önbellekli arazi (planlayıcı); main() kurar
HOME_AMSL = None
HOME = None          # kalkış noktası (lat, lon); main() kurar
EXIT_RETURN = None   # pist yönleri tanımsız dünyada bırakma sonrası dönüş denetimi (PB.ring_return); main() kurar
THROTTLE_LIMIT_PARAM = 'FW_THR_MAX'
THROTTLE_LIMIT_EXPECTED = 1.0
# Weight (airframe WEIGHT_BASE 1200, the weight the TECS and airspeeds were tuned at). The tank mass is written to
# WEIGHT_GROSS before arming and on every change
# of WEIGHT_STEP_KG; PX4 scales the TECS throttle gain and the trim/min/stall airspeeds with it (empty: trim 37 m/s).
# Back to WEIGHT_BASE_KG after landing, so the saved parameters match the airframe.
WEIGHT_PARAM = 'WEIGHT_GROSS'
WEIGHT_BASE_KG = 1200.0
EMPTY_MASS_KG = 800.0
WEIGHT_STEP_KG = 50.0
RUN_BEARING_DEG = 315.0   # direction of the pass over the fire
CRUISE_ABOVE_TERRAIN_M = 200.0  # above the highest ground on the route
TAKEOFF_REL_M = 70.0      # climb and circle planning; the takeoff step itself ends at TAKEOFF_ITEM_REL_M
# The takeoff step ends low and the mission climbs on to the next point (user, 2026-11-08; airframe FW_TKO_AIRSPD 45,
# FW_TKO_PITCH_MIN 4, FW_T_CLMB_MAX 4): a high takeoff step climbed at 7-8 m/s and then sagged 3-7 m into the 4 m/s climb.
TAKEOFF_ITEM_REL_M = 10.8
GEAR_UP_REL_M = 20.0       # gear raised by command once above this (user); a mission step could only act at a waypoint
AUTO_LOITER, AUTO_MISSION = 3, 4  # PX4 custom sub modes of AUTO
DROP_AGL_M = 100.0
WAYPOINT_ACCEPT_M = 440.0  # NAV_ACC_RAD. PX4 ignores a mission item's own radius on fixed wing: a point that must be flown up to lies this far beyond it
LAND_ALONG_M = 1200.0     # landing point from the runway start
EXIT_RUN_M = 3000.0       # straight on after the drop (the planner shortens it when the turn back does not clear)
EXIT_TURN_CLEAR_M = 500.0  # only worlds without runway_end: ground checked this far around the end of the straight run
RETURN_ALONG_M = 4000.0   # first point of the return, from the runway start
COMMIT_RANGE_M = 3000.0   # stop searching and turn the run towards the fire from this range
MIN_OBSERVATIONS = 6
RELEASE_S = 1.5           # su boşaltma süresi (kullanıcı 2026-11-11); 45 m/s'de perde hat boyunca ~68 m
HIT_ALONG_M = 165.0
HIT_ACROSS_M = 60.0
MODEL_NAME = 'gasi_ffa_0'
CLIMB_REL_M = 300.0       # the scooping mission climbs on the runway heading and levels off here
SCAN_LEG_M = 6000.0       # scan legs around the search point when the fire was not seen on the way there
SCAN_SPACING_M = 2000.0   # the camera sees about 1.7 km to each side at the 3 km locking range
SCAN_LEGS = 4
SEARCH_FROM_SEQ = 2       # takeoff and level leg come first in climb_items
# Landing gear by MAV_CMD_DO_SET_ACTUATOR (user decision 2026-11-08): airframe 4022 maps the gear servo to Peripheral via
# Actuator Set 1 (SIM_GZ_SV_FUNC5 301), so PX4 no longer moves it on its own. PX4's landing gear function only raised the
# gear once the runway takeoff passed CLIMBOUT, which a short takeoff step skipped (gear stayed down). Values as PX4's
# landing gear function: +1 up, -1 down. A command without a position must use the mission frame.
MAV_CMD_DO_SET_ACTUATOR = 187  # not named in the installed pymavlink dialect
GEAR_DOWN_MAX_WHEEL_Z = -0.6   # left wheel: down -0.69, up about -0.31
# Düz uçuş bacağı yalnız CLIMB_REL_M'ye çıkacak kadar (kullanıcı 2026-11-09: 300 m'den sonra düz uçuşa gerek yok,
# doğrudan aleve yönelsin). Önceden 8 km'ydi. Yol: tırmanış süresi × seyir hızı + kalkış koşusu payı.
LEVEL_END_M = None  # aşağıda CLIMB_RATE_MPS ve CRUISE_SPEED_MPS tanımlandıktan sonra hesaplanır
WATER_LEG_RETURN_MIN_AMSL_M = 30.0  # below this after a failed water leg the aircraft is still on the sea
ROUTE_STEP_M = 100.0
ROUTE_MIN_MARGIN_M = 50.0
LAND_MIN_AMSL_M = 5.0
CIRCLE_RADIUS_M = 500.0   # wider than the 25 degree bank needs at 45 m/s (443 m)
CLIMB_RATE_MPS = 3.45     # measured in the scenario's own cruise climb
CRUISE_SPEED_MPS = 45.0
LEVEL_END_M = CLIMB_REL_M / CLIMB_RATE_MPS * CRUISE_SPEED_MPS + 300.0  # ~4,2 km (300 m kalkış koşusu payı)
# Boş uçağın hızı: PX4 trim hızını ağırlıkla küçültür (800 kg: ~37 m/s). Piste dönüşte dönüş yarıçapı buradan.
EMPTY_SPEED_MPS = CRUISE_SPEED_MPS * math.sqrt(EMPTY_MASS_KG / WEIGHT_BASE_KG)
# Su bırakma geçişi, düz çıkış ve dönüş kuralları planlayıcıda: otonom/planlayici/birakma.py (2026-11-27).
BIRAKMA_AYAR = PB.BirakmaAyar(commit_range_m=COMMIT_RANGE_M, accept_m=WAYPOINT_ACCEPT_M, margin_m=ROUTE_MIN_MARGIN_M,
                              step_m=ROUTE_STEP_M, climb_gradient=CLIMB_RATE_MPS / CRUISE_SPEED_MPS,
                              exit_runs_m=(EXIT_RUN_M, 2500.0, 2000.0, 1500.0, 1000.0))
CAMERA_TOPIC = '/world/{world}/model/gasi_ffa_0/link/base_link/sensor/fire_camera/image'
WATER_CMD = '/model/gasi_ffa_0/link/base_link/particle_emitter/water/cmd'
FIRE_CMD = '/model/wildfire/link/fire_link/particle_emitter/{}/cmd'
WATER_RELEASE = '/model/gasi_ffa_0/water/release'
# Suya geçiş kuralları planlayıcıda: otonom/planlayici/su_gecisi.py (yerden en az 100 m, alçalış payı koruyan ilk
# noktada, su küçükse karada çemberde alçalış). Burada sayı tutulmaz.
SU_GECIS_AYAR = SG.SuGecisAyar(accept_m=WAYPOINT_ACCEPT_M, circle_radius_m=CIRCLE_RADIUS_M)
WATER_RELEASE_KGS = 270.0  # 400 kg ~1,48 s'de: RELEASE_S (1,5 s) içinde tank boşalır


def offset(lat, lon, north, east):
    return lat + north / 111320.0, lon + east / (111320.0 * math.cos(math.radians(lat)))


def along(lat, lon, bearing_deg, distance, right=0.0):
    b = math.radians(bearing_deg)
    return offset(lat, lon, distance * math.cos(b) - right * math.sin(b), distance * math.sin(b) + right * math.cos(b))


def distance_m(lat1, lon1, lat2, lon2):
    return math.hypot((lat2 - lat1) * 111320.0, (lon2 - lon1) * 111320.0 * math.cos(math.radians(lat1)))


def route_samples(lat, lon, target, region):
    """(distance, ground height) every ROUTE_STEP_M along a straight route, or None when the terrain is not available."""
    length = distance_m(lat, lon, target[0], target[1])
    root = data_root_dir()
    samples = []
    steps = max(1, int(length / ROUTE_STEP_M))
    for i in range(1, steps + 1):
        k = i / steps
        try:
            samples.append((k * length, ground_height(root, region, lat + (target[0] - lat) * k, lon + (target[1] - lon) * k)))
        except (OSError, SystemExit):
            return None
    return samples


def plan_search(lat, lon, alt_amsl, target, region):
    """Cruise height from the terrain on the route. All heights are above sea level; None when the terrain is not available.

    climb_first: the straight climb leaves too little room over land, so the aircraft circles up where it is first.
    safe: False when circling is not an option either, because the aircraft is too low over the ground under it.
    The sea does not count as terrain: the aircraft leaves the water leg low, and that is not a terrain risk.
    """
    samples = route_samples(lat, lon, target, region)
    try:
        here = ground_height(data_root_dir(), region, lat, lon)
    except (OSError, SystemExit):
        return None
    if samples is None:
        return None
    crest = max(ground for _, ground in samples)
    cruise = crest + CRUISE_ABOVE_TERRAIN_M
    land = [(x, ground) for x, ground in samples if ground >= LAND_MIN_AMSL_M]
    margin = min((min(alt_amsl + CLIMB_RATE_MPS * x / CRUISE_SPEED_MPS, cruise) - ground for x, ground in land), default=float('inf'))
    return dict(crest=crest, cruise=cruise, margin=margin, climb_first=margin < ROUTE_MIN_MARGIN_M,
                safe=margin >= ROUTE_MIN_MARGIN_M or here < LAND_MIN_AMSL_M or alt_amsl - here >= ROUTE_MIN_MARGIN_M)


def circle_clear(center, radius, floor_amsl, region, margin=ROUTE_MIN_MARGIN_M):
    """True when the ground around a descent circle stays margin under its lowest height."""
    root = data_root_dir()
    try:
        highest = max(float(ground_height(root, region, *along(*center, bearing, r)))
                      for bearing in range(0, 360, 30) for r in (0.0, radius, 1.7 * radius))
    except (OSError, SystemExit):
        return False
    return highest + margin <= floor_amsl


def read_param(vehicle, name, timeout=3.0):
    """Value of a PX4 parameter, or None when PX4 does not answer. Only for use on the ground: it holds up the telemetry."""
    vehicle.link.mav.param_request_read_send(vehicle.link.target_system, vehicle.link.target_component, name.encode(), -1)
    end = time.time() + timeout
    while time.time() < end:
        msg = vehicle.link.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if msg is not None and msg.param_id == name:
            return float(msg.param_value)
    return None


def set_weight(vehicle, kg):
    """WEIGHT_GROSS'u yazar ve okuyarak doğrular; değer ya da None. Yalnız yerde (okuma telemetriyi bekletir)."""
    for _ in range(3):
        vehicle.link.mav.param_set_send(vehicle.link.target_system, vehicle.link.target_component, WEIGHT_PARAM.encode(), kg, M.MAV_PARAM_TYPE_REAL32)
        value = read_param(vehicle, WEIGHT_PARAM)
        if value is not None and abs(value - kg) < 0.5:
            return value
    return None


class Vehicle:
    def __init__(self, url):
        self.link = mavutil.mavlink_connection(url)
        self.last_heartbeat = 0.0
        self.state = dict(armed=False, mode=0, lat=0.0, lon=0.0, rel=0.0, alt=0.0, airspeed=0.0, throttle=0, landed=0, seq=-1, roll=0.0, pitch=0.0, yaw=0.0)

    def wait_connection(self, timeout):
        # Yalnız otopilotun (bileşen 1) HEARTBEAT'i: yer testindeki motor çalıştırmasından sonra PX4 ESC_INFO/ESC_STATUS
        # yayınlıyor; pymavlink'in v1 sözlüğü bunları tanımadığı için sistem 0 okuyor ve hedef sistemi 0'a çekiyordu.
        hb = self.link.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout,
                                  condition='HEARTBEAT.autopilot != 8')  # 8 = MAV_AUTOPILOT_INVALID (GCS/yardımcı)
        if hb is None:
            sys.exit('PX4 bulunamadı. Önce simülasyonu başlatın: make px4_sitl gz_gasi_ffa_antalya')
        self.link.target_system, self.link.target_component = hb.get_srcSystem(), hb.get_srcComponent()

    def update(self, seconds):
        end = time.time() + seconds
        s = self.state
        while time.time() < end:
            if time.time() - self.last_heartbeat > 1.0:
                self.last_heartbeat = time.time()
                self.link.mav.heartbeat_send(M.MAV_TYPE_GCS, M.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            msg = self.link.recv_match(blocking=True, timeout=0.1)
            if msg is None:
                continue
            kind = msg.get_type()
            if kind == 'GLOBAL_POSITION_INT':
                s.update(lat=msg.lat / 1e7, lon=msg.lon / 1e7, rel=msg.relative_alt / 1e3, alt=msg.alt / 1e3)
            elif kind == 'ATTITUDE':
                s.update(roll=msg.roll, pitch=msg.pitch, yaw=msg.yaw)
            elif kind == 'VFR_HUD':
                s.update(airspeed=msg.airspeed, throttle=msg.throttle)
            elif kind == 'EXTENDED_SYS_STATE':
                s.update(landed=msg.landed_state)
            elif kind == 'MISSION_CURRENT':
                s.update(seq=msg.seq)
            elif kind == 'HEARTBEAT' and msg.get_srcSystem() == self.link.target_system and msg.get_srcComponent() == 1:
                s.update(armed=bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED), mode=msg.custom_mode)
            elif kind == 'STATUSTEXT' and msg.severity <= M.MAV_SEVERITY_WARNING and 'denied' not in msg.text:
                print(f'  PX4: {msg.text.strip()}', flush=True)

    def upload(self, items):
        self.link.mav.mission_count_send(self.link.target_system, self.link.target_component, len(items), 0)
        end = time.time() + 20
        while time.time() < end:
            msg = self.link.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK'], blocking=True, timeout=1)
            if msg is None:
                continue
            if msg.get_type() == 'MISSION_ACK':
                if msg.type != M.MAV_MISSION_ACCEPTED:
                    sys.exit(f'Görev reddedildi (kod {msg.type})')
                return
            item = items[msg.seq]
            self.link.mav.mission_item_int_send(
                self.link.target_system, self.link.target_component, msg.seq, item.get('frame', M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT),
                item['cmd'], int(msg.seq == 0), 1, item.get('param1', 0), item.get('radius', 0), item.get('param3', 0), item.get('param4', float('nan')),
                int(item['lat'] * 1e7), int(item['lon'] * 1e7), item['alt'], 0)
        sys.exit('Görev yüklenemedi: PX4 yanıt vermedi')

    def command(self, command, *params):
        params = list(params) + [0.0] * (7 - len(params))
        self.link.mav.command_long_send(self.link.target_system, self.link.target_component, command, 0, *params)


def upload_in_flight(v, items):
    """Uçuşta yeni görev: görev modunda kalınarak yüklenir; yeni görev 0. adımdan başlar.

    Vehicle.upload 0. adımı 'current' işaretler: PX4 yeni görevin sıra numarasını bundan alır (mavlink_mission.cpp
    _transfer_current_seq). İşaret yokken PX4 eski sıra numarasını koruyordu (mission_base.cpp: current_seq < 0 ise eskisi):
    2026-11-10'da yeni görevin 3. adımı 'Takımı aç' çalıştı. Onu önlemek için eklenen AUTO.LOITER geçişi tırmanışı kesiyordu:
    bekleme o anki irtifayı (33 m) hedef aldı, gaz %40'a düştü, uçak 81 m'den 76 m'ye indi. Bekleme artık yok."""
    v.upload(items)
    v.update(0.5)
    if not set_auto_mode(v, AUTO_MISSION, timeout=10):
        print('UYARI: görev moduna geçilemedi', flush=True)


def set_auto_mode(v, sub_mode, timeout=15):
    """Switches to an AUTO sub mode and returns only after the heartbeat reports it."""
    wanted = (4 << 16) | (sub_mode << 24)
    end = time.time() + timeout
    while v.state['mode'] != wanted and time.time() < end:
        v.command(M.MAV_CMD_DO_SET_MODE, 1, 4, sub_mode)
        v.update(1)
    return v.state['mode'] == wanted


def wait_enter(v, prompt):
    """Waits for Enter, keeping the MAVLink link alive. Presses made before the prompt do not count."""
    termios.tcflush(sys.stdin, termios.TCIFLUSH)
    print(prompt, flush=True)
    while not select.select([sys.stdin], [], [], 0)[0]:
        v.update(0.2)
    sys.stdin.readline()


class Camera:
    def __init__(self, world):
        self.lock = threading.Lock()
        self.message = None
        self.node = Node()
        self.node.subscribe(Image, CAMERA_TOPIC.format(world=world), self.on_image)

    def on_image(self, msg):
        with self.lock:
            self.message = msg

    def frame(self):
        with self.lock:
            msg, self.message = self.message, None
        if msg is None:
            return None
        return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)


def emitter(topic, payload, message='gz.msgs.ParticleEmitter'):
    subprocess.run(['gz', 'topic', '-t', topic, '-m', message, '-p', payload], check=False, timeout=10)


def pattern_items(plan):
    """Planlayıcının dönüş deseni (PL.PistDeseni) görev adımlarına: ilk dönüşün yay noktaları, sonra desen. Takım ilk desen
    noktasından (taban ya da son yaklaşma başı) sonra açılır."""
    def item(name, cmd, point, alt):
        return dict(name=name, cmd=cmd, lat=point[0], lon=point[1], alt=alt)

    wp = M.MAV_CMD_NAV_WAYPOINT
    items = [item(name, wp, point, alt) for name, point, alt in plan.transition]
    gear_done = False
    for name, point, alt in plan.points:
        if name == 'İniş':
            items.append(item(name, M.MAV_CMD_NAV_LAND, point, alt))
            continue
        items.append(item(name, wp, point, alt))
        if not gear_done:
            items.append(gear_item(up=False))
            gear_done = True
    return items


def describe_return(plan):
    turn = f', {plan.turn} dönüş (en az arazi payı {plan.min_clearance_m:.0f} m)' if plan.turn else ''
    return f'Piste dönüş (planlayıcı): {plan.runway.name} pistine {plan.entry}{turn}, yol {plan.length_m / 1000:.1f} km'


def plan_return(start, course, first_rel, actual_rel=None, at_aircraft=False):
    """Planlayıcıdan piste dönüş deseni (PL.PistDeseni). start görev noktasıysa PX4 onu NAV_ACC_RAD erken bırakır, dönüş
    o kadar erken de başlayabilir; uçağın kendi konumundan (at_aircraft) planlanan dönüşte bu pay yok."""
    return PL.plan_runway_return(tuple(start), first_rel, course, RUNWAY_DIRS, HOME_AMSL, PIST_TERRAIN, PIST_AYAR,
                                 actual_rel_m=actual_rel, early_m=0.0 if at_aircraft else WAYPOINT_ACCEPT_M)


def exit_return(point, course, hold_amsl, actual_amsl):
    """Bırakma sonrası çıkış noktasından piste dönüş, planlayıcının (PB.plan_exit) return_fn'i: (ok, adımlar ve açıklama,
    gerekçe). Pist yönleri tanımsız dünyada eski denetim (yalnız çıkış noktası çevresinde halka) ve sabit desen."""
    if RUNWAY_DIRS:
        plan = plan_return(point, course, hold_amsl - HOME_AMSL, actual_amsl - HOME_AMSL)
        if not plan.ok:
            return False, None, '; '.join(f'{n} ({w})' for n, w in plan.rejected)
        return True, dict(items=pattern_items(plan), text=describe_return(plan)), None
    ok, first_rel, why = EXIT_RETURN(point, course, hold_amsl, actual_amsl)
    if not ok:
        return False, None, why
    return True, dict(items=fixed_return_items(HOME[0], HOME[1], first_rel), text=f'Piste dönüş: sabit desen, ilk nokta {first_rel:.0f} m'), None


def return_items(home_lat, home_lon, first_rel=300.0, start=None, course=None, actual_rel=None, at_aircraft=False):
    """Piste dönüş adımları. start (dönüşün başladığı nokta) ve course (oradaki yön) verilirse ve dünyada iki iniş yönü
    tanımlıysa desen planlayıcıdan gelir: yolu kısa olan yön, uçağın bulunduğu yerden taban ya da düz giriş; ilk dönüş
    planlanan yönde ve araziyle denetlenmiş. first_rel ilk dönüşte istenen yükseklik, actual_rel uçağın oradaki gerçek
    yüksekliği (verilmezse first_rel). Planlayıcı arazi payı tutan aday bulamazsa sabit desen (pistin ötesinden dolaşan)."""
    if RUNWAY_DIRS and start is not None:
        plan = plan_return(start, course, first_rel, actual_rel, at_aircraft)
        if plan.ok:
            print(describe_return(plan) + (f'; elenen: {"; ".join(f"{n} ({w})" for n, w in plan.rejected)}' if plan.rejected else ''),
                  flush=True)
            return pattern_items(plan)
        print('Piste dönüş: planlayıcı arazi payı tutan desen bulamadı, sabit desen. Nedenler: '
              + '; '.join(f'{n} ({w})' for n, w in plan.rejected), flush=True)
    return fixed_return_items(home_lat, home_lon, first_rel)


def fixed_return_items(home_lat, home_lon, first_rel):
    """Sabit dönüş deseni: pistin ötesinde ilk nokta, taban, son yaklaşma, iniş. Arazi ve dönüş yönü denetlenmez."""
    def item(name, cmd, point, alt):
        return dict(name=name, cmd=cmd, lat=point[0], lon=point[1], alt=alt)

    wp = M.MAV_CMD_NAV_WAYPOINT
    runway = (home_lat, home_lon, RUNWAY_HEADING_DEG)
    right, base, rel = PATTERN['right'], PATTERN['base'], PATTERN['rel']
    angle = PATTERN.get('final_angle')
    glide = [item('Süzülüş başı', wp, along(*runway, LAND_ALONG_M - (rel - LAND_REL_M) / math.tan(math.radians(angle))), rel)] if angle else []
    return [
        item('Piste dönüş', wp, along(*runway, RETURN_ALONG_M, right), first_rel),
        item('Taban bacağı', wp, along(*runway, base, right), rel),
        gear_item(up=False),
        item('Son yaklaşma', wp, along(*runway, base), rel),
    ] + glide + [
        item('İniş', M.MAV_CMD_NAV_LAND, along(*runway, LAND_ALONG_M), LAND_REL_M),
    ]


def gear_item(up):
    """Mission step: raise (up) or lower the landing gear. Actuator Set 1 only (index 0); the other channels stay."""
    nan = float('nan')
    return dict(name='Takımı topla' if up else 'Takımı aç', cmd=MAV_CMD_DO_SET_ACTUATOR, lat=0.0, lon=0.0, alt=0.0,
                param1=1.0 if up else -1.0, radius=nan, param4=nan, frame=M.MAV_FRAME_MISSION)


def left_wheel_z(world, model, timeout=10.0):
    """Height of the left wheel link in the model, from the gz pose topic, or None."""
    found = []
    done = threading.Event()

    def on_links(msg):
        for pose in msg.pose:
            if pose.name == 'left_wheel':
                found.append(pose.position.z)
                done.set()
                return
    node = Node()
    node.subscribe(Pose_V, f'/world/{world}/dynamic_pose/info', on_links)
    done.wait(timeout)
    return found[-1] if found else None


def lower_gear_on_ground(v, world, model, timeout=15.0):
    """The actuator set value is undefined at start and the servo can sit half way: command the gear down and check it."""
    nan = float('nan')
    end = time.time() + timeout
    while time.time() < end:
        v.command(MAV_CMD_DO_SET_ACTUATOR, -1.0, nan, nan, nan, nan, nan, 0)
        v.update(1)
        z = left_wheel_z(world, model, 3.0)
        if z is not None and z < GEAR_DOWN_MAX_WHEEL_Z:
            return z
    return None


def takeoff_item(home_lat, home_lon, alt=TAKEOFF_ITEM_REL_M):
    point = along(home_lat, home_lon, RUNWAY_HEADING_DEG, 800)
    return dict(name='Kalkış', cmd=M.MAV_CMD_NAV_TAKEOFF, lat=point[0], lon=point[1], alt=alt)


def plan_scan(home, home_amsl, center, bearing, region):
    """Parallel legs around the search point, at one height clear of all their terrain.

    Returns (mission items, height of the first return point), or None when the terrain is not available.
    """
    points = []
    for leg in range(SCAN_LEGS):
        right = (leg - (SCAN_LEGS - 1) / 2) * SCAN_SPACING_M
        ends = [along(*center, bearing, -SCAN_LEG_M / 2, right), along(*center, bearing, SCAN_LEG_M / 2, right)]
        points += ends if leg % 2 == 0 else ends[::-1]
    crest = -float('inf')
    for start, end in zip([center] + points, points):
        samples = route_samples(*start, end, region)
        if samples is None:
            return None
        crest = max(crest, max(ground for _, ground in samples))
    back = route_samples(*points[-1], along(*home, RUNWAY_HEADING_DEG, RETURN_ALONG_M, PATTERN['right']), region)
    if back is None:
        return None
    scan_rel = max(crest + CRUISE_ABOVE_TERRAIN_M - home_amsl, CLIMB_REL_M)
    items = [dict(name=f'Tarama {k + 1}/{len(points)}', cmd=M.MAV_CMD_NAV_WAYPOINT, lat=point[0], lon=point[1], alt=scan_rel) for k, point in enumerate(points)]
    return items, max(300.0, max(ground for _, ground in back) + CRUISE_ABOVE_TERRAIN_M - home_amsl)


def search_items(home_lat, home_lon, search_lat, search_lon, cruise_rel, climb_at=None, scan=None, return_rel=300.0):
    climb = [dict(name='Çember atarak tırmanış', cmd=M.MAV_CMD_NAV_LOITER_TO_ALT, lat=climb_at[0], lon=climb_at[1], alt=cruise_rel, radius=CIRCLE_RADIUS_M)] if climb_at else []
    route = [(search_lat, search_lon)] + [(it['lat'], it['lon']) for it in (scan or [])]
    prev = route[-2] if len(route) > 1 else (home_lat, home_lon)
    last_rel = scan[-1]['alt'] if scan else cruise_rel   # dönüş başında uçağın yüksekliği
    return climb + [dict(name='Arama bölgesine uçuş', cmd=M.MAV_CMD_NAV_WAYPOINT, lat=search_lat, lon=search_lon, alt=cruise_rel)] + (scan or []) + \
        return_items(home_lat, home_lon, max(return_rel, last_rel), route[-1], bearing_deg(*prev, *route[-1]), actual_rel=last_rel)


def climb_items(home_lat, home_lon):
    """Takeoff for the scooping mission: the takeoff step ends at TAKEOFF_ITEM_REL_M and the level leg climbs on to
    CLIMB_REL_M (FW_T_CLMB_MAX 4 m/s), then level flight. The search follows it."""
    level_end = along(home_lat, home_lon, RUNWAY_HEADING_DEG, LEVEL_END_M)
    return [takeoff_item(home_lat, home_lon),
            dict(name='Düz uçuş', cmd=M.MAV_CMD_NAV_WAYPOINT, lat=level_end[0], lon=level_end[1], alt=CLIMB_REL_M)]


def attack_mission(target, bearing_deg, run_rel, cruise_rel, exit_plan):
    """Su bırakma geçişi, düz çıkış ve piste dönüş. exit_plan: planlayıcının çıkışı (PB.Cikis); dönüş adımları onda."""
    wp = M.MAV_CMD_NAV_WAYPOINT
    run_end = along(target[0], target[1], bearing_deg, WAYPOINT_ACCEPT_M)  # the climb starts over the target, not before it
    return [
        dict(name='Su bırakma geçişi', cmd=wp, lat=run_end[0], lon=run_end[1], alt=run_rel),
        # irtifa: seyir, uçak oraya yetişemiyorsa yetişebildiği (PX4 irtifası tutmayan noktada döner; planlayıcı)
        dict(name='Geçişten çıkış ve tırmanma', cmd=wp, lat=exit_plan.point[0], lon=exit_plan.point[1],
             alt=min(cruise_rel, exit_plan.point_alt_amsl - HOME_AMSL)),
    ] + exit_plan.ret['items']


def strike_items(target, bearing, run_rel, cruise_rel, exit_plan, climb_at=None):
    """From the water to the fire: stays at cruise height over the terrain and descends only on the final run."""
    approach = along(target[0], target[1], bearing, WAYPOINT_ACCEPT_M - COMMIT_RANGE_M)  # the descent starts at COMMIT_RANGE_M, as the planner checks
    climb = [dict(name='Çember atarak tırmanış', cmd=M.MAV_CMD_NAV_LOITER_TO_ALT, lat=climb_at[0], lon=climb_at[1], alt=cruise_rel, radius=CIRCLE_RADIUS_M)] if climb_at else []
    return climb + [dict(name='Yangına yaklaşma', cmd=M.MAV_CMD_NAV_WAYPOINT, lat=approach[0], lon=approach[1], alt=cruise_rel)] + \
        attack_mission(target, bearing, run_rel, cruise_rel, exit_plan)


def plan_attack(target, preferred, run_rel, cruise_rel, bearings=None):
    """Planlayıcıdan su bırakma geçişi (PB): yön, düz çıkış ve oradan araziyle denetlenmiş piste dönüş.
    bearings verilirse yalnız bu yönler sırayla denenir (düzeltme: yeni yön, olmazsa uçulan yön); verilmezse tercih edilen
    yöne yakından uzağa. Dönüş: (yön, PB.Cikis) ya da gerekçe metni."""
    run_amsl, cruise_amsl = HOME_AMSL + run_rel, HOME_AMSL + cruise_rel
    if bearings is None:
        plan = PB.plan_run(target, preferred, run_amsl, cruise_amsl, PIST_TERRAIN, exit_return, BIRAKMA_AYAR)
        if plan.ok:
            return plan.bearing_deg, plan.exit
        return '; '.join(f'{b:.0f}°: {w[:160]}' for b, w in plan.rejected[:3])
    reasons = []
    for bearing in bearings:
        exit_plan = PB.plan_exit(target, bearing, run_amsl, cruise_amsl, PIST_TERRAIN, exit_return, BIRAKMA_AYAR)
        if exit_plan.ok:
            return bearing, exit_plan
        reasons.append(f'{bearing:.0f}°: {exit_plan.reason[:160]}')
    return '; '.join(reasons)


class Estimate:
    """Fire position from terrain intersections. Close, steep views count far more, because the error grows with range squared."""

    def __init__(self):
        self.obs = []

    def add(self, lat, lon, ground, rng):
        self.obs.append((lat, lon, ground, rng))

    def position(self):
        a = np.array(self.obs[-40:])
        w = 1.0 / np.maximum(a[:, 3], 100.0) ** 4
        lat, lon = a[:, 0], a[:, 1]
        return float((lat * w).sum() / w.sum()), float((lon * w).sum() / w.sum()), float((a[:, 2] * w).sum() / w.sum())


def bearing_deg(lat1, lon1, lat2, lon2):
    return math.degrees(math.atan2((lon2 - lon1) * math.cos(math.radians(lat1)), lat2 - lat1)) % 360


def track_errors(lat, lon, target, bearing):
    """Along and across track distance of a point from the target; positive along means past it."""
    n = (lat - target[0]) * 111320.0
    e = (lon - target[1]) * 111320.0 * math.cos(math.radians(target[0]))
    b = math.radians(bearing)
    return n * math.cos(b) + e * math.sin(b), -n * math.sin(b) + e * math.cos(b)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--search-lat', type=float, help='arama bölgesi; varsayılan dünyaya göre')
    ap.add_argument('--search-lon', type=float)
    ap.add_argument('--fire-lat', type=float, help='yalnızca puanlama için gerçek konum; varsayılan dünyaya göre')
    ap.add_argument('--fire-lon', type=float)
    ap.add_argument('--url', default='udpin:0.0.0.0:14540')
    ap.add_argument('--timeout', type=float, default=3600.0, help='saniye')
    ap.add_argument('--scoop', action='store_true', help='yangını bulduktan sonra denize inip su al, sonra yangına dön (water_leg.py)')
    ap.add_argument('--offboard-hizalanma', metavar='KAYIT.jsonl', default=None,
                    help='su bölümünde suya hizalanma ve değiş offboard (otonom.hizalanma; flare PX4 yasa 3, gaz hızı tutar); karar kaydı bu dosyaya')
    ap.add_argument('--key-start', action='store_true', help='Enter ile arm et, ikinci Enter ile kalk (terminal gerekir)')
    ap.add_argument('--world', default='antalya', help='çalışan Gazebo dünyası; arazi verisi aynı adlı bölgeden okunur')
    ap.add_argument('--sysid', type=int, default=1, help='beklenen PX4 sistem numarası; başkasına komut gönderilmez')
    args = ap.parse_args()
    global RUNWAY_HEADING_DEG, LAND_REL_M, PATTERN
    site = SITES.get(args.world)
    if site is None:
        sys.exit(f'"{args.world}" dünyası için pist bilgisi yok: SITES')
    RUNWAY_HEADING_DEG = site['runway_heading']
    LAND_REL_M = site.get('land_rel', 0.0)
    PATTERN = site.get('pattern', PATTERN)
    args.search_lat = site['search'][0] if args.search_lat is None else args.search_lat
    args.search_lon = site['search'][1] if args.search_lon is None else args.search_lon
    args.fire_lat = site['fire'][0] if args.fire_lat is None else args.fire_lat
    args.fire_lon = site['fire'][1] if args.fire_lon is None else args.fire_lon

    if args.key_start and not sys.stdin.isatty():
        sys.exit('--key-start terminal ister. Komut gönderilmedi.')
    v = Vehicle(args.url)
    camera = Camera(args.world)
    # Two simulators can share this machine: nothing is sent until this is known to be the right one.
    if os.environ.get('GZ_PARTITION'):
        sys.exit(f'GZ_PARTITION={os.environ["GZ_PARTITION"]} ayarlı; bu senaryo varsayılan Gazebo bölümü için. Komut gönderilmedi.')
    print('PX4 bekleniyor...', flush=True)
    v.wait_connection(120)
    if v.link.target_system != args.sysid:
        sys.exit(f'Yanlış araç: sistem {v.link.target_system}, beklenen {args.sysid}. Komut gönderilmedi.')
    print(f'Bağlandı: sistem {v.link.target_system}, {MODEL_NAME}, dünya {args.world}', flush=True)
    if args.scoop:
        print('Etiket: Kütle değişimi yok. Su alma yalnızca temas olarak ölçülür, su bırakma görseldir.', flush=True)
    v.update(5)
    wait_end = time.time() + 120
    while v.state['lat'] == 0.0 and time.time() < wait_end:
        v.update(1)
    s = v.state
    if s['armed']:
        v.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 0)
        v.update(3)
    home = (s['lat'], s['lon'])
    home_amsl = s['alt']
    search_point = (args.search_lat, args.search_lon)
    global RUNWAY_DIRS, PIST_AYAR, PIST_TERRAIN, HOME_AMSL, HOME, EXIT_RETURN
    root = data_root_dir()
    gh = lambda la, lo: float(ground_height(root, args.world, la, lo))  # noqa: E731
    HOME, HOME_AMSL = home, home_amsl
    PIST_TERRAIN = SG.OnbellekliArazi(gh)
    # Pist yönleri tanımsız dünya: bırakma sonrası eski dönüş denetimi (çıkış çevresinde halka) ve sabit desen
    EXIT_RETURN = PB.ring_return(PIST_TERRAIN, along(*home, RUNWAY_HEADING_DEG, RETURN_ALONG_M, PATTERN['right']), home_amsl,
                                 EXIT_TURN_CLEAR_M, 300.0, CRUISE_ABOVE_TERRAIN_M, BIRAKMA_AYAR)
    if site.get('runway_end'):
        # İki iniş yönü (planlayıcı yolu kısa olanı seçer). İlk yön kalkış yönü, eşik kalkış noktası; öbür yönün eşiği
        # pistin öbür başı, iniş noktası yüksekliği arazi farkıyla. İlk dönüş boş uçağın hızıyla (dönüşler genelde boş).
        # Dönüşün PX4 sınırları parametrelerden (kullanıcı 2026-11-27); yerde okunur, okunamazsa kalkılmaz.
        px4 = {name: read_param(v, name) for name in ('NAV_FW_ALT_RAD', 'FW_T_SINK_MAX', 'FW_R_LIM')}
        missing = [name for name, value in px4.items() if value is None]
        if missing:
            sys.exit(f'PX4 parametreleri okunamadı: {", ".join(missing)}. Kalkılmadı.')
        PIST_AYAR = PL.PistAyar(land_along_m=LAND_ALONG_M, final_m=-PATTERN['base'], base_m=PATTERN['right'],
                                pattern_rel_m=PATTERN['rel'], final_angle_deg=PATTERN.get('final_angle', 2.99),
                                turn_radius_m=CIRCLE_RADIUS_M, margin_m=ROUTE_MIN_MARGIN_M, airspeed=EMPTY_SPEED_MPS,
                                bank_deg=px4['FW_R_LIM'], accept_m=WAYPOINT_ACCEPT_M, corridor_m=WAYPOINT_ACCEPT_M + 60.0,
                                climb_gradient=CLIMB_RATE_MPS / CRUISE_SPEED_MPS,
                                sink_max_gradient=px4['FW_T_SINK_MAX'] / EMPTY_SPEED_MPS,
                                descent_max_gradient=0.8 * px4['FW_T_SINK_MAX'] / EMPTY_SPEED_MPS,
                                alt_accept_m=px4['NAV_FW_ALT_RAD'])
        print(f"Dönüş planı PX4 sınırları: NAV_FW_ALT_RAD {px4['NAV_FW_ALT_RAD']:g} m, FW_T_SINK_MAX {px4['FW_T_SINK_MAX']:g} m/s, "
              f"FW_R_LIM {px4['FW_R_LIM']:g}°", flush=True)
        back_heading = (RUNWAY_HEADING_DEG + 180.0) % 360.0
        land_fwd = along(*home, RUNWAY_HEADING_DEG, LAND_ALONG_M)
        land_back = along(*site['runway_end'], back_heading, LAND_ALONG_M)
        names = site.get('designators', ('ileri', 'geri'))
        RUNWAY_DIRS = [PL.PistYonu(names[0], home, RUNWAY_HEADING_DEG, LAND_REL_M),
                       PL.PistYonu(names[1], tuple(site['runway_end']), back_heading,
                                   LAND_REL_M + gh(*land_back) - gh(*land_fwd))]
        print(f'Pist yönleri: {names[0]} ({RUNWAY_HEADING_DEG:.0f}°), {names[1]} ({back_heading:.0f}°, iniş noktası '
              f'{RUNWAY_DIRS[1].land_rel_m:+.2f} m); dönüşte yolu kısa olan seçilir', flush=True)
    # Checked on the ground: a missing terrain must stop the mission before takeoff, not during it.
    plan = plan_search(*home, home_amsl + TAKEOFF_REL_M, search_point, args.world)
    if plan is None:
        sys.exit(f'"{args.world}" bölgesinin arazi verisi bu rotada yok ({data_root_dir()}). Kalkılmadı.')
    print(f'Rotadaki en yüksek arazi {plan["crest"]:.0f} m, seyir {plan["cruise"]:.0f} m (deniz seviyesinden)', flush=True)
    cruise_rel = plan['cruise'] - home_amsl
    if args.scoop:
        samples = route_samples(*home, along(*home, RUNWAY_HEADING_DEG, LEVEL_END_M), args.world)
        if samples is None:
            sys.exit(f'"{args.world}" bölgesinin arazi verisi tırmanış hattında yok. Kalkılmadı.')
        climb_margin = home_amsl + CLIMB_REL_M - max(ground for _, ground in samples)
        if climb_margin < ROUTE_MIN_MARGIN_M:
            sys.exit(f'Tırmanış hattında arazi payı yetersiz ({climb_margin:.0f} m). Kalkılmadı.')
        # Imported here: the plain fire run must not depend on the water module.
        import water_leg
        water_leg.WORLD = args.world
        water_leg.WATER_DIR = os.path.join(data_root_dir(), args.world, 'water')
        water_leg.GROUND_ROLL_HEADING_DEG = RUNWAY_HEADING_DEG
        water_leg.LAND_REL_M = LAND_REL_M
        level_end = along(*home, RUNWAY_HEADING_DEG, LEVEL_END_M)
        via = site.get('search_via')
        first = via or search_point
        plan = plan_search(*level_end, home_amsl + CLIMB_REL_M, first, args.world)
        last = plan_search(*via, home_amsl + CLIMB_REL_M, search_point, args.world) if via else plan
        if plan is None or last is None:
            sys.exit(f'"{args.world}" bölgesinin arazi verisi arama rotasında yok. Kalkılmadı.')
        if plan['climb_first'] and not plan['safe']:
            sys.exit(f'Arama rotasında arazi payı yetersiz ({plan["margin"]:.0f} m). Kalkılmadı.')
        if plan['climb_first'] and not circle_clear(level_end, CIRCLE_RADIUS_M, home_amsl + CLIMB_REL_M, args.world):
            sys.exit(f'Tırmanış çemberinde {CIRCLE_RADIUS_M:.0f} m yarıçapta arazi payı yok. Kalkılmadı.')
        cruise_rel = max(plan['cruise'], last['cruise']) - home_amsl
        cruise_rel = max(cruise_rel, CLIMB_REL_M)
        items = climb_items(*home)
        if via:
            items.append(dict(name='Arama hattına geçiş', cmd=M.MAV_CMD_NAV_WAYPOINT, lat=via[0], lon=via[1], alt=cruise_rel))
        scan_plan = plan_scan(home, home_amsl, search_point, bearing_deg(*first, *search_point) if via else bearing_deg(*level_end, *search_point), args.world)
        if scan_plan is None:
            print('UYARI: tarama alanının arazi verisi yok, yangın bulunamazsa tarama yapılmadan dönülecek', flush=True)
        items += search_items(*home, *search_point, cruise_rel, level_end if plan['climb_first'] else None, *(scan_plan or ()))
    else:
        climb_at = home if plan['climb_first'] else None
        if plan['climb_first'] and not plan['safe']:
            sys.exit(f'Rotada arazi payı yetersiz ({plan["margin"]:.0f} m) ve tırmanış için güvenli alan yok. Kalkılmadı.')
        if plan['climb_first'] and not circle_clear(home, CIRCLE_RADIUS_M, home_amsl + TAKEOFF_REL_M, args.world):
            sys.exit(f'Tırmanış çemberinde {CIRCLE_RADIUS_M:.0f} m yarıçapta arazi payı yok. Kalkılmadı.')
        scan_plan = plan_scan(home, home_amsl, search_point, bearing_deg(*home, *search_point), args.world)
        if scan_plan is None:
            print('UYARI: tarama alanının arazi verisi yok, yangın bulunamazsa tarama yapılmadan dönülecek', flush=True)
        items = [takeoff_item(*home)] + search_items(*home, *search_point, cruise_rel, climb_at, *(scan_plan or ()))
    v.upload(items)
    print(f'Görev yüklendi: {len(items)} adım', flush=True)

    # PX4 keeps the mission index from the previous flight, which would start this one at its landing.
    end = time.time() + 15
    while time.time() < end:
        v.link.mav.mission_set_current_send(v.link.target_system, v.link.target_component, 0)
        v.update(1)
        if s['seq'] == 0:
            break
    else:
        sys.exit('Görev başa alınamadı')
    if args.key_start:
        # Arming in mission mode starts the takeoff at once, so arm in hold: the motor stays idle on the ground.
        if not set_auto_mode(v, AUTO_LOITER):
            sys.exit('AUTO.LOITER moduna geçilemedi. Arm edilmedi.')
        wait_enter(v, '>>> ARM için Enter tuşuna basın')
    else:
        v.command(M.MAV_CMD_DO_SET_MODE, 1, 4, AUTO_MISSION)
        v.update(2)
    # An interrupted water leg can leave a low throttle limit in the saved parameters, which the next flight would take as normal.
    throttle_limit = read_param(v, THROTTLE_LIMIT_PARAM)
    if throttle_limit is None:
        sys.exit(f'{THROTTLE_LIMIT_PARAM} okunamadı. Arm edilmedi.')
    if abs(throttle_limit - THROTTLE_LIMIT_EXPECTED) > 1e-3:
        sys.exit(f'{THROTTLE_LIMIT_PARAM}={throttle_limit:.3f}, beklenen {THROTTLE_LIMIT_EXPECTED}. Arm edilmedi. Parametre dosyasını kontrol edin.')
    print(f'{THROTTLE_LIMIT_PARAM}={throttle_limit:.3f}', flush=True)
    if args.scoop:
        # Su alma hız sınırları PX4 parametrelerinden (kullanıcı 2026-11-27); havada okumak telemetriyi bekletir, yerde okunur
        try:
            water_leg.HIZ_SINIRI = SA.hiz_siniri_px4(lambda name: read_param(v, name))
        except RuntimeError as error:
            sys.exit(f'{error}. Arm edilmedi.')
        lim = water_leg.HIZ_SINIRI
        print(f'Su alma hız sınırları (PX4): gaz hedefi {water_leg.SPEED_MIN_MS:.2f}-{lim.kontrol_ust_ms:.2f} m/s, '
              f'başarısız: {lim.iptal_alt_ms:.2f}-{lim.iptal_ust_ms:.2f} m/s ({lim.iptal_alt_ms * 3.6:.0f}-{lim.iptal_ust_ms * 3.6:.0f} km/h) '
              f'dışında toplam {lim.disarida_en_cok_s:.1f} s\'den uzun', flush=True)
    # The actual weight from the start: globalpilot-49's takeoff tests (2026-11-08) showed the takeoff at 800 kg without
    # the sag after the takeoff step (PX4 also scales the takeoff speeds: 38.6 m/s, liftoff in 305-317 m).
    from otonom.sensor import SimTankKutlesi
    tank = SimTankKutlesi(MODEL_NAME, EMPTY_MASS_KG)
    v.update(2)   # the tank mass topic arrives about once a second
    mass = tank.read()
    weight = set_weight(v, mass if mass is not None else WEIGHT_BASE_KG)
    if weight is None:
        sys.exit(f'{WEIGHT_PARAM} yazılamadı. Arm edilmedi.')
    print(f'{WEIGHT_PARAM}={weight:.0f} kg (kalkış), tank kütlesi {mass if mass is not None else "yok (WEIGHT_BASE)"}', flush=True)
    wheel = lower_gear_on_ground(v, args.world, MODEL_NAME)
    if wheel is None:
        sys.exit('İniş takımı açılamadı (SIM_GZ_SV_FUNC5 301 mi?). Arm edilmedi.')
    print(f'İniş takımı açık: tekerlek z {wheel:.3f}', flush=True)
    end = time.time() + 180
    while not s['armed'] and time.time() < end:
        v.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        v.update(3)
    if not s['armed']:
        sys.exit('Uçak arm edilemedi: 3 dakika içinde hazır olmadı')
    if args.key_start:
        print('Arm edildi, uçak yerde bekliyor', flush=True)
        wait_enter(v, '>>> KALKIŞ için Enter tuşuna basın')
        if not s['armed']:
            sys.exit('Uçak beklerken disarm oldu. Kalkılmadı.')
        if not set_auto_mode(v, AUTO_MISSION):
            sys.exit('AUTO.MISSION moduna geçilemedi. Kalkılmadı.')
    print('Arm edildi, görev başlıyor', flush=True)

    t0 = time.time()
    stamp = lambda: f'[{time.time() - t0:5.0f} s]'
    estimate = Estimate()
    target = None
    aimed = None
    corrections = [1800.0, 900.0]
    run_bearing = None
    released = None
    release_end = None
    points = []
    effect_time = None
    hit = None
    last_print = 0.0
    last_look = 0.0
    searching = True
    current = -1
    airborne = False
    gear_up_sent = False
    last_weight = 0.0

    while time.time() - t0 < args.timeout:
        v.update(0.2)
        if s['seq'] != current and 0 <= s['seq'] < len(items):
            current = s['seq']
            print(f'{stamp()} {current + 1}/{len(items)}: {items[current]["name"]}', flush=True)
        airborne = airborne or s['rel'] > 20
        if not gear_up_sent and s['armed'] and s['rel'] > GEAR_UP_REL_M:
            nan = float('nan')
            for _ in range(3):   # COMMAND_LONG is not acknowledged here: repeat so one lost packet does not leave it down
                v.command(MAV_CMD_DO_SET_ACTUATOR, 1.0, nan, nan, nan, nan, nan, 0)
            gear_up_sent = True
            print(f'{stamp()} İniş takımı toplanıyor: göreli {s["rel"]:.0f} m', flush=True)
        if s['armed'] and time.time() - last_weight > 2.0:
            last_weight = time.time()
            mass = tank.read()
            if mass is not None and abs(mass - weight) >= WEIGHT_STEP_KG:
                weight = mass
                for _ in range(2):   # not acknowledged in flight: repeat so one lost packet does not leave the old weight
                    v.link.mav.param_set_send(v.link.target_system, v.link.target_component, WEIGHT_PARAM.encode(), mass, M.MAV_PARAM_TYPE_REAL32)
                print(f'{stamp()} Ağırlık PX4\'e yazıldı: {WEIGHT_PARAM} {mass:.0f} kg', flush=True)

        # Look for the fire until the drop has been made.
        # The scooping mission holds its straight climb and level leg: the search starts after them.
        if searching and released is None and s['rel'] > 100 and (not args.scoop or target is not None or s['seq'] >= SEARCH_FROM_SEQ) and time.time() - last_look > 0.4:
            last_look = time.time()
            frame = camera.frame()
            found = find_fire(frame) if frame is not None else None
            if found:
                ray = pixel_to_ned(found[0], found[1], frame.shape[1], frame.shape[0], s['roll'], s['pitch'], s['yaw'])
                hit_ground = ray_to_ground(s['lat'], s['lon'], s['alt'], ray, args.world)
                if hit_ground:
                    estimate.add(*hit_ground)
                    if len(estimate.obs) == 1:
                        print(f'{stamp()} ALEV BULUNDU: kamerada ({found[0]:.0f}, {found[1]:.0f}), {hit_ground[3] / 1000:.1f} km ileride', flush=True)

        if searching and estimate.obs:
            position = estimate.position()
            rng = distance_m(s['lat'], s['lon'], position[0], position[1])
            if target is None and len(estimate.obs) >= MIN_OBSERVATIONS and rng < COMMIT_RANGE_M:
                target = position
                aimed = target[:2]
                run_bearing = bearing_deg(s['lat'], s['lon'], target[0], target[1])
                run_rel = target[2] + DROP_AGL_M - home_amsl
                err = distance_m(target[0], target[1], args.fire_lat, args.fire_lon)
                print(f'{stamp()} KONUM HESAPLANDI: {target[0]:.5f}, {target[1]:.5f} ({len(estimate.obs)} gözlem). Gerçeğe uzaklık {err:.0f} m. '
                      + ('Su almaya gidiliyor.' if args.scoop else 'Yangına yöneliniyor.'), flush=True)
                if args.scoop:
                    # Straight to the nearest water. The water leg descends at once, so it takes over only where that clears the terrain.
                    water_map = water_leg.WaterMap()
                    v.update(0.2)
                    route = water_map.nearest_run((s['lat'], s['lon']))
                    entry = route[0] if route else None
                    transit = plan_search(s['lat'], s['lon'], s['alt'], entry, args.world) if entry else None
                    leg = dict(ok=False, reason='su rotası veya arazi verisi yok')
                    gecis = None
                    if transit is not None:
                        # Planlayıcının otonom su geçişi modu (otonom/planlayici/su_gecisi.py): yerden her zaman en az
                        # 100 m; alçalış payı koruyabildiği ilk noktada; su küçükse irtifa karada çemberde kaybedilir.
                        transit_rel = max(cruise_rel, transit['cruise'] - home_amsl)
                        entry_amsl = water_map.height + water_leg.APPROACH_ABOVE_WATER_M
                        terrain = SG.OnbellekliArazi(lambda la, lo: float(ground_height(data_root_dir(), args.world, la, lo)))
                        gecis = SG.plan_water_transit((s['lat'], s['lon']), home_amsl + transit_rel, entry, route[2], entry_amsl,
                                                      terrain, water_map.is_water, SU_GECIS_AYAR)
                        if not gecis.ok:
                            leg = dict(ok=False, reason=f'su geçişi planlanamadı: {gecis.reason}')
                    if gecis is not None and gecis.ok:
                        print(f'{stamp()} Doğrudan suya ({gecis.mode}): giriş {entry[0]:.4f},{entry[1]:.4f}, '
                              f'{distance_m(s["lat"], s["lon"], entry[0], entry[1]) / 1000:.1f} km; '
                              + (f'alçalış {gecis.descent_start_m:.0f} m sonra' if gecis.descent_start_m is not None else '')
                              + (f', çemberde {gecis.floor_amsl - home_amsl:.0f} m\'ye' if gecis.mode != 'düz alçalış' else ''), flush=True)
                        transit_items = R.to_mission(gecis.items, home_amsl, M)
                        for it in transit_items:
                            if 'radius' in it:  # su bacağı devralacak: çember irtifaya inince bitsin, teğet bekleme yok
                                it['param1'] = 0
                                it.pop('param4', None)
                        upload_in_flight(v, transit_items + return_items(*home, 300.0, entry, route[2], actual_rel=entry_amsl - home_amsl))
                        v.update(2)
                        while s['seq'] < len(transit_items) and s['armed'] and time.time() - t0 < args.timeout:
                            v.update(0.2)
                        print(f'{stamp()} Su bölümü başlıyor, irtifa {s["rel"]:.0f} m', flush=True)
                        try:
                            if os.environ.get('GASI_SU_PX4') == '1':
                                # Su alma PX4 içinde (kullanıcı kararı 2026-11-11; otonom/planlayici/su_alma.py). Varsayılan değil.
                                leg = water_leg.run_water_leg_px4(v, (*home, home_amsl), MODEL_NAME, args.timeout - (time.time() - t0))
                            else:
                                alignment = dict(record=args.offboard_hizalanma) if args.offboard_hizalanma else None
                                leg = water_leg.run_water_leg(v, (*home, home_amsl), MODEL_NAME, args.timeout - (time.time() - t0),
                                                              offboard_alignment=alignment)
                        except Exception as error:  # the aircraft is in the air: a fault here must still lead to a return
                            leg = dict(ok=False, reason=f'su bölümü hatası: {error!r}')
                    print(f'{stamp()} Su bölümü bitti: {leg}', flush=True)
                    # Do not attack with no water on board. A return is only possible from the air.
                    if not leg.get('ok') and s['alt'] < WATER_LEG_RETURN_MIN_AMSL_M:
                        sys.exit(f'Su bölümü başarısız, uçak havalanamadı: {leg.get("reason")}')
                    why = f'su alınamadı ({leg.get("reason")})'
                    strike = plan_search(s['lat'], s['lon'], s['alt'], target[:2], args.world) if leg.get('ok') else None
                    if leg.get('ok') and strike is None:
                        why = 'arazi verisi bu rotada yok'
                    elif strike is not None and strike['climb_first'] and not strike['safe']:
                        why = f'arazi payı {strike["margin"]:.0f} m ve tırmanış için güvenli alan yok'
                        strike = None
                    elif strike is not None and strike['climb_first'] and not circle_clear((s['lat'], s['lon']), CIRCLE_RADIUS_M, s['alt'], args.world):
                        why = f'tırmanış çemberinde {CIRCLE_RADIUS_M:.0f} m yarıçapta arazi payı yok'
                        strike = None
                    if strike is None:
                        searching = False
                        target = None
                        items = return_items(*home, max(300.0, s['rel']), (s['lat'], s['lon']), math.degrees(s.get('yaw', 0.0)) % 360.0,
                                             actual_rel=s['rel'], at_aircraft=True)
                        print(f'{stamp()} Yangına gidilmiyor: {why}. Piste dönülüyor', flush=True)
                    else:
                        # Exit point depends on the water, so the run is planned from where the aircraft is now.
                        cruise_rel = strike['cruise'] - home_amsl
                        attack = plan_attack(target, bearing_deg(s['lat'], s['lon'], target[0], target[1]), run_rel, cruise_rel)
                        if isinstance(attack, str):
                            searching = False
                            target = None
                            items = return_items(*home, max(300.0, s['rel']), (s['lat'], s['lon']), math.degrees(s.get('yaw', 0.0)) % 360.0,
                                                 actual_rel=s['rel'], at_aircraft=True)
                            print(f'{stamp()} Yangına gidilmiyor: su bırakma için yaklaşması, çıkışı ve piste dönüşü güvenli yön yok '
                                  f'({attack}). Piste dönülüyor', flush=True)
                        else:
                            run_bearing, exit_plan = attack
                            items = strike_items(target, run_bearing, run_rel, cruise_rel, exit_plan, (s['lat'], s['lon']) if strike['climb_first'] else None)
                            print(f'{stamp()} Su alındı, yangına gidiliyor. Yön {run_bearing:.0f}°, seyir {strike["cruise"]:.0f} m, '
                                  f'bırakma sonrası düz {exit_plan.exit_m:.0f} m' + ('. Önce çember atarak tırmanılacak' if strike['climb_first'] else ''), flush=True)
                            print(f'{stamp()} {exit_plan.ret["text"]}', flush=True)
                else:
                    attack = plan_attack(target, run_bearing, run_rel, cruise_rel)
                    if isinstance(attack, str):
                        searching = False
                        target = None
                        items = return_items(*home, max(300.0, s['rel']), (s['lat'], s['lon']), math.degrees(s.get('yaw', 0.0)) % 360.0,
                                             actual_rel=s['rel'], at_aircraft=True)
                        print(f'{stamp()} Yangına gidilmiyor: su bırakma için yaklaşması, çıkışı ve piste dönüşü güvenli yön yok '
                              f'({attack}). Piste dönülüyor', flush=True)
                    else:
                        run_bearing, exit_plan = attack
                        items = attack_mission(target, run_bearing, run_rel, cruise_rel, exit_plan)
                        print(f'{stamp()} Yön {run_bearing:.0f}°, bırakma sonrası düz {exit_plan.exit_m:.0f} m. {exit_plan.ret["text"]}', flush=True)
                upload_in_flight(v, items)
                current = -1
            elif target is not None and released is None:
                # The position improves with every closer view: the release point follows it all the
                # way in, and the track is updated at set ranges while there is room left to correct.
                shift = distance_m(position[0], position[1], target[0], target[1])
                target = (position[0], position[1], target[2])
                if corrections and rng < corrections[0]:
                    corrections.pop(0)
                    if shift > 5 or distance_m(position[0], position[1], *aimed) > 15:
                        # Düzeltilen hedefe yeni yön, olmazsa uçulan yön; ikisinde de çıkış ve piste dönüş planlayıcıdan.
                        # Hiçbiri tutmazsa yüklü görev (denetlenmiş çıkış ve dönüşüyle) kalır; bırakma anı yine son hesaba göre.
                        err = distance_m(target[0], target[1], args.fire_lat, args.fire_lon)
                        new_bearing = bearing_deg(s['lat'], s['lon'], target[0], target[1])
                        attack = plan_attack(target, new_bearing, run_rel, cruise_rel, bearings=(new_bearing, run_bearing))
                        if isinstance(attack, str):
                            print(f'{stamp()} Rota güncellenmedi: düzeltilen hatta güvenli çıkış ya da dönüş yok ({attack}). '
                                  f'Hesap hatası {err:.0f} m', flush=True)
                        else:
                            aimed = target[:2]
                            run_bearing, exit_plan = attack
                            items = attack_mission(target, run_bearing, run_rel, cruise_rel, exit_plan)
                            v.upload(items)
                            v.link.mav.mission_set_current_send(v.link.target_system, v.link.target_component, 0)
                            current = -1
                            print(f'{stamp()} Rota güncellendi: hedefe {rng:.0f} m, hesap hatası {err:.0f} m. Yön {run_bearing:.0f}°, '
                                  f'bırakma sonrası düz {exit_plan.exit_m:.0f} m. {exit_plan.ret["text"]}', flush=True)

        if target is not None:
            along_m, across_m = track_errors(s['lat'], s['lon'], target, run_bearing)
            half_curtain = s['airspeed'] * RELEASE_S / 2
            if released is None and along_m >= -half_curtain:
                released = time.time()
                err = distance_m(target[0], target[1], args.fire_lat, args.fire_lon)
                print(f'{stamp()} SU BIRAKILIYOR: yerden ~{s["alt"] - target[2]:.0f} m, hat dışı {across_m:.0f} m. Son hesap hatası {err:.0f} m.', flush=True)
                emitter(WATER_CMD, 'emitting: {data: true}')
                # The tank empties in the physics too (WaterPhysics water/release, kg/s), a little faster than the curtain
                emitter(WATER_RELEASE, f'data: {WATER_RELEASE_KGS}', 'gz.msgs.Double')
            if released is not None and release_end is None:
                points.append((s['lat'], s['lon']))
                if time.time() - released >= RELEASE_S:
                    release_end = time.time()
                    emitter(WATER_CMD, 'emitting: {data: false}')
                    emitter(WATER_RELEASE, 'data: 0', 'gz.msgs.Double')
                    mid = points[len(points) // 2]
                    miss_along, miss_across = track_errors(args.fire_lat, args.fire_lon, mid, run_bearing)
                    hit = abs(miss_along) <= HIT_ALONG_M and abs(miss_across) <= HIT_ACROSS_M
                    print(f'{stamp()} Su bitti. Perde merkezi yangından: hat boyunca {miss_along:.0f} m, yanal {miss_across:.0f} m.', flush=True)
                    effect_time = time.time() + 4
        if effect_time and time.time() >= effect_time:
            effect_time = None
            if hit:
                emitter(FIRE_CMD.format('flames'), 'emitting: {data: false}')
                emitter(FIRE_CMD.format('dark_smoke'), 'emitting: {data: false}')
                emitter(FIRE_CMD.format('smoke_column'), 'rate: {data: 2}')
                print(f'{stamp()} İSABET: alev söndü, koyu duman kesildi.', flush=True)
            else:
                print(f'{stamp()} ISKA: yangın yanmaya devam ediyor.', flush=True)

        if time.time() - last_print > 20:
            last_print = time.time()
            where = f'hesaplanan hedefe {distance_m(s["lat"], s["lon"], *estimate.position()[:2]) / 1000:4.1f} km, {len(estimate.obs)} gözlem' if estimate.obs else 'alev aranıyor'
            print(f'{stamp()} yükseklik {s["rel"]:5.0f} m, hız {s["airspeed"]:4.1f} m/s, {where}', flush=True)
        if airborne and not s['armed']:
            result = 'İSABET' if hit else ('ISKA' if hit is False else 'su bırakılmadı')
            print(f'{stamp()} İniş tamamlandı. Sonuç: {result}.', flush=True)
            restored = set_weight(v, WEIGHT_BASE_KG)
            print(f'{WEIGHT_PARAM} geri alındı: {restored}' if restored is not None else f'UYARI: {WEIGHT_PARAM} geri alınamadı', flush=True)
            stop_along, stop_across = track_errors(s['lat'], s['lon'], home, RUNWAY_HEADING_DEG)
            print(f'{stamp()} Durduğu nokta kalkış noktasına {distance_m(*home, s["lat"], s["lon"]):.0f} m: '
                  f'pist yönünde {stop_along:.0f} m, yana {stop_across:.0f} m (pist yönü {RUNWAY_HEADING_DEG:.0f}° varsayımı).', flush=True)
            return
    sys.exit(f'Süre doldu ({args.timeout:.0f} s), görev bitmedi')


if __name__ == '__main__':
    main()
