#!/usr/bin/env python3
"""GASI su bölümü: havadan suya alçalış, hız aralığında su alma penceresi, tam gazla hızlanıp sudan ayrılma.

Geçiş NAV_LAND değil, irtifası su yüzeyinden hesaplanan normal görev adımıdır: PX4 iniş moduna girmez, motoru kesmez ve
geçiş sonunda tırmanışa devam edebilir. Su yüksekliği varsayılmaz, çalışan dünyadaki su modeli ve mesh'inden okunur.

Pencere hız aralığa girince başlar. Pencerede hız FW_THR_MAX ile tutulur (suda TECS gazı üst sınıra dayar); pencere
bitince ya da herhangi bir hatada özgün değer geri yazılır ve doğrulanır. Su alma henüz ölçüm olarak vardır: kütle değişmez.

Birleşik görevden:   run_water_leg(vehicle, home, model)   (Vehicle: fire_attack.py, uçak havada)
Tek başına deneme:   GZ_PARTITION=<bölüm> python3 water_leg.py --url udpin:0.0.0.0:14541 --model gasi_ffa_1 --sysid 2
"""
import argparse
import math
import os
import re
import subprocess
import sys
import threading
import time

import numpy as np
from gz.math7 import Angle, SphericalCoordinates, Vector3d
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
from PIL import Image
from pymavlink import mavutil

M = mavutil.mavlink
WORLD = 'antalya'
WATER_MODEL = 'geo_terrain_water'
WATER_DIR = '~/.local/share/gz-geo-terrain/antalya/water'
HULL_COLUMNS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'gasi_ffa', 'hull_columns.txt')
GROUND_ROLL_HEADING_DEG = 5.0
LAND_REL_M = 0.0                  # iniş noktasında pist, eve göre; tek başına deneme ayarlar

# Kabul şartı: su alma penceresinin süresi ve o süredeki Gazebo hızı. Sudan ayrılmanın süre şartı yok, ayrı raporlanır.
# Hız sınırları PX4 parametrelerinden (kullanıcı 2026-11-27; otonom/planlayici/su_alma.py HizSiniri): gaz kontrol bandını
# (SPEED_MIN_MS .. FW_SC_VMAX) hedefler, pencere ve dolum orada; su alma ancak iptal bandı (FW_SC_A_VMIN .. FW_SC_A_VMAX)
# dışında toplam FW_SC_OOB_T'den uzun kalınırsa başarısız. Önceden sabit 80-100 km/h ve bant dışı süre sıfırdı.
SCOOP_MIN_S, SCOOP_MAX_S = 8.0, 12.0
SPEED_MIN_MS = 80 / 3.6           # kontrol bandı alt sınırı (PX4 yolunda görev maddesinin param4'ü), yuvarlatmadan
HIZ_SINIRI = None                 # SA.HizSiniri; fire_attack yerde PX4'ten okur, yoksa su bölümü başında okunur
SCOOP_TARGET_S = 10.0
SPEED_TARGET_MS = 25.0
THROTTLE_PARAM = b'FW_THR_MAX'
GEAR_UP_MIN_WHEEL_Z = -0.45       # sol tekerlek modele göre: kapalı ~-0,31, açık -0,69
GEAR_WAIT_S = 5.0
THROTTLE_MIN, THROTTLE_MAX = 0.27, 1.0  # 5002 N (itki gazın karesi): alt sınır 3000 N'deki yavaşlamayı korur
THROTTLE_BASE = 0.65              # 25 m/s için; kütleli görevde giriş boştur (uçuş k9, k10: yavaşlama 631-633 m). Dolu girişte ölçülmedi, görevde böyle bir giriş yok
# Su alırken durgun su uçak hızına çıkarılır: momentum direnci ṁ·v (WaterPhysics, 5d incelemesi). Taban gaza eklenir:
# gaz = sqrt(THROTTLE_BASE² + ṁ·v / itki_en_çok). itki gazın karesiyle orantılı, en çok 5002 N; ṁ = water_tank.txt fill_rate.
# Uçuş n5 (dirençli, eski taban 0,65): pencerede hız 22,6 m/s'ye düştü (sınır 22,22).
SCOOP_FILL_RATE_KGS = 40.0
THRUST_MAX_N = 5002.0
THROTTLE_GAIN = 0.06              # (m/s) başına; 0,715 gazda itki eğimi yüksek olduğu için azaltıldı
THROTTLE_EXPECTED_ORIGINAL = 1.0  # farklıysa önceki uçuştan parameters.bson'a kalmış ara değer olabilir
THROTTLE_MIN_CHANGE = 0.01        # daha küçük farklar gönderilmez: her değişiklik PX4'te parametre kaydı tetikler
THROTTLE_LOOKAHEAD_S = 2.5        # motor ve TECS gecikmesi: öngörülen hıza göre kontrol, ölçülen alt aşım 20,8 m/s
THROTTLE_PERIOD_S = 0.5

APPROACH_LENGTH_M = 5000.0        # ölçülen alçalma ~2 m/s: 150 m için 4 km, geri kalanı düz gelmek için
DECELERATION_ALLOWANCE_M = 1440.0 # boş girişte ölçülen 1259 m ve 440 m kabul yarıçapı: adım 3 pencereden önce gelmesin
SCOOP_LENGTH_M = 400.0            # 27,8 m/s ile 12 s = 333 m ve pay
RUN_LENGTH_M = DECELERATION_ALLOWANCE_M + SCOOP_LENGTH_M
CLIMB_LENGTH_M = 4000.0           # ölçülen en az 2,8 km ayrılma mesafesi ve pay
CORRIDOR_HALF_WIDTH_M = 150.0
SAMPLE_STEP_M = 100.0
SEARCH_STEP_M = 500.0
SEARCH_BEARING_STEP_DEG = 15
# Kullanıcı kararı (2026-11-10): uçak yerden en az 100 m yukarıda kalır; irtifa karanın üstünde kaybedilir ve suya 100 m'de
# gelinir. Hizalanmanın gerektirdiği su üstü yol 150 m'den ~4,75 km iken 100 m'den ~3,8-3,9 km (800/1200 kg).
APPROACH_ABOVE_WATER_M = 100.0
CLIMB_ABOVE_WATER_M = 150.0
# Geçişte hedef, su yüzeyinin altındadır: PX4 irtifa hatası (ölçülen ~2 m) yüzünden uçak yüzeyde hedeflenince suya değmiyor.
# Uçak kaldırma ve kayma basıncıyla yüzeyde kalır; ölçülen derinlik bu hedeften bağımsız ~0,3 m.
RUN_TARGET_BELOW_SURFACE_M = 3.0
CRUISE_REL_M = 250.0
LEFT_WATER_ABOVE_M = 3.0          # bu yüksekliğin altındaki ayrılma sekmedir
RELEASE_ABOVE_WATER_M = 30.0      # bu yüksekliği geçince tırmanış başarılı sayılır
SPEED_WINDOW_S = 0.5              # hız, bu kadar simülasyon saniyesindeki konum farkından


def gz_pose(name):
    """Tek seferlik okuma; döngüde değil, yalnızca hazırlıkta kullanılır."""
    text = re.sub(r'\x1b\[[0-9;]*m', '', subprocess.run(['gz', 'model', '-m', name, '-p'], capture_output=True, text=True, timeout=20).stdout)
    found = re.search(r'Pose \[ XYZ.*?\n\s*\[([^\]]+)\]\s*\n\s*\[([^\]]+)\]', text)
    return (list(map(float, found.group(1).split())), list(map(float, found.group(2).split()))) if found else None


class Telemetry:
    """Gazebo konu aboneliği: model konumu ve duruşu, tekerlek konumu, simülasyon saati. Yeni süreç açmaz."""

    def __init__(self, model):
        self.model = model
        self.lock = threading.Lock()
        self.position = None
        self.rotation = None
        self.sim_time = None
        self.wheel_z = float('nan')
        self.node = Node()
        self.node.subscribe(Pose_V, f'/world/{WORLD}/pose/info', self.on_pose)
        self.node.subscribe(Pose_V, f'/world/{WORLD}/dynamic_pose/info', self.on_links)

    def on_pose(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        for pose in msg.pose:
            if pose.name == self.model:
                with self.lock:
                    self.position = (pose.position.x, pose.position.y, pose.position.z)
                    q = pose.orientation
                    self.rotation = (q.w, q.x, q.y, q.z)
                    self.sim_time = stamp
                return

    def on_links(self, msg):
        for pose in msg.pose:
            if pose.name == 'left_wheel':
                with self.lock:
                    self.wheel_z = pose.position.z
                return

    def snapshot(self):
        with self.lock:
            return self.sim_time, self.position, self.rotation, self.wheel_z

    def wait(self, timeout=20.0):
        end = time.time() + timeout
        while time.time() < end:
            if self.snapshot()[1] is not None:
                return True
            time.sleep(0.1)
        return False


def body_axes(rotation):
    """Dünya çerçevesinde gövde ekseni (ileri) ve sağ-yukarı için yatış; roll, pitch (burun aşağı +), yaw."""
    w, x, y, z = rotation
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def along(lat, lon, bearing_deg, distance, right=0.0):
    b = math.radians(bearing_deg)
    north = distance * math.cos(b) - right * math.sin(b)
    east = distance * math.sin(b) + right * math.cos(b)
    return lat + north / 111320.0, lon + east / (111320.0 * math.cos(math.radians(lat)))


def distance_m(a, b):
    return math.hypot((b[0] - a[0]) * 111320.0, (b[1] - a[1]) * 111320.0 * math.cos(math.radians(a[0])))


class WaterMap:
    """Su maskesi: kıyı ve bölge sınırı dahil. Kaynak kamera değil, dünyayı üreten harita verisi."""

    def __init__(self):
        root = os.path.expanduser(WATER_DIR)
        Image.MAX_IMAGE_PIXELS = None
        self.mask = np.asarray(Image.open(os.path.join(root, 'materials/textures/water_mask.png'))) >= 128
        cfg = dict(line.split() for line in open(os.path.join(root, 'physics.cfg')) if line.strip())
        self.west, self.south = float(cfg['west']), float(cfg['south'])
        self.east, self.north = float(cfg['east']), float(cfg['north'])
        self.height = float(cfg['height'])

    def is_water(self, lat, lon):
        if not (self.west <= lon < self.east and self.south <= lat < self.north):
            return False
        rows, cols = self.mask.shape
        row = min(int((self.north - lat) / (self.north - self.south) * rows), rows - 1)
        col = min(int((lon - self.west) / (self.east - self.west) * cols), cols - 1)
        return bool(self.mask[row, col])

    def corridor_clear(self, start, course, length):
        """Başlangıçtan course yönünde length boyunca, her iki yanda CORRIDOR_HALF_WIDTH_M dahil tamamı su mu?"""
        steps = int(length / SAMPLE_STEP_M) + 1
        for k in range(steps + 1):
            for side in (-CORRIDOR_HALF_WIDTH_M, 0.0, CORRIDOR_HALF_WIDTH_M):
                if not self.is_water(*along(*start, course, k * SAMPLE_STEP_M, side)):
                    return False
        return True

    def nearest_run(self, position):
        """Uçağa en yakın geçiş: yaklaşma, geçiş ve tırmanış hattı suda kalan düz rota. (yaklaşma başı, geçiş başı, yön)"""
        total = APPROACH_LENGTH_M + RUN_LENGTH_M + CLIMB_LENGTH_M
        step_lat = SEARCH_STEP_M / 111320.0
        step_lon = SEARCH_STEP_M / (111320.0 * math.cos(math.radians(position[0])))
        lats = np.arange(self.south, self.north, step_lat)
        lons = np.arange(self.west, self.east, step_lon)
        candidates = [(lat, lon) for lat in lats for lon in lons if self.is_water(lat, lon)]
        candidates.sort(key=lambda c: distance_m(position, c))
        for start in candidates:
            # uçağa dönük yönleri önce dene: yaklaşma uçağın geldiği taraftan başlasın
            toward = math.degrees(math.atan2((position[1] - start[1]) * math.cos(math.radians(start[0])), position[0] - start[0]))
            for offset in sorted(range(0, 360, SEARCH_BEARING_STEP_DEG), key=lambda a: abs((a + 180) % 360 - 180)):
                course = (toward + 180.0 + offset) % 360
                if self.corridor_clear(start, course, total):
                    run_start = along(*start, course, APPROACH_LENGTH_M)
                    return start, run_start, course
        return None


class WaterSurface:
    """Çalışan dünyadaki su yüzeyi. home: dünyanın başlangıç noktası (enlem, boylam)."""

    def __init__(self, home):
        pose = gz_pose(WATER_MODEL)
        if pose is None:
            raise RuntimeError('su modeli çalışan dünyada yok')
        self.offset = pose[0][2]
        with open(os.path.join(os.path.expanduser(WATER_DIR), 'meshes', 'water.obj')) as f:
            self.vertices = np.array([list(map(float, line.split()[1:])) for line in f if line.startswith('v ')])
        with open(HULL_COLUMNS) as f:
            self.hull_bottom = min(float(line.split()[2]) for line in f.read().split('\n')[2:] if line.strip())
        self.sc = SphericalCoordinates(SphericalCoordinates.EARTH_WGS84)
        self.sc.set_latitude_reference(Angle(math.radians(home[0])))
        self.sc.set_longitude_reference(Angle(math.radians(home[1])))
        self.sc.set_elevation_reference(-self.offset)  # eklenti modeli world elevation kadar indirir
        self.sc.set_heading_offset(Angle(0.0))

    def z_at(self, x, y):
        d2 = (self.vertices[:, 0] - x) ** 2 + (self.vertices[:, 1] - y) ** 2
        near = np.argsort(d2)[:6]
        if d2[near[0]] > 300.0 ** 2:
            return None
        a = np.column_stack((self.vertices[near, 0], self.vertices[near, 1], np.ones(6)))
        coef = np.linalg.lstsq(a, self.vertices[near, 2], rcond=None)[0]
        return float(coef[0] * x + coef[1] * y + coef[2]) + self.offset

    def height_of(self, x, y, z):
        return self.sc.position_transform(Vector3d(x, y, z), SphericalCoordinates.LOCAL2, SphericalCoordinates.SPHERICAL).z()

    def local_of(self, lat, lon):
        p = self.sc.position_transform(Vector3d(math.radians(lat), math.radians(lon), 0.0),
                                       SphericalCoordinates.SPHERICAL, SphericalCoordinates.LOCAL2)
        return p.x(), p.y()

    def height_at(self, lat, lon):
        """Enlem ve boylamda su yüksekliği, deniz seviyesine göre. Su yoksa None."""
        x, y = self.local_of(lat, lon)
        z = self.z_at(x, y)
        return None if z is None else self.height_of(x, y, z)

    def clearance(self, pose):
        """Gövde altının su üstündeki yüksekliği; su yoksa None."""
        z = self.z_at(pose[0], pose[1])
        return None if z is None else pose[2] + self.hull_bottom - z


def plan(surface, water_map, vehicle_lat, vehicle_lon, vehicle_rel, vehicle_alt):
    """Görev adımları, göreli irtifalarla. Rota uçağın konumuna en yakın uygun sudan hesaplanır."""
    # PX4'te evin yüksekliği: aynı mesajdan gelen iki değer olduğu için tırmanışta zaman kayması olmaz
    home_height = vehicle_alt - vehicle_rel
    route = water_map.nearest_run((vehicle_lat, vehicle_lon))
    if route is None:
        raise RuntimeError('uygun su koridoru yok')
    entry, run_start, course = route
    water = surface.height_at(*run_start)
    if water is None:
        raise RuntimeError('geçiş noktasında su yüzeyi yok')
    run_end = along(*run_start, course, RUN_LENGTH_M)
    climb_end = along(*run_end, course, CLIMB_LENGTH_M)
    # Taban bağlantısı gövde altından -hull_bottom yukarıda
    run_rel = water - surface.hull_bottom - RUN_TARGET_BELOW_SURFACE_M - home_height
    wp = M.MAV_CMD_NAV_WAYPOINT
    items = [
        dict(name='Yaklaşma', cmd=wp, lat=entry[0], lon=entry[1], alt=water + APPROACH_ABOVE_WATER_M - home_height),
        dict(name='Geçiş başı', cmd=wp, lat=run_start[0], lon=run_start[1], alt=run_rel),
        dict(name='Geçiş sonu', cmd=wp, lat=run_end[0], lon=run_end[1], alt=run_rel),
        dict(name='Tırmanış', cmd=wp, lat=climb_end[0], lon=climb_end[1], alt=water + CLIMB_ABOVE_WATER_M - home_height),
    ]
    info = dict(water_height=water, home_height=home_height, run_rel=run_rel, run_course=course, entry=entry, run_start=run_start, end=run_end, climb_end=climb_end)
    return items, info


def run_offboard_alignment(vehicle, items, info, telemetry, surface, model, options, log):
    """Görev Yaklaşma'yı geçince offboard'a girer, suya değip 2 s sonra 'Geçiş sonu'ndan göreve devreder.
    Bu sürede bağlantıyı yalnız otonom.Baglanti okur; sonra vehicle.update kaldığı yerden sürer."""
    from otonom import rehber
    from otonom.baglanti import Baglanti
    from otonom.hizalanma import Hizalanma, Plan
    from otonom.kayit import Kayit
    from otonom.sensor import SimTankKutlesi, SimYukseklik
    names = [item['name'] for item in items]
    plan_ = Plan(hat=rehber.Hat(info['run_start'][0], info['run_start'][1], info['run_course']),
                 approach_seq=names.index('Yaklaşma'), run_start_seq=names.index('Geçiş başı'),
                 handover_seq=names.index('Geçiş sonu'), capture_height_m=APPROACH_ABOVE_WATER_M,
                 handover_throttle_param=THROTTLE_PARAM.decode(), handover_throttle=THROTTLE_BASE)
    record = Kayit(options['record'])
    record.karar('plan', hat=[info['run_start'][0], info['run_start'][1], info['run_course']], su=info['water_height'])
    link = Baglanti(vehicle.link, log).start()
    try:
        # suya değiş pistteki iniş flare'i (yasa 3) ile aynı: değerler PX4'ten
        values = {name: link.read_param(name) for name in rehber.REHBER_PX4_PARAMS}
        missing = [name for name, value in values.items() if value is None]
        if missing:
            raise RuntimeError(f'PX4 parametreleri okunamadı: {", ".join(missing)}')
        ayar = rehber.ayar_from_px4(values)
        record.karar('rehber parametreleri', **values)
        log('rehber: ' + ', '.join(f'{k} {v:g}' for k, v in values.items()))
        machine = Hizalanma(link, SimYukseklik(telemetry, surface), SimTankKutlesi(model, options.get('empty_mass_kg', 800.0)),
                            plan_, record, ayar=ayar, log=log, default_mass_kg=options.get('default_mass_kg', 1200.0))
        outcome = machine.run()
    finally:
        link.stop()
        record.karar('bitti')
        record.close()
    vehicle.update(0.5)
    log(f'offboard hizalanma: {outcome}')
    return outcome


# İniş takımı DO_SET_ACTUATOR ile (kullanıcı kararı 2026-11-08): takım servosu 'Peripheral via Actuator Set 1' (301)
# olmalı (SIM_GZ_SV_FUNC5 301). PX4'ün kendi takım işlevi (400) kalkış adımı CLIMBOUT'tan önce biterse takımı toplamıyordu
# (k5, n6). Değerler PX4 takım işleviyle aynı: +1 kapalı, -1 açık. İşlev 400 iken bu adımlar etkisizdir (gösteri değişmez).
GEAR_FUNCTION_ACTUATOR_SET1 = 301
MAV_CMD_DO_SET_ACTUATOR = 187      # MAVLink ortak komutu; kurulu pymavlink lehçesinde adı yok
GEAR_UP_VALUE, GEAR_DOWN_VALUE = 1.0, -1.0
GEAR_DOWN_MAX_WHEEL_Z = -0.6        # sol tekerlek: açık -0,69, kapalı ~-0,31
# Takım adımları görevlere eklenir (airframe 4022 SIM_GZ_SV_FUNC5 301, herkes için; kullanıcı kararı 2026-11-08).
# Yükleyici görev çerçevesini (item['frame']) göndermelidir: fire_attack.Vehicle, StandaloneVehicle, otonom.Baglanti gönderir.
USE_GEAR_ACTUATOR = True


def gear_item(up):
    """Görev adımı: takımı topla (up) ya da aç. Yalnız Actuator Set 1 (index 0); diğer kanallar NaN, değişmez."""
    nan = float('nan')
    # Konumsuz komut: görev çerçevesi. x, y, z = param5, 6, 7; param7 = 0: Actuator Set 1 (5 ve 6. kanal 0 olur, kullanılmıyor)
    return dict(name='Takımı topla' if up else 'Takımı aç', cmd=MAV_CMD_DO_SET_ACTUATOR, lat=0.0, lon=0.0, alt=0.0,
                param1=GEAR_UP_VALUE if up else GEAR_DOWN_VALUE, radius=nan, param4=nan, frame=M.MAV_FRAME_MISSION)


def gear_command(vehicle, up):
    """Anında takım komutu (COMMAND_LONG DO_SET_ACTUATOR, param7 = 0: Actuator Set 1)."""
    nan = float('nan')
    vehicle.command(MAV_CMD_DO_SET_ACTUATOR, GEAR_UP_VALUE if up else GEAR_DOWN_VALUE, nan, nan, nan, nan, nan, 0)


def return_items(home, cruise_rel=CRUISE_REL_M, gear=None):
    """PX4 görevi iniş adımı ister: pist dönüşü. Birleşik görev kendi görevini yükler.
    gear (varsayılan USE_GEAR_ACTUATOR): takım taban bacağından sonra DO_SET_ACTUATOR ile açılır."""
    if gear is None:
        gear = USE_GEAR_ACTUATOR
    head = GROUND_ROLL_HEADING_DEG
    wp = M.MAV_CMD_NAV_WAYPOINT
    a = along(*home, head, 4000, 1200)
    b = along(*home, head, -1800, 1200)
    c = along(*home, head, -1800)
    land = along(*home, head, 1200)
    items = [dict(name='Piste dönüş', cmd=wp, lat=a[0], lon=a[1], alt=cruise_rel),
             dict(name='Taban bacağı', cmd=wp, lat=b[0], lon=b[1], alt=120)]
    if gear:
        items.append(gear_item(up=False))
    items += [dict(name='Son yaklaşma', cmd=wp, lat=c[0], lon=c[1], alt=120),
              dict(name='İniş', cmd=M.MAV_CMD_NAV_LAND, lat=land[0], lon=land[1], alt=LAND_REL_M)]
    return items


def read_param(vehicle, name, timeout=3.0):
    """PARAM_VALUE döndürür; yanıt gelmezse None. Yalnızca hazırlıkta ve geri almada kısa süre mesaj okur."""
    vehicle.link.mav.param_request_read_send(vehicle.link.target_system, vehicle.link.target_component, name, -1)
    end = time.time() + timeout
    while time.time() < end:
        msg = vehicle.link.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
        if msg is not None and msg.param_id == name.decode():
            return float(msg.param_value)
    return None


def hiz_siniri(vehicle):
    """Su alma hız sınırları (SA.HizSiniri): yerde okunmuşsa o, değilse şimdi PX4'ten. Okunamazsa RuntimeError."""
    from otonom.planlayici import su_alma as SA
    return HIZ_SINIRI or SA.hiz_siniri_px4(lambda name: read_param(vehicle, name.encode()))


def set_param(vehicle, name, value):
    vehicle.link.mav.param_set_send(vehicle.link.target_system, vehicle.link.target_component, name, float(value),
                                    M.MAV_PARAM_TYPE_REAL32)


def restore_throttle(vehicle, original, result, log):
    """Özgün gaz sınırını geri yazar ve PARAM_VALUE ile doğrular. Sonuç alanlarına yazar."""
    if original is None:
        return
    for _ in range(3):
        set_param(vehicle, THROTTLE_PARAM, original)
        value = read_param(vehicle, THROTTLE_PARAM)
        if value is not None and abs(value - original) < 1e-3:
            result.update(throttle_restored=True, throttle_after=value)
            log(f'FW_THR_MAX geri alındı ve doğrulandı: {value:.3f}')
            return
        result['throttle_after'] = value
    result['throttle_restored'] = False
    log(f"UYARI: FW_THR_MAX geri alınamadı, okunan {result['throttle_after']}, özgün {original}")


def run_water_leg(vehicle, home, model, timeout=720.0, log=print, offboard_alignment=None):
    """Uçak havadayken çağrılır. timeout gerçek saniye; süreler ve hız simülasyon saatiyle ölçülür.

    Dönüş: dict(ok, reason, lat, lon, seconds, scoop_s, speed_min, speed_max, contact_s, liftoff_s, liftoff_speed, ...).
    offboard_alignment: None (varsayılan, eski davranış) ya da dict(record=<jsonl yolu>, empty_mass_kg, default_mass_kg).
    Verilirse suya hizalanma ve değiş offboard durum makinesiyle (otonom.hizalanma) yapılır; su alma ve gaz mantığı aynı kalır.

    ok yalnızca pencere SCOOP_MIN_S-SCOOP_MAX_S sürüp hız PX4 iptal bandı (FW_SC_A_VMIN-FW_SC_A_VMAX) dışında toplam
    FW_SC_OOB_T'den uzun kalmadıysa, gaz sınırı geri alındıysa ve uçak sudan ayrılıp RELEASE_ABOVE_WATER_M üstüne
    çıktıysa True.
    """
    from otonom.planlayici import su_alma as SA
    started = time.time()
    home = tuple(home[:2])  # fire_attack (enlem, boylam, AMSL) verir
    surface = WaterSurface(home)
    water_map = WaterMap()
    telemetry = Telemetry(model)
    if not telemetry.wait():
        raise RuntimeError(f'model konumu gelmiyor: {model}')
    vehicle.update(0.2)
    items, info = plan(surface, water_map, vehicle.state['lat'], vehicle.state['lon'], vehicle.state['rel'], vehicle.state['alt'])
    log(f"su {info['water_height']:.3f} m, ev {info['home_height']:.2f} m, geçiş göreli {info['run_rel']:.2f} m, yön {info['run_course']:.0f}°, "
        f"yaklaşma {info['entry'][0]:.4f},{info['entry'][1]:.4f} geçiş {info['run_start'][0]:.4f},{info['run_start'][1]:.4f} "
        f"tırmanış sonu {info['climb_end'][0]:.4f},{info['climb_end'][1]:.4f}")
    climb_seq = next(k for k, item in enumerate(items) if item['name'] == 'Tırmanış')
    result = dict(ok=False, reason='zaman aşımı', contact_s=0.0, scoop_s=None, speed_min=None, speed_max=None, entered_speed=None,
                  decel_s=None, decel_distance_m=None, decel_speed_max=None, liftoff_s=None, liftoff_distance_m=None, out_of_band_s=0.0,
                  out_of_abort_s=0.0, liftoff_speed=None, liftoff_pitch_up_deg=None,
                  exit_aoa_deg=None, airspeed_min=None, lowest=None, max_roll=0.0, wheel_z=None, water_height=info['water_height'],
                  climb_end=info['climb_end'], throttle_original=None, throttle_after=None, throttle_restored=None)
    try:
        siniri = hiz_siniri(vehicle)
    except RuntimeError as error:
        result['reason'] = f'{error}; su bölümü başlatılmadı'
        log(result['reason'])
        result.update(lat=vehicle.state['lat'], lon=vehicle.state['lon'], seconds=time.time() - started)
        return result
    # Takım açıkken suya girmek uçağı devirir (ölçüldü): görev yüklenmeden denetlenir, ölçülemiyorsa da başlanmaz
    gear_end = time.time() + GEAR_WAIT_S
    wheel = telemetry.snapshot()[3]
    while not wheel > GEAR_UP_MIN_WHEEL_Z and time.time() < gear_end:
        vehicle.update(0.2)
        wheel = telemetry.snapshot()[3]
    result['wheel_z'] = wheel
    if not wheel > GEAR_UP_MIN_WHEEL_Z:
        result['reason'] = f'iniş takımı toplu değil (tekerlek z {wheel:.3f}); su bölümü başlatılmadı'
        log(result['reason'])
        result.update(lat=vehicle.state['lat'], lon=vehicle.state['lon'], seconds=time.time() - started)
        return result
    # Görev yüklenmeden denetlenir: uçak düşük gaz sınırıyla suya yönelmesin
    original = read_param(vehicle, THROTTLE_PARAM)
    result['throttle_original'] = original
    if original is None:
        result['reason'] = 'FW_THR_MAX okunamadı; su bölümü başlatılmadı'
    elif abs(original - THROTTLE_EXPECTED_ORIGINAL) > 1e-3:
        # Bu değer "özgün" sayılıp geri yazılırsa düşük gaz sınırı kalıcı olur
        result['reason'] = f'FW_THR_MAX {original:.3f}, beklenen {THROTTLE_EXPECTED_ORIGINAL:.3f}; su bölümü başlatılmadı'
    if original is None or abs(original - THROTTLE_EXPECTED_ORIGINAL) > 1e-3:
        log(result['reason'])
        result.update(lat=vehicle.state['lat'], lon=vehicle.state['lon'], seconds=time.time() - started)
        return result

    all_items = items + return_items(home)
    vehicle.upload(all_items)
    log(f'su görevi yüklendi: {len(all_items)} adım')
    vehicle.update(0.5)
    vehicle.link.mav.mission_set_current_send(vehicle.link.target_system, vehicle.link.target_component, 0)
    vehicle.command(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 4)

    if offboard_alignment is not None:
        alignment = run_offboard_alignment(vehicle, items, info, telemetry, surface, model, offboard_alignment, log)
        result['alignment'] = alignment
        if not alignment['ok']:
            result['reason'] = f"offboard hizalanma: {alignment['reason']}"
            log(result['reason'])
            result.update(lat=vehicle.state['lat'], lon=vehicle.state['lon'], seconds=time.time() - started)
            return result

    history = []  # (sim saati, konum)
    contact_start = None
    contact_position = None
    last_contact = None
    left_water = False
    window_start = None
    window_end = None
    window_position = None
    window_last = None
    throttle_set_at = 0.0
    mission_started = offboard_alignment is not None  # offboard devri 'Geçiş sonu'ndan yapılır, baş adımlar görülmez
    last_seq = None
    throttle_speed = None
    throttle_sent = None
    throttle_time = None
    last_print = 0.0
    last_sim = None
    try:
        while time.time() - started < timeout and original is not None:
            vehicle.update(0.1)
            sim, position, rotation, wheel = telemetry.snapshot()
            if sim is None or sim == last_sim:
                continue
            last_sim = sim
            history.append((sim, position))
            while len(history) > 2 and history[0][0] < sim - SPEED_WINDOW_S:
                history.pop(0)
            dt = history[-1][0] - history[0][0]
            velocity = tuple((history[-1][1][k] - history[0][1][k]) / dt for k in range(3)) if dt > 0 else (0.0, 0.0, 0.0)
            speed = math.sqrt(sum(v * v for v in velocity))
            clear = surface.clearance(position)
            roll, pitch, yaw = body_axes(rotation)
            airspeed = vehicle.state['airspeed']
            now = time.time()
            if clear is not None:
                result['lowest'] = clear if result['lowest'] is None else min(result['lowest'], clear)
            wet = clear is not None and clear <= 0.0

            if wet:
                if contact_start is None:
                    contact_start = sim
                    contact_position = position
                    result.update(entered_speed=speed, wheel_z=wheel, decel_speed_max=speed)
                    log(f'*** SUYA GİRDİ: {now - started:.0f}s, Gazebo hızı {speed:.1f} m/s, derinlik {-clear:.2f} m, sol tekerlek z {wheel:.3f} (kapalı -0,306)')
                elif last_contact is not None:
                    result['contact_s'] += sim - last_contact
                last_contact = sim
                left_water = False
                result['max_roll'] = max(result['max_roll'], abs(math.degrees(roll)))
                result['airspeed_min'] = airspeed if result['airspeed_min'] is None else min(result['airspeed_min'], airspeed)

                if window_start is None:
                    result['decel_speed_max'] = max(result['decel_speed_max'], speed)
                    if SPEED_MIN_MS <= speed <= siniri.kontrol_ust_ms:
                        window_start = sim
                        result.update(decel_s=sim - contact_start, speed_min=speed, speed_max=speed,
                                      decel_distance_m=math.dist(position[:2], contact_position[:2]))
                        log(f"su alma penceresi başladı: yavaşlama {result['decel_s']:.1f} s, {result['decel_distance_m']:.0f} m, hız {speed:.1f} m/s")
                elif window_end is None:
                    result['speed_min'] = min(result['speed_min'], speed)
                    result['speed_max'] = max(result['speed_max'], speed)
                    if window_last is not None:
                        if not SPEED_MIN_MS <= speed <= siniri.kontrol_ust_ms:
                            result['out_of_band_s'] += sim - window_last
                        if not siniri.iptal_alt_ms <= speed <= siniri.iptal_ust_ms:
                            result['out_of_abort_s'] += sim - window_last
                    window_last = sim
                    if sim - window_start >= SCOOP_TARGET_S:
                        window_end = sim
                        result['scoop_s'] = window_end - window_start
                        log(f"su alma penceresi bitti: {result['scoop_s']:.1f} s, hız {result['speed_min']:.1f}-{result['speed_max']:.1f} m/s")
                        restore_throttle(vehicle, original, result, log)
                        # Pencere bitti: geçiş hedefi yüzeyin altında kalmasın, tırmanış adımına geç
                        vehicle.link.mav.mission_set_current_send(vehicle.link.target_system, vehicle.link.target_component, climb_seq)
                        window_position = position
                # Hız tutma: suya değdikten pencere bitene kadar
                if window_end is None and sim - throttle_set_at >= THROTTLE_PERIOD_S:
                    throttle_set_at = sim
                    # Yavaşlarken gazı erken aç: hız hedefin üstündeyken bile öngörülen hıza bakılır
                    acceleration = (speed - throttle_speed) / max(sim - throttle_time, 1e-3) if throttle_time is not None else 0.0
                    predicted = speed + acceleration * THROTTLE_LOOKAHEAD_S
                    throttle_speed, throttle_time = speed, sim
                    # Momentum direnci yalnız dolumda (hız bandında) vardır; yavaşlarken eklenirse uçak yavaşlayamaz
                    # (n9: ek gaz her durumda verildi, 45 m/s'den 27,5'e inmek 6,4 km sürdü)
                    filling = predicted <= siniri.kontrol_ust_ms
                    base = math.sqrt(THROTTLE_BASE ** 2 + (SCOOP_FILL_RATE_KGS * speed / THRUST_MAX_N if filling else 0.0))
                    command = max(THROTTLE_MIN, min(THROTTLE_MAX, base + THROTTLE_GAIN * (SPEED_TARGET_MS - predicted)))
                    if throttle_sent is None or abs(command - throttle_sent) >= THROTTLE_MIN_CHANGE:
                        set_param(vehicle, THROTTLE_PARAM, command)
                        throttle_sent = command
                        result['throttle_restored'] = None
            elif contact_start is not None and not left_water and clear is not None:
                # Sekmede de çalışır: geçerli olan, son ayrılmanın ölçümüdür
                left_water = True
                last_contact = None
                horizontal = math.hypot(velocity[0], velocity[1])
                path_up = math.atan2(velocity[2], horizontal) if horizontal > 0 else 0.0
                reference = window_end if window_end is not None else contact_start
                result['liftoff_distance_m'] = math.dist(position[:2], window_position[:2]) if window_position else None
                result.update(liftoff_s=sim - reference, liftoff_speed=speed, liftoff_pitch_up_deg=-math.degrees(pitch),
                              exit_aoa_deg=math.degrees(-pitch - path_up))
                log(f"su yüzeyinden ayrıldı: pencere sonundan {result['liftoff_s']:.1f} s, toplam temas {result['contact_s']:.1f} s, hız {speed:.1f} m/s, "
                    f"burun yukarı {-math.degrees(pitch):.2f}°, hücum {result['exit_aoa_deg']:.1f}°")

            # Hız aralığına girilemeden su gerçekten bırakıldı: gaz sınırını açık bırakma. Kısa sekmeler sayılmaz:
            # boş gövde su üstünde sekerken sınır her temasta geri açılırsa uçak hiç yavaşlamaz (ölçüldü).
            if (contact_start is not None and window_end is None and clear is not None and clear > LEFT_WATER_ABOVE_M
                    and result['throttle_restored'] is not True):
                restore_throttle(vehicle, original, result, log)
                throttle_sent = None

            period = 1.0 if (clear is not None and clear < 40.0) else 15.0
            if now - last_print > period:
                last_print = now
                s = vehicle.state
                above = f'{clear:7.2f}' if clear is not None else '    yok'
                log(f"{now - started:5.0f}s adım {s['seq']} su üstü {above} m hız {speed:5.1f} hava {airspeed:5.1f} gaz %{s['throttle']:3.0f} "
                    f"burun yukarı {-math.degrees(pitch):5.2f} yatış {math.degrees(roll):5.2f} temas {result['contact_s']:4.1f} s")

            if vehicle.state['seq'] != last_seq:
                last_seq = vehicle.state['seq']
                log(f"adım değişti: {last_seq}, sim {sim:.1f}, temastan {sim - contact_start if contact_start is not None else float('nan'):.1f} s, su üstü {clear if clear is not None else float('nan'):.1f} m, hız {speed:.1f}")
            # Görev yüklenince eski adım numarası kısa süre daha gelebilir: yeni görevin başı görülmeden sayılmaz
            if vehicle.state['seq'] <= 1:
                mission_started = True
            if contact_start is None and mission_started and vehicle.state['seq'] > climb_seq:
                result['reason'] = 'temas yok: tırmanış adımı su değmeden geçildi'
                break
            if contact_start is None and mission_started and vehicle.state['seq'] == climb_seq and clear is not None and clear > RELEASE_ABOVE_WATER_M:
                result['reason'] = f"temas yok: tırmanışa geçildi, en düşük {result['lowest']:.2f} m"
                break
            if contact_start is not None and clear is not None and clear > RELEASE_ABOVE_WATER_M:
                problems = []
                if result['scoop_s'] is None:
                    problems.append('su alma penceresi tamamlanmadı')
                else:
                    if not SCOOP_MIN_S <= result['scoop_s'] <= SCOOP_MAX_S:
                        problems.append(f"pencere {result['scoop_s']:.1f} s, şart {SCOOP_MIN_S:.0f}-{SCOOP_MAX_S:.0f} s")
                    speed_ok, why = SA.hiz_kabul(result['out_of_abort_s'], siniri)
                    if not speed_ok:
                        problems.append(why)
                result.update(ok=not problems, reason='; '.join(problems) or None)
                break
            if contact_start is not None and speed < 5.0 and sim - contact_start > 30.0:
                result.update(reason=f'suda kaldı: hız {speed:.1f} m/s')
                break
    finally:
        if result['throttle_restored'] is not True:
            restore_throttle(vehicle, original, result, log)
    if result['ok'] and result['throttle_restored'] is not True:
        result.update(ok=False, reason='FW_THR_MAX geri alınamadı')
    if not result['ok'] and contact_start is not None and vehicle.state['alt'] < RELEASE_ABOVE_WATER_M:
        # Uçak denizde kaldı: motoru güvene al
        vehicle.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196)
        log('uçak suda kaldı, motor kesildi')
    result.update(lat=vehicle.state['lat'], lon=vehicle.state['lon'], seconds=time.time() - started)
    return result


# PX4 içinde su alma (kullanıcı kararı 2026-11-11, arayüz v1: otonom/planlayici/su_alma.py). Uçuşu PX4 yapar; burada görev
# yüklenir, PX4 aşamaları izlenir, iptalden sonra yeniden deneme ya da bırakma kararı verilir. Kabul ölçümü run_water_leg
# ile aynı: Gazebo hızı ve su yüzeyi.
PX4_SCOOP_PARAM = b'FW_SC_VMAX'      # yoksa PX4 ikilisinde su alma yok: başlanmaz
PX4_LOITER_WAIT_S = 90.0             # iptal sonrası tırmanış ve bekleme için en çok


def run_water_leg_px4(vehicle, home, model, timeout=720.0, log=print):
    """run_water_leg ile aynı sözleşme (dönüş dict'i aynı alanlar + px4_* alanları). Uçak havada, görev modunda."""
    from otonom.baglanti import Baglanti, PX4_AUTO_LOITER, PX4_AUTO_MISSION, PX4_MAIN_AUTO
    from otonom.planlayici import su_alma as SA
    import queue
    ayar = SA.SuAlmaAyar()
    started = time.time()
    home = tuple(home[:2])
    surface = WaterSurface(home)
    water_map = WaterMap()
    telemetry = Telemetry(model)
    if not telemetry.wait():
        raise RuntimeError(f'model konumu gelmiyor: {model}')
    vehicle.update(0.2)
    _, info = plan(surface, water_map, vehicle.state['lat'], vehicle.state['lon'], vehicle.state['rel'], vehicle.state['alt'])
    items = SA.su_alma_maddeleri(info['entry'], info['run_start'], info['end'], info['climb_end'], info['water_height'],
                                 info['home_height'], ayar)
    names = [item['name'] for item in items]
    approach_seq, run_start_seq, run_end_seq = names.index('Yaklaşma'), names.index('Su hattı başı'), names.index('Su hattı sonu')
    log(f"PX4 su alma: su {info['water_height']:.3f} m, yön {info['run_course']:.0f}°, hat {info['run_start'][0]:.4f},{info['run_start'][1]:.4f}"
        f" → {info['end'][0]:.4f},{info['end'][1]:.4f}, hedef {ayar.hedef_hiz_ms:g} m/s {ayar.sure_s:g} s")
    result = dict(ok=False, reason='zaman aşımı', contact_s=0.0, scoop_s=None, speed_min=None, speed_max=None, entered_speed=None,
                  decel_s=None, liftoff_s=None, liftoff_speed=None, out_of_band_s=0.0, out_of_abort_s=0.0, lowest=None, max_roll=0.0, wheel_z=None,
                  water_height=info['water_height'], climb_end=info['climb_end'], tank_kg=None, attempts=0,
                  px4_phases=[], px4_report=None, px4_report_ok=None, aborts=[])

    state_of = [lambda: vehicle.state]   # Baglanti çalışırken durum ondan okunur

    def finish(reason=None):
        if reason is not None:
            result['reason'] = reason
            log(reason)
        st = state_of[0]()
        result.update(lat=st['lat'], lon=st['lon'], seconds=time.time() - started)
        return result

    gear_end = time.time() + GEAR_WAIT_S
    wheel = telemetry.snapshot()[3]
    while not wheel > GEAR_UP_MIN_WHEEL_Z and time.time() < gear_end:
        vehicle.update(0.2)
        wheel = telemetry.snapshot()[3]
    result['wheel_z'] = wheel
    if not wheel > GEAR_UP_MIN_WHEEL_Z:
        return finish(f'iniş takımı toplu değil (tekerlek z {wheel:.3f}); su bölümü başlatılmadı')
    if read_param(vehicle, PX4_SCOOP_PARAM) is None:
        return finish(f'PX4 {PX4_SCOOP_PARAM.decode()} yok: bu PX4 ikilisinde su alma yok; su bölümü başlatılmadı')
    try:
        siniri = hiz_siniri(vehicle)
    except RuntimeError as error:
        return finish(f'{error}; su bölümü başlatılmadı')
    throttle = read_param(vehicle, THROTTLE_PARAM)
    if throttle is None or abs(throttle - THROTTLE_EXPECTED_ORIGINAL) > 1e-3:
        return finish(f'FW_THR_MAX {throttle}, beklenen {THROTTLE_EXPECTED_ORIGINAL:.3f}; su bölümü başlatılmadı')

    all_items = items + return_items(home)
    vehicle.upload(all_items)
    log(f'PX4 su görevi yüklendi: {len(all_items)} adım')
    vehicle.update(0.5)
    vehicle.link.mav.mission_set_current_send(vehicle.link.target_system, vehicle.link.target_component, 0)
    vehicle.command(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_MAIN_AUTO, PX4_AUTO_MISSION)

    events = queue.Queue()   # ('metin', SuMesaji, zaman) | ('asama', ad, zaman)

    def listen(msg, now):
        kind = msg.get_type()
        if kind == 'STATUSTEXT':
            mesaj = SA.statustext_coz(msg.text)
            if mesaj is not None:
                events.put(('metin', mesaj, now))
        elif kind == 'NAMED_VALUE_FLOAT' and msg.name.replace('\x00', '').strip() == 'SU_ASAMA':
            events.put(('asama', SA.asama_adi(msg.value), now))

    link = Baglanti(vehicle.link, log, dinleyici=listen).start()
    state_of[0] = lambda: link.state
    tank = None
    try:
        from otonom.sensor import SimTankKutlesi
        tank = SimTankKutlesi(model, 0.0)   # yalnız su kütlesi
    except Exception as error:
        log(f'tank kütlesi okunamıyor: {error!r}')
    tank_start = None

    def measure_reset():
        return dict(history=[], contact=None, last_contact=None, window=None, window_last=None, kalkis=None, left=False)

    m = measure_reset()
    phase, phase_since, phase_heard = 'KAPALI', time.time(), time.time()
    watcher_reason = None          # izleyici iptal verdiyse nedeni (PX4 OPERATOR yazar)
    aborted_at = None              # iptal gönderildi/alındı, karar bekleniyor
    pending = None                 # ('yeniden'|'birak', açıklama)
    attempt = 1
    last_print = 0.0
    last_sim = None
    try:
        while time.time() - started < timeout:
            time.sleep(0.05)
            s = link.state
            now = time.time()
            while True:
                try:
                    kind, value, when = events.get_nowait()
                except queue.Empty:
                    break
                if kind == 'asama':
                    phase_heard = when
                    if value != phase:
                        phase, phase_since = value, when
                elif kind == 'metin':
                    result['px4_phases'].append((round(when - started, 1), value.asama, value.ayrinti))
                    log(f'PX4 SU: {value.asama} {value.ayrinti}')
                    if value.asama != phase:
                        phase, phase_since = value.asama, when
                    phase_heard = when
                    if value.asama == 'KALKIS' and m['kalkis'] is None:
                        m['kalkis'] = telemetry.snapshot()[0]
                    elif value.asama == 'BITTI':
                        ok, why = SA.bitti_kabul(value, siniri, ayar)
                        result.update(px4_report=value.degerler, px4_report_ok=ok)
                        log(f'PX4 raporu: {why}')
                    elif value.iptal and pending is None:
                        result['aborts'].append(dict(attempt=attempt, px4=value.neden, watcher=watcher_reason))
                        pending = SA.iptal_karari(value.neden, attempt, ayar, watcher_reason)
                        aborted_at = now
                        log(f'İPTAL ({attempt}. deneme): {pending[1]}')
            sim, position, rotation, wheel = telemetry.snapshot()
            if sim is None or position is None:
                continue
            if tank is not None and tank_start is None:
                tank_start = tank.read()

            # --- iptal sonrası karar: PX4 tırmanıp beklemeye geçince
            if pending is not None:
                main, sub = link.main_mode()
                clear = surface.clearance(position)
                loitering = main == PX4_MAIN_AUTO and sub == PX4_AUTO_LOITER and (clear is None or clear > 40.0)
                if loitering or now - aborted_at > PX4_LOITER_WAIT_S:
                    action, why = pending
                    pending, watcher_reason = None, None
                    if action == 'birak':
                        link.command(SA.MAV_CMD_USER_1, *SA.su_alma_kapat(info['water_height']))
                        result['attempts'] = attempt
                        return finish(f'PX4 su alma bırakıldı: {why}')
                    attempt += 1
                    m = measure_reset()
                    link.set_current(approach_seq)
                    if not link.set_mode(PX4_MAIN_AUTO, PX4_AUTO_MISSION, timeout=10.0):
                        return finish('yeniden deneme: görev moduna geçilemedi')
                    log(f'yeniden deneme {attempt}/{ayar.deneme_en_cok}: yaklaşmadan')
                continue

            # --- Gazebo ölçümü (run_water_leg ile aynı tanımlar)
            if sim == last_sim:
                continue
            last_sim = sim
            h = m['history']
            h.append((sim, position))
            while len(h) > 2 and h[0][0] < sim - SPEED_WINDOW_S:
                h.pop(0)
            dt = h[-1][0] - h[0][0]
            velocity = tuple((h[-1][1][k] - h[0][1][k]) / dt for k in range(3)) if dt > 0 else (0.0, 0.0, 0.0)
            speed = math.sqrt(sum(v * v for v in velocity))
            clear = surface.clearance(position)
            roll, pitch, _ = body_axes(rotation)
            if clear is not None:
                result['lowest'] = clear if result['lowest'] is None else min(result['lowest'], clear)
            wet = clear is not None and clear <= 0.0
            if wet:
                if m['contact'] is None:
                    m['contact'] = sim
                    result.update(entered_speed=speed, contact_s=0.0, scoop_s=None, speed_min=None, speed_max=None, out_of_band_s=0.0)
                    log(f'*** SUYA GİRDİ: Gazebo hızı {speed:.1f} m/s, derinlik {-clear:.2f} m, burun yukarı {-math.degrees(pitch):.2f}°')
                elif m['last_contact'] is not None:
                    result['contact_s'] += sim - m['last_contact']
                m['last_contact'], m['left'] = sim, False
                result['max_roll'] = max(result['max_roll'], abs(math.degrees(roll)))
                # Pencere: suda ilk bant içi ölçümden PX4 KALKIS'a kadar (dolum PX4 hız bandında; kabul bandı ayrı)
                if m['kalkis'] is None:
                    if m['window'] is None and ayar.bant_alt_ms <= speed <= siniri.kontrol_ust_ms:
                        m['window'] = sim
                        result.update(decel_s=sim - m['contact'], speed_min=speed, speed_max=speed)
                        log(f'su alma penceresi başladı: değmeden {result["decel_s"]:.1f} s, hız {speed:.1f} m/s')
                    elif m['window'] is not None:
                        result['speed_min'] = min(result['speed_min'], speed)
                        result['speed_max'] = max(result['speed_max'], speed)
                        if m['window_last'] is not None:
                            if not ayar.bant_alt_ms <= speed <= siniri.kontrol_ust_ms:
                                result['out_of_band_s'] += sim - m['window_last']
                            if not siniri.iptal_alt_ms <= speed <= siniri.iptal_ust_ms:
                                result['out_of_abort_s'] += sim - m['window_last']
                    m['window_last'] = sim if m['window'] is not None else None
            elif m['contact'] is not None and not m['left'] and clear is not None:
                m['left'], m['last_contact'] = True, None
                if m['window'] is not None:
                    end = m['kalkis'] if m['kalkis'] is not None else sim
                    result['scoop_s'] = end - m['window']
                result.update(liftoff_s=sim - m['contact'], liftoff_speed=speed)
                log(f"su yüzeyinden ayrıldı: temas {result['contact_s']:.1f} s, pencere {result['scoop_s']}, hız {speed:.1f} m/s")

            # --- izleyici: su aşamalarında kendi iptal kurallarımız
            if watcher_reason is None and phase in SA.SUDA:
                ahead = along(s['lat'], s['lon'], s['course'], ayar.su_onde_m)
                reason = SA.izleyici_iptal(phase, now - phase_since, now - phase_heard, water_map.is_water(*ahead), ayar)
                if reason is not None:
                    watcher_reason = reason
                    link.command(SA.MAV_CMD_DO_GO_AROUND)
                    log(f'izleyici iptal istedi: {reason}')

            if now - last_print > (1.0 if clear is not None and clear < 40.0 else 15.0):
                last_print = now
                above = f'{clear:7.2f}' if clear is not None else '    yok'
                log(f"{now - started:5.0f}s PX4 {phase:10s} adım {s['seq']} su üstü {above} m hız {speed:5.1f} hava {s['airspeed']:5.1f} "
                    f"gaz %{s['throttle']:3.0f} burun yukarı {-math.degrees(pitch):5.2f} temas {result['contact_s']:4.1f} s")

            # --- PX4 su almayı üstlenmedi: su hattına su yüzeyi irtifasında düz uçuş olur, bekleme kipine al
            if phase == 'KAPALI' and s['seq'] >= run_start_seq and m['contact'] is None and clear is not None and clear < 30.0:
                link.set_mode(PX4_MAIN_AUTO, PX4_AUTO_LOITER, timeout=5.0)
                return finish('PX4 su alma aşaması bildirmedi (SU_ASAMA yok); su hattına inilmedi, beklemeye alındı')

            # --- bitiş
            if m['contact'] is None and s['seq'] > run_end_seq and clear is not None and clear > RELEASE_ABOVE_WATER_M:
                return finish(f"temas yok: su hattı sonu geçildi, en düşük {result['lowest']}")
            if m['contact'] is not None and m['left'] and clear is not None and clear > RELEASE_ABOVE_WATER_M:
                problems = []
                if result['scoop_s'] is None:
                    problems.append('su alma penceresi yok (hız banda girmedi)')
                else:
                    if not ayar.sure_min_s <= result['scoop_s'] <= ayar.sure_max_s:
                        problems.append(f"pencere {result['scoop_s']:.1f} s, şart {ayar.sure_min_s:.0f}-{ayar.sure_max_s:.0f} s")
                    speed_ok, why = SA.hiz_kabul(result['out_of_abort_s'], siniri)
                    if not speed_ok:
                        problems.append(why)
                if tank is not None:
                    w = tank.read()
                    result['tank_kg'] = None if w is None else w - (tank_start or 0.0)
                result['attempts'] = attempt
                result.update(ok=not problems, reason='; '.join(problems) or None)
                return finish()
            if m['contact'] is not None and speed < 5.0 and sim - m['contact'] > 30.0:
                link.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196)
                return finish(f'suda kaldı: hız {speed:.1f} m/s, motor kesildi')
        return finish()
    finally:
        link.stop()
        vehicle.update(0.5)


class StandaloneVehicle:
    """Tek başına deneme için fire_attack.Vehicle ile aynı arayüz."""

    def __init__(self, url):
        self.link = mavutil.mavlink_connection(url)
        self.last_heartbeat = 0.0
        self.state = dict(armed=False, lat=0.0, lon=0.0, rel=0.0, alt=0.0, airspeed=0.0, throttle=0, seq=-1)

    def update(self, seconds):
        end = time.time() + seconds
        s = self.state
        while time.time() < end:
            if time.time() - self.last_heartbeat > 1.0:
                self.last_heartbeat = time.time()
                self.link.mav.heartbeat_send(M.MAV_TYPE_GCS, M.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            msg = self.link.recv_match(blocking=True, timeout=0.05)
            if msg is None or msg.get_srcSystem() != self.link.target_system:
                continue   # yalnız bağlı araç: aynı porta başka örnek yayın yapabilir
            kind = msg.get_type()
            if kind == 'GLOBAL_POSITION_INT':
                s.update(lat=msg.lat / 1e7, lon=msg.lon / 1e7, rel=msg.relative_alt / 1e3, alt=msg.alt / 1e3)
            elif kind == 'VFR_HUD':
                s.update(airspeed=msg.airspeed, throttle=msg.throttle)
            elif kind == 'MISSION_CURRENT':
                s.update(seq=msg.seq)
            elif kind == 'HEARTBEAT' and msg.get_srcSystem() == self.link.target_system and msg.get_srcComponent() == 1:
                s.update(armed=bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED))
            elif kind == 'STATUSTEXT':
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
                self.link.target_system, self.link.target_component, msg.seq,
                item.get('frame', M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT), item['cmd'], int(msg.seq == 0), 1, item.get('param1', 0), item.get('radius', 0), item.get('param3', 0), item.get('param4', float('nan')),
                int(item['lat'] * 1e7), int(item['lon'] * 1e7), item['alt'], 0)
        sys.exit('Görev yüklenemedi')

    def command(self, command, *params):
        params = list(params) + [0.0] * (7 - len(params))
        self.link.mav.command_long_send(self.link.target_system, self.link.target_component, command, 0, *params)


# Tek başına deneme: başlangıç noktası, kalkış yönü, 1200 m'deki iniş noktasında pistin eve göre yüksekliği
STANDALONE_WORLDS = dict(antalya=(36.8865, 30.7908, 6.27, -0.42), dalaman=(36.726091, 28.796766, 195.68, -1.54))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--url', required=True)
    ap.add_argument('--model', required=True)
    ap.add_argument('--sysid', type=int, required=True, help='beklenen araç; farklıysa hiçbir komut gönderilmez')
    ap.add_argument('--world', choices=sorted(STANDALONE_WORLDS), default='antalya')
    ap.add_argument('--offboard-hizalanma', metavar='KAYIT.jsonl', default=None,
                    help='suya hizalanmayı offboard durum makinesiyle yap (varsayılan kapalı); karar kaydı bu dosyaya')
    args = ap.parse_args()
    global WORLD, WATER_DIR, GROUND_ROLL_HEADING_DEG, LAND_REL_M
    home_lat, home_lon, GROUND_ROLL_HEADING_DEG, LAND_REL_M = STANDALONE_WORLDS[args.world]
    WORLD = args.world
    WATER_DIR = f'~/.local/share/gz-geo-terrain/{args.world}/water'
    home = (home_lat, home_lon)

    v = StandaloneVehicle(args.url)
    if not v.link.wait_heartbeat(timeout=30) or v.link.target_system != args.sysid:
        sys.exit(f'yanlış araç: {v.link.target_system}, beklenen {args.sysid}; komut gönderilmedi')
    print(f'bağlandı: sistem {v.link.target_system}', flush=True)
    v.update(3)
    # Takım arm'dan önce açık: Actuator Set değeri açılışta tanımsız (NaN), servo ara konumda kalabilir
    telemetry = Telemetry(args.model)
    telemetry.wait()
    end = time.time() + 15
    while time.time() < end and not telemetry.snapshot()[3] < GEAR_DOWN_MAX_WHEEL_Z:
        gear_command(v, up=False)
        v.update(1)
    print(f'takım açık: tekerlek z {telemetry.snapshot()[3]:.3f}', flush=True)
    if not telemetry.snapshot()[3] < GEAR_DOWN_MAX_WHEEL_Z:
        sys.exit('takım açılamadı (SIM_GZ_SV_FUNC5 301 mi?); arm edilmedi')
    takeoff = along(*home, GROUND_ROLL_HEADING_DEG, 800)
    # Kalkış adımı bitince görev takımı toplar: PX4'ün kalkış evresine bağlı değil
    v.upload([dict(name='Kalkış', cmd=M.MAV_CMD_NAV_TAKEOFF, lat=takeoff[0], lon=takeoff[1], alt=70), gear_item(up=True)]
             + return_items(home))
    v.update(0.5)
    v.link.mav.mission_set_current_send(v.link.target_system, v.link.target_component, 0)
    v.command(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 4)
    v.update(2)
    start = time.time()
    while not v.state['armed'] and time.time() - start < 60:
        v.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        v.update(3)
    if not v.state['armed']:
        sys.exit('arm edilemedi')
    # Görev takımı kalkış adımından sonra toplar; su görevi ondan önce yüklenirse takım açık kalır
    while time.time() - start < 180:
        v.update(1)
        wheel = telemetry.snapshot()[3]
        if v.state['seq'] >= 1 and wheel > GEAR_UP_MIN_WHEEL_Z:
            break
    print(f"havada: göreli {v.state['rel']:.0f} m, hava {v.state['airspeed']:.0f} m/s, adım {v.state['seq']}, tekerlek z {telemetry.snapshot()[3]:.3f}", flush=True)
    if os.environ.get('GASI_SU_PX4') == '1':   # su alma PX4 içinde (otonom/planlayici/su_alma.py)
        result = run_water_leg_px4(v, home, args.model, log=lambda text: print(text, flush=True))
    else:
        alignment = dict(record=args.offboard_hizalanma) if args.offboard_hizalanma else None
        result = run_water_leg(v, home, args.model, log=lambda text: print(text, flush=True), offboard_alignment=alignment)
    print('sonuç:', {k: (round(val, 3) if isinstance(val, float) else val) for k, val in result.items()}, flush=True)
    v.update(30)
    print(f"30 s sonra göreli {v.state['rel']:.0f} m, hava {v.state['airspeed']:.1f} m/s, gaz %{v.state['throttle']}", flush=True)


if __name__ == '__main__':
    main()
