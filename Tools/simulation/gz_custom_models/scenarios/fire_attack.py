#!/usr/bin/env python3
"""Yangına müdahale senaryosu: kalkış, yangına yaklaşma, alçak geçiş, dönüş ve piste iniş.

Önce simülasyonu başlatın, sonra bu betiği çalıştırın:

    make px4_sitl gz_gasi_ffa_antalya
    python3 Tools/simulation/gz_custom_models/scenarios/fire_attack.py

Su bırakma henüz modellenmedi: alçak geçişte uçak yalnızca yangının üzerinden uçar.
"""
import argparse
import math
import sys
import time

from pymavlink import mavutil

M = mavutil.mavlink
RUNWAY_HEADING_DEG = 5.0
RUN_BEARING_DEG = 315.0   # direction of the pass over the fire
CRUISE_REL_M = 450.0      # above the runway; the highest ground on the way is about 240 m above it
RUN_REL_M = 330.0         # about 100 m above the ground at the fire
PATTERN_REL_M = 120.0


def offset(lat, lon, north, east):
    return lat + north / 111320.0, lon + east / (111320.0 * math.cos(math.radians(lat)))


def along(lat, lon, bearing_deg, distance, right=0.0):
    b = math.radians(bearing_deg)
    return offset(lat, lon, distance * math.cos(b) - right * math.sin(b), distance * math.sin(b) + right * math.cos(b))


def distance_m(lat1, lon1, lat2, lon2):
    return math.hypot((lat2 - lat1) * 111320.0, (lon2 - lon1) * 111320.0 * math.cos(math.radians(lat1)))


class Vehicle:
    def __init__(self, url):
        self.link = mavutil.mavlink_connection(url)
        self.last_heartbeat = 0.0
        self.state = dict(armed=False, lat=0.0, lon=0.0, rel=0.0, airspeed=0.0, throttle=0, landed=0, seq=-1, seq_seen=False)

    def wait_connection(self, timeout):
        if self.link.wait_heartbeat(timeout=timeout) is None:
            sys.exit('PX4 bulunamadı. Önce simülasyonu başlatın: make px4_sitl gz_gasi_ffa_antalya')

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
                s.update(lat=msg.lat / 1e7, lon=msg.lon / 1e7, rel=msg.relative_alt / 1e3)
            elif kind == 'VFR_HUD':
                s.update(airspeed=msg.airspeed, throttle=msg.throttle)
            elif kind == 'EXTENDED_SYS_STATE':
                s.update(landed=msg.landed_state)
            elif kind == 'MISSION_CURRENT':
                s.update(seq=msg.seq)
            elif kind == 'HEARTBEAT' and msg.get_srcSystem() == self.link.target_system and msg.get_srcComponent() == 1:
                s.update(armed=bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED))
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
                self.link.target_system, self.link.target_component, msg.seq, M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                item['cmd'], 0, 1, 0, 0, 0, float('nan'), int(item['lat'] * 1e7), int(item['lon'] * 1e7), item['alt'], 0)
        sys.exit('Görev yüklenemedi: PX4 yanıt vermedi')

    def command(self, command, *params):
        params = list(params) + [0.0] * (7 - len(params))
        self.link.mav.command_long_send(self.link.target_system, self.link.target_component, command, 0, *params)


def build_mission(home_lat, home_lon, fire_lat, fire_lon):
    def item(name, cmd, point, alt):
        return dict(name=name, cmd=cmd, lat=point[0], lon=point[1], alt=alt)

    runway = (home_lat, home_lon, RUNWAY_HEADING_DEG)
    run = (fire_lat, fire_lon, RUN_BEARING_DEG)
    wp, land = M.MAV_CMD_NAV_WAYPOINT, M.MAV_CMD_NAV_LAND
    return [
        item('Kalkış', M.MAV_CMD_NAV_TAKEOFF, along(*runway, 800), 70),
        item('Yangına yaklaşma (yangına 5 km)', wp, along(*run, -5000), CRUISE_REL_M),
        item('Alçalma, geçiş hattına giriş (yangına 2,5 km)', wp, along(*run, -2500), RUN_REL_M),
        item('Yangının üzerinden geçiş', wp, (fire_lat, fire_lon), RUN_REL_M),
        item('Geçişten çıkış ve tırmanma', wp, along(*run, 3000), CRUISE_REL_M),
        item('Piste dönüş', wp, along(*runway, 4000, 1200), 300),
        item('Taban bacağı', wp, along(*runway, -1800, 1200), PATTERN_REL_M),
        item('Son yaklaşma', wp, along(*runway, -1800), PATTERN_REL_M),
        item('İniş', land, along(*runway, 1200), 0),
    ]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--fire-lat', type=float, default=36.9759)
    ap.add_argument('--fire-lon', type=float, default=30.6779)
    ap.add_argument('--url', default='udpin:0.0.0.0:14540')
    ap.add_argument('--timeout', type=float, default=1200.0, help='saniye')
    args = ap.parse_args()

    v = Vehicle(args.url)
    print('PX4 bekleniyor...', flush=True)
    v.wait_connection(120)
    v.update(5)
    start_wait = time.time()
    while v.state['lat'] == 0.0 and time.time() - start_wait < 120:
        v.update(1)
    s = v.state
    home = (s['lat'], s['lon'])
    items = build_mission(*home, args.fire_lat, args.fire_lon)
    v.upload(items)
    print(f'Görev yüklendi: {len(items)} adım', flush=True)

    v.update(2)
    v.command(M.MAV_CMD_DO_SET_MODE, 1, 4, 4)
    v.update(2)
    end = time.time() + 180
    while not s['armed'] and time.time() < end:
        v.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        v.update(3)
    if not s['armed']:
        sys.exit('Uçak arm edilemedi: 3 dakika içinde hazır olmadı')
    print('Arm edildi, görev başlıyor', flush=True)

    t0 = time.time()
    last_print = 0.0
    current = -1
    airborne = False
    closest = 1e12
    while time.time() - t0 < args.timeout:
        v.update(0.5)
        if s['seq'] != current and 0 <= s['seq'] < len(items):
            current = s['seq']
            print(f'[{time.time() - t0:5.0f} s] {current + 1}/{len(items)}: {items[current]["name"]}', flush=True)
        d = distance_m(s['lat'], s['lon'], args.fire_lat, args.fire_lon)
        closest = min(closest, d)
        airborne = airborne or s['rel'] > 20
        if time.time() - last_print > 20:
            last_print = time.time()
            print(f'[{time.time() - t0:5.0f} s] yükseklik {s["rel"]:5.0f} m, hız {s["airspeed"]:4.1f} m/s, gaz %{s["throttle"]:3d}, yangına {d / 1000:5.1f} km', flush=True)
        if airborne and not s['armed']:
            print(f'[{time.time() - t0:5.0f} s] İniş tamamlandı. Yangına en yakın geçiş {closest:.0f} m.', flush=True)
            return
    sys.exit(f'Süre doldu ({args.timeout:.0f} s), görev bitmedi')


if __name__ == '__main__':
    main()
