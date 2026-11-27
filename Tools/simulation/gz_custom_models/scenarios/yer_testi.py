#!/usr/bin/env python3
"""GASI FFA yer testi: uçak yerde ve arm edilmeden önce hareketli yüzeyler tek tek, ardından motor çalıştırılır.

PX4'ün çıkış testi (MAV_CMD_ACTUATOR_TEST) kullanılır; arm gerekmez, uçak kıpırdamaz (motor testi düşük gazla, frenler
çekili kalır). Her yüzey önce bir yöne, sonra öbür yöne tam gider ve ortaya döner. Kanallar airframe 4022'den:
201 sol kanatçık, 202 sağ kanatçık, 203 irtifa dümeni, 204 istikamet dümeni, 205 flap, 101 motor.

    python3 yer_testi.py                       # örnek 0 (gösteri)
    python3 yer_testi.py --url udpin:0.0.0.0:14543 --sysid 4
"""
import argparse
import math
import sys
import time

from pymavlink import mavutil

M = mavutil.mavlink
MAV_CMD_ACTUATOR_TEST = 310  # MAVLink common; pymavlink varsayılan sözlüğünde adı yok
YUZEYLER = [  # (ad, çıkış işlevi, uçlar)
    ('Sol kanatçık', 201, (1.0, -1.0)),
    ('Sağ kanatçık', 202, (1.0, -1.0)),
    ('İrtifa dümeni', 203, (1.0, -1.0)),
    ('İstikamet dümeni', 204, (1.0, -1.0)),
    ('Flaplar', 205, (1.0, -1.0)),   # flap tek yönlü: -1 kapalı, +1 tam açık (40°)
]
MOTOR = ('Motor', 101)


class Arac:
    def __init__(self, url, sysid):
        self.link = mavutil.mavlink_connection(url, source_system=245)
        if not self.link.wait_heartbeat(timeout=30) or self.link.target_system != sysid:
            sys.exit(f'Yanlış araç: sistem {self.link.target_system}, beklenen {sysid}. Komut gönderilmedi.')
        self.armed = False
        self.reddedildi = False

    def oku(self, sure):
        bitis = time.time() + sure
        while time.time() < bitis:
            msg = self.link.recv_match(blocking=True, timeout=0.1)
            if msg and msg.get_type() == 'HEARTBEAT' and msg.get_srcSystem() == self.link.target_system and msg.get_srcComponent() == 1:
                self.armed = bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
            elif msg and msg.get_type() == 'COMMAND_ACK' and msg.command == MAV_CMD_ACTUATOR_TEST and msg.result != 0:
                if not self.reddedildi:
                    print(f'  UYARI: PX4 çıkış testini reddetti (sonuç {msg.result})', flush=True)
                self.reddedildi = True

    def test(self, islev, deger, zaman_asimi=3.0):
        """Çıkışa değer ver; deger NaN ise testi bırak (zaman aşımı 0 = bırak). PX4 en çok 3 s tutar, uzun adımlar
        yenilenir. İşlev PX4 çıkış işlevi + 1000 olarak gönderilir (Commander: 1000'in altı MAVLink numaralarıdır).
        Yanıt beklenmez (her adımda beklemek testi yavaşlatıyordu); reddedilirse oku() içinde bildirilir."""
        if math.isnan(deger):
            deger, zaman_asimi = 0.0, 0.0
        self.link.mav.command_long_send(self.link.target_system, self.link.target_component, MAV_CMD_ACTUATOR_TEST, 0,
                                        deger, zaman_asimi, 0, 0, islev + 1000, 0, 0)

    def tut(self, islev, deger, sure):
        bitis = time.time() + sure
        while time.time() < bitis:
            self.test(islev, deger, min(3.0, bitis - time.time() + 0.3))
            self.oku(min(1.0, max(bitis - time.time(), 0.0)))


def yavas(arac, islev, a, b, sure):
    """a'dan b'ye sure saniyede yumuşak geçiş (yüzey ani sıçramasın)."""
    adim = 20
    for i in range(1, adim + 1):
        arac.test(islev, a + (b - a) * i / adim, 1.0)
        arac.oku(sure / adim)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--url', default='udpin:0.0.0.0:14540')
    ap.add_argument('--sysid', type=int, default=1)
    ap.add_argument('--motor-gaz', type=float, default=0.18,
                    help='motor testi gazı (0-1). 0.18: fren eklentisi motor %%20 gazı (motor_off) geçince freni bırakır, altında uçak yerinde kalır')
    ap.add_argument('--motor-sure', type=float, default=6.0, help='motorun çalışma süresi (s)')
    ap.add_argument('--yalniz-motor', action='store_true', help='yüzeyleri atla, yalnız motoru çalıştır')
    args = ap.parse_args()
    if args.yalniz_motor:
        YUZEYLER.clear()

    arac = Arac(args.url, args.sysid)
    arac.oku(1.0)
    if arac.armed:
        sys.exit('Uçak arm edilmiş: yer testi yalnız arm edilmeden önce yapılır.')
    print('== Yer testi başladı: önce hareketli yüzeyler, sonra motor', flush=True)

    for ad, islev, (uc1, uc2) in YUZEYLER:
        print(f'  {ad}: {uc1:+.0f} yönüne, {uc2:+.0f} yönüne, ortaya', flush=True)
        orta = -1.0 if islev == 205 else 0.0          # flapın dinlenme konumu kapalı (-1)
        yavas(arac, islev, orta, uc1, 1.0)
        arac.tut(islev, uc1, 0.8)
        yavas(arac, islev, uc1, uc2, 1.5)
        arac.tut(islev, uc2, 0.8)
        yavas(arac, islev, uc2, orta, 1.0)
        arac.test(islev, math.nan)                    # testi bırak, çıkış normal denetime döner
        arac.oku(0.5)

    ad, islev = MOTOR
    print(f'  {ad}: %{args.motor_gaz * 100:.0f} gazla {args.motor_sure:.0f} s', flush=True)
    yavas(arac, islev, 0.0, args.motor_gaz, 2.0)
    arac.tut(islev, args.motor_gaz, args.motor_sure)
    yavas(arac, islev, args.motor_gaz, 0.0, 2.0)
    arac.test(islev, math.nan)
    arac.oku(0.5)
    print('== Yer testi bitti', flush=True)


if __name__ == '__main__':
    main()
