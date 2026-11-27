"""PX4 içinde su alma, planlayıcı tarafı (otonom/planlayici/su_alma.py): maddeler, mesaj çözümü, iptal kararları."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from otonom.planlayici import su_alma as SA  # noqa: E402

A = SA.SuAlmaAyar()
S = SA.HizSiniri(19.44, 34.72, 3.0, 27.5)   # PX4 varsayılanları: FW_SC_A_VMIN, FW_SC_A_VMAX, FW_SC_OOB_T, FW_SC_VMAX


def maddeler():
    return SA.su_alma_maddeleri((36.70, 28.79), (36.71, 28.80), (36.725, 28.80), (36.76, 28.80), 0.3, 6.0)


def test_madde_sirasi_ve_adlar():
    assert [m['name'] for m in maddeler()] == ['Yaklaşma', 'Su alma aç', 'Su hattı başı', 'Su hattı sonu', 'Tırmanış']


def test_user1_parametreleri():
    m = maddeler()[1]
    assert m['cmd'] == 31010 and m['frame'] == SA.MAV_FRAME_MISSION
    assert m['param1'] == 1.0 and m['radius'] == 25.0 and m['param3'] == 10.0
    assert abs(m['param4'] - 22.2222) < 1e-3
    assert m['alt'] == 0.3                      # param7 = su AMSL
    assert m['lat'] == 0.0 and m['lon'] == 0.0  # param5/6 kullanılmaz


def test_su_hatti_su_yuzeyinde_goreli():
    m = maddeler()
    assert abs(m[2]['alt'] - (0.3 - 6.0)) < 1e-9 and m[2]['alt'] == m[3]['alt']
    assert abs(m[0]['alt'] - (0.3 - 6.0 + 100.0)) < 1e-9
    assert abs(m[4]['alt'] - (0.3 - 6.0 + 150.0)) < 1e-9


def test_statustext_coz():
    b = SA.statustext_coz('SU: BITTI sure=10.1 vmin=22.6 vmax=24.8 bant_disi=0.0')
    assert b.asama == 'BITTI' and b.degerler == dict(sure=10.1, vmin=22.6, vmax=24.8, bant_disi=0.0)
    i = SA.statustext_coz('SU: IPTAL KAPI yan=17.2')
    assert i.iptal and i.neden == 'KAPI' and i.degerler['yan'] == 17.2
    assert SA.statustext_coz('Takeoff detected') is None
    assert SA.statustext_coz('SU: FLARE\x00\x00').asama == 'FLARE'


def test_asama_adi():
    assert SA.asama_adi(5.0) == 'SUALMA' and SA.asama_adi(8.0) == 'IPTAL' and SA.asama_adi(float('nan')) is None


def test_iptal_yeniden_dener_en_cok_uc():
    assert SA.iptal_karari('KAPI', 1)[0] == 'yeniden'
    assert SA.iptal_karari('BANT', 2)[0] == 'yeniden'
    assert SA.iptal_karari('KAPI', 3)[0] == 'birak'


def test_radar_yeniden_denenmez():
    assert SA.iptal_karari('RADAR', 1)[0] == 'birak'


def test_izleyici_nedeni_onceliklidir():
    assert SA.iptal_karari('OPERATOR', 1, izleyici_nedeni='su bitiyor (300 m önde kara)')[0] == 'birak'
    assert SA.iptal_karari('OPERATOR', 1, izleyici_nedeni='telemetri yok (3.0 s)')[0] == 'yeniden'
    assert SA.iptal_karari('OPERATOR', 1)[0] == 'birak'   # yer istasyonu iptali: yeniden denenmez


def test_izleyici_yalniz_su_asamalarinda():
    assert SA.izleyici_iptal('YAKLASMA', 100.0, 10.0, False) is None
    assert SA.izleyici_iptal('KALKIS', 100.0, 10.0, False) is None


def test_izleyici_kurallari():
    assert SA.izleyici_iptal('FLARE', 1.0, 0.4, True) is None
    assert SA.izleyici_iptal('FLARE', 1.0, 3.0, True).startswith('telemetri')
    assert SA.izleyici_iptal('DEGME', 1.0, 0.4, False).startswith('su bitiyor')
    assert SA.izleyici_iptal('SUALMA', 15.9, 0.4, True) is None
    assert SA.izleyici_iptal('SUALMA', 16.1, 0.4, True).startswith('süre')
    assert SA.izleyici_iptal('SUALMA', 5.0, 0.4, None) is None   # su bilinmiyorsa iptal yok


def test_bitti_kabul_px4_sinirlari():
    """Hız sınırları PX4 parametrelerinden (kullanıcı 2026-11-27): süre 8-12 s, iptal bandı dışında en çok FW_SC_OOB_T."""
    assert SA.bitti_kabul(SA.statustext_coz('SU: BITTI sure=10.1 vmin=22.6 vmax=24.8 bant_disi=0.0 iptal_disi=0'), S)[0]
    # 80 km/h altı ama iptal bandı (70-125 km/h) içinde: 2026-11-27 test uçuşları (22,06 ve 20,6 m/s) artık başarılı
    assert SA.bitti_kabul(SA.statustext_coz('SU: BITTI t=10 v=20.6-23.7 d=1.4 a=0'), S)[0]
    assert not SA.bitti_kabul(SA.statustext_coz('SU: BITTI sure=7.9 vmin=22.6 vmax=24.8 iptal_disi=0'), S)[0]
    assert not SA.bitti_kabul(SA.statustext_coz('SU: BITTI t=10 v=18.0-24.8 d=4 a=3.2'), S)[0]
    assert SA.bitti_kabul(SA.statustext_coz('SU: BITTI t=10 v=18.0-24.8 d=2 a=2.9'), S)[0]
    assert not SA.bitti_kabul(SA.statustext_coz('SU: BITTI sure=10'), S)[0]
    # Sınırlar parametre olduğu için eski katı kural da PX4'ten kurulabilir (FW_SC_A_VMIN 22,22, FW_SC_OOB_T 0)
    strict = SA.HizSiniri(80 / 3.6, 100 / 3.6, 0.0, 27.5)
    assert not SA.bitti_kabul(SA.statustext_coz('SU: BITTI t=10 v=22.0-24.8 d=0.6 a=0.6'), strict)[0]


def test_hiz_siniri_px4_okunur():
    values = dict(FW_SC_A_VMIN=19.44, FW_SC_A_VMAX=34.72, FW_SC_OOB_T=3.0, FW_SC_VMAX=27.5)
    assert SA.hiz_siniri_px4(values.get) == S
    with pytest.raises(RuntimeError):
        SA.hiz_siniri_px4(lambda name: None if name == 'FW_SC_OOB_T' else values[name])
    assert SA.hiz_kabul(3.0, S)[0] and not SA.hiz_kabul(3.01, S)[0]


def test_iptal_tamam_yeni_iptal_degil():
    m = SA.statustext_coz('SU: IPTAL tamam, bekleme (AUTO.LOITER)')
    assert m.asama == 'IPTAL_TAMAM' and not m.iptal
    mod = SA.statustext_coz('SU: IPTAL MOD')
    assert mod.iptal and mod.neden == 'MOD' and SA.iptal_karari(mod.neden, 1)[0] == 'birak'


def test_kalkis_raporu_degerleri():
    k = SA.statustext_coz('SU: KALKIS sure=10.0 vmin=23.1 vmax=25.9 bant_disi=0.0 iptal_disi=0.0')
    assert k.asama == 'KALKIS' and SA.bitti_kabul(k, S)[0]


def test_kesik_rapor_kabul_sayilmaz():
    k = SA.statustext_coz('SU: KALKIS sure=10.0 vmin=24.9 vmax=27.5 bant_disi')
    assert not SA.bitti_kabul(k, S)[0]
    assert not SA.bitti_kabul(SA.statustext_coz('SU: BITTI t=10 v=23-25 d=0 a'), S)[0]


def test_kisa_bicim():
    b = SA.statustext_coz('SU: BITTI t=10.0 v=22.1-25.1 d=0.6 k=0')
    assert b.degerler == dict(sure=10.0, vmin=22.1, vmax=25.1, bant_disi=0.6, kabul=0.0)
    assert not SA.bitti_kabul(b, S)[0]                      # iptal bandı dışı süre (a) yok: kesik rapor
    a = SA.statustext_coz('SU: BITTI t=10.0 v=22.1-25.1 d=0.6 a=0.0 k=1')
    assert a.degerler['iptal_disi'] == 0.0 and SA.bitti_kabul(a, S)[0]
    assert SA.bitti_kabul(SA.statustext_coz('SU: KALKIS t=10.2 v=22.9-25.8 d=0 a=0 k=1'), S)[0]
    d = SA.statustext_coz('SU: DEGME v=24.5 p=1.4 vz=0.37')
    assert d.degerler == dict(hiz=24.5, burun=1.4, vz=0.37)
    i = SA.statustext_coz('SU: IPTAL BANT t=2.7 v=20.8-27.5 d=3.1')
    assert i.iptal and i.neden == 'BANT' and i.degerler['vmin'] == 20.8 and i.degerler['bant_disi'] == 3.1
