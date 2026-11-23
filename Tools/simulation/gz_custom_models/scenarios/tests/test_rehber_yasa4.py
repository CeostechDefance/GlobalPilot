"""Suya değiş yasa 4 (rehber.flare_law4_water): burun sabit, gaz alçalmayı tutar."""
import os
import sys
from dataclasses import replace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from otonom import rehber as r  # noqa: E402

A = replace(r.Ayar(), water_touch_law='yasa4', fl_att_water=6.0)


def durum(**kw):
    base = dict(lat=0.0, lon=0.0, course_deg=0.0, airspeed=30.0, height=8.0, climb=-0.75, roll_deg=0.0, pitch_deg=1.0, mass_kg=800.0)
    base.update(kw)
    return r.Durum(**base)


def test_yavaslama_gaz_rolanti_tirmanamaz():
    k = r.Kontrol()
    pitch, sp, thr, k = r.flare_law4_water(durum(climb=0.5, pitch_deg=1.0), k, 0.1, A)
    assert k.fl_stage == 0 and thr == A.thr_limits[0]
    assert pitch < 1.0                       # tırmanıyor: burun başlangıcın altına iner
    assert sp == -A.fl_snk1


def test_hizli_alcalista_burun_kalkar_ve_sabit_aciya_gecer():
    k = r.Kontrol()
    for _ in range(200):
        pitch, _, _, k = r.flare_law4_water(durum(climb=-2.0, pitch_deg=1.0), k, 0.1, A)
        if k.fl_stage == 1:
            break
    assert k.fl_stage == 1
    for _ in range(20):
        pitch, _, _, k = r.flare_law4_water(durum(), k, 0.1, A)
    assert abs(pitch - 6.0) < 1e-6


def test_hold_altinda_sabit_aci():
    pitch, sp, _, k = r.flare_law4_water(durum(height=2.0), r.Kontrol(), 0.1, A)
    assert k.fl_stage == 1 and sp == -A.fl_sink


def sabit(**kw):
    k = replace(r.Kontrol(), flare=True, fl_stage=1, fl_t1=0.0, fl_p1=6.0, fl_t=5.0)
    return r.flare_law4_water(durum(**kw), k, 0.1, A)


def test_sabit_acida_hizli_alcalista_gaz_artar():
    assert sabit(climb=-1.5)[2] > sabit(climb=-0.75)[2]


def test_hiz_korumasi_burnu_en_cok_iki_derece_indirir():
    assert abs(sabit(airspeed=15.0)[0] - 4.0) < 1e-6


def test_varsayilan_yasa3():
    assert r.Ayar().water_touch_law in ('yasa3', os.environ.get('GASI_WATER_LAW', 'yasa3'))
