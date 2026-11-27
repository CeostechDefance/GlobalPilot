"""Yasa 6 su değişi (rehber.flare_law6_water, update_vs_estimate): tahmin, sarsıntısız gaz, hız hedefi, gaz sınırı."""
import math
from dataclasses import replace

from otonom import rehber as r

A = replace(r.Ayar(), water_touch_law='yasa6')


def durum(**kw):
    base = dict(lat=0.0, lon=0.0, course_deg=0.0, airspeed=30.0, height=50.0, climb=-1.5, roll_deg=0.0, pitch_deg=0.0,
                mass_kg=800.0)
    base.update(kw)
    return r.Durum(**base)


def steady_pitch(mass, v, climb):
    """Kalibrasyon modeliyle tutarlı burun: CL = l6_cl0 + l6_cla·α, L = W."""
    cl = mass * r.G / (0.5 * r.RHO * v * v * r.WING_AREA_M2)
    alpha = (cl - A.l6_cl0) / A.l6_cla
    return math.degrees(alpha + math.atan2(climb, v))


def test_vs_estimate_finds_stall_speed_without_mass():
    k = r.Kontrol()
    v, climb = 31.0, -1.5
    pitch = steady_pitch(800.0, v, climb)
    for _ in range(400):   # 20 s, kütle bilerek yanlış (1200)
        k = r.update_vs_estimate(durum(airspeed=v, climb=climb, pitch_deg=pitch, mass_kg=1200.0), k, 0.05, A)
    assert k.vs_t >= 5.0
    vs_true = math.sqrt(2 * 800.0 * r.G / (r.RHO * r.WING_AREA_M2 * r.CL_MAX_CLEAN))
    assert abs(k.vs_est - vs_true) < 0.2
    assert abs(r.vs_of(k, 1200.0, A) - vs_true) < 0.2


def test_no_estimate_in_turn_or_low():
    k = r.Kontrol()
    for _ in range(200):
        k = r.update_vs_estimate(durum(roll_deg=20.0), k, 0.05, A)
        k = r.update_vs_estimate(durum(height=10.0), k, 0.05, A)
    assert math.isnan(k.vs_est)


def test_flare_starts_from_last_throttle_and_slews():
    k = replace(r.Kontrol(), last_thr=0.42, vs_est=23.2, vs_t=10.0)
    d = durum(height=10.0, airspeed=30.0, climb=-1.6, pitch_deg=-1.0)
    pitch, climb_sp, thr, speed_sp, k = r.flare_law6_water(d, k, 0.05, A)
    assert abs(thr - 0.42) <= A.l6_tslw * 0.05 + 1e-9
    assert abs(climb_sp + 1.6) < 0.05   # alçalma hedefi giriş alçalmasından başlar
    thrs = [thr]
    for _ in range(40):   # 2 s, çok yavaş: gaz artmalı ama en çok l6_tslw/s
        _, _, thr, _, k = r.flare_law6_water(durum(height=8.0, airspeed=20.0, climb=-0.7, pitch_deg=3.0), k, 0.05, A)
        thrs.append(thr)
    assert thrs[-1] > thrs[0]
    assert max(b - a for a, b in zip(thrs, thrs[1:])) <= A.l6_tslw * 0.05 + 1e-9


def test_speed_target_reaches_touchdown_ratio_at_hold():
    k = replace(r.Kontrol(), last_thr=0.4, vs_est=23.2, vs_t=10.0)
    _, _, _, _, k = r.flare_law6_water(durum(height=10.0, airspeed=30.0), k, 0.05, A)
    for _ in range(100):   # alçalma hedefi l6_tatt_s içinde oturur
        _, climb_sp, _, sp, k = r.flare_law6_water(durum(height=0.5, airspeed=27.0, climb=-0.5), k, 0.05, A)
    assert abs(sp - A.l6_vtd_k * 23.2) < 1e-6
    assert climb_sp == -A.fl_sink


def test_throttle_not_cut_on_water():
    k = replace(r.Kontrol(), last_thr=0.4, vs_est=23.2, vs_t=10.0)
    _, _, _, _, k = r.flare_law6_water(durum(height=10.0), k, 0.05, A)
    for _ in range(20):
        _, _, thr, _, k = r.flare_law6_water(durum(height=0.0, airspeed=26.0, climb=0.0, pitch_deg=4.0), k, 0.05, A)
    assert thr > 0.3


def test_default_law_is_yasa6():
    """Kullanıcı kararı 2026-11-27: varsayılan yasa 6; tahmin oturmadan Vs kütleden."""
    assert r.Ayar().water_touch_law == __import__('os').environ.get('GASI_WATER_LAW', 'yasa6')
    k = r.Kontrol()
    assert r.vs_of(k, 800.0) == r.stall_speed(800.0)


def test_sink_target_ramps_and_pitch_rate_limited():
    k = replace(r.Kontrol(), last_thr=0.4, vs_est=23.2, vs_t=10.0)
    d = durum(height=12.0, airspeed=29.0, climb=-2.0, pitch_deg=-1.0)
    pitches = []
    for _ in range(100):   # 5 s; alçalma hatası büyük: burun kalkmak ister
        pitch, climb_sp, _, _, k = r.flare_law6_water(d, k, 0.05, A)
        pitches.append(pitch)
    assert abs(climb_sp + A.fl_snk1) < 1e-9          # 4 s sonra fl_snk1
    steps = [b - a for a, b in zip([-1.0] + pitches, pitches)]
    assert max(abs(x) for x in steps) <= A.l6_prate * 0.05 + 1e-9
