"""PX4 içinde su alma (kullanıcı kararı 2026-11-11): görev maddeleri, PX4 aşama mesajlarının çözümü, izleyici ve
iptal/yeniden deneme kuralları. Uçuşun kendisi (yaklaşma, flare, değme, su alma, kalkış) PX4'te; burası yalnız ne
uçulacağına ve iptalden sonra ne yapılacağına karar verir.

Arayüz v1 (globalpilot-6b ile, 2026-11-11):
- Madde sırası: [Yaklaşma] → [DO MAV_CMD_USER_1 su alma aç] → [Su hattı başı, alt = su] → [Su hattı sonu, alt = su]
  → [Tırmanış] → piste dönüş.
  USER_1: param1 1 aç / 0 kapat, param2 hedef hız m/s, param3 süre s, param4 bant alt sınırı m/s, param7 (z) su AMSL.
  param5/6 kullanılmaz: MISSION_ITEM_INT'te x/y tamsayıdır (enlem/boylam ·1e7).
- Telemetri: STATUSTEXT "SU: <AŞAMA> <ayrıntı>" her aşama değişiminde; NAMED_VALUE_FLOAT "SU_ASAMA" 2 Hz.
- İptal: MAV_CMD_DO_GO_AROUND; PX4 su + FW_SC_GA_ALT'a düz tırmanır, AUTO.LOITER'da bekler, görev ilerlemez.
  Karar burada: yeniden deneme (MISSION_SET_CURRENT yaklaşma + AUTO.MISSION) ya da bırakma.

Kabul ölçütü: suda 8-12 s. Hız sınırları PX4 parametrelerinden (kullanıcı 2026-11-27: "sınırlar px4 parametre
kısmından alınsın"), PX4'ün kendi su alma kuralıyla aynı: gaz kontrol bandını (param4 .. FW_SC_VMAX, 80 km/h-27,5 m/s)
hedefler; su alma ancak iptal bandının (FW_SC_A_VMIN .. FW_SC_A_VMAX) dışında toplam FW_SC_OOB_T'den uzun kalınırsa
başarısız sayılır (önceden sabit 22,22-27,78 m/s ve bant dışı süre sıfır). Değerlendirme Gazebo ölçümüyle yapılır
(water_leg); PX4'ün BITTI raporu yalnız karşılaştırma için kaydedilir.

Simülasyon ya da MAVLink bağlantısı bilmez: girdi metin ve sayılar, çıktı görev maddesi (dict) ve karar.
"""
import math
import re
from dataclasses import dataclass, field

from . import rota as R

__all__ = ['MAV_CMD_USER_1', 'MAV_CMD_DO_GO_AROUND', 'ASAMALAR', 'SuAlmaAyar', 'SuMesaji', 'su_alma_maddeleri',
           'su_alma_kapat', 'statustext_coz', 'asama_adi', 'iptal_karari', 'izleyici_iptal', 'bitti_kabul',
           'HizSiniri', 'KABUL_PARAMS', 'hiz_siniri_px4', 'hiz_kabul', 'pencere_kabul']

MAV_CMD_USER_1 = 31010
MAV_CMD_DO_GO_AROUND = 191
MAV_CMD_NAV_WAYPOINT = 16
MAV_FRAME_MISSION = 2

# NAMED_VALUE_FLOAT "SU_ASAMA" değerleri
ASAMALAR = {0: 'KAPALI', 1: 'YAKLASMA', 2: 'KAPI_GECTI', 3: 'FLARE', 4: 'DEGME', 5: 'SUALMA', 6: 'KALKIS', 7: 'BITTI',
            8: 'IPTAL'}
SUDA = ('FLARE', 'DEGME', 'SUALMA')           # su yüzeyine yakın aşamalar: izleyici bunlarda denetler
# Yeniden denemenin işe yarayabileceği nedenler: hizalanma ve hız. RADAR ölçüm arızasıdır, yeniden denenmez.
# OPERATOR: iptali biz (izleyici) ya da yer istasyonu verdi; izleyicinin nedeni ayrıca bilinir (iptal_karari'na verilir).
YENIDEN_DENENIR = ('KAPI', 'HIZ_DUSUK', 'YATIS', 'YAN_SAPMA', 'BANT')


@dataclass(frozen=True)
class SuAlmaAyar:
    hedef_hiz_ms: float = 25.0
    sure_s: float = 10.0
    bant_alt_ms: float = 80 / 3.6           # kontrol bandı alt sınırı (görev maddesi param4); üstü PX4 FW_SC_VMAX
    sure_min_s: float = 8.0
    sure_max_s: float = 12.0
    deneme_en_cok: int = 3                  # bugünkü offboard hizalanma ile aynı
    yaklasma_ustu_m: float = 100.0          # su üstü yaklaşma ve su_gecisi giriş irtifası
    tirmanis_ustu_m: float = 150.0          # su alma sonrası tırmanış (su üstü)
    # izleyici
    su_onde_m: float = 300.0                # su aşamalarında uçağın bu kadar önü su olmalı
    telemetri_en_cok_s: float = 2.5         # SU_ASAMA 2 Hz; bundan uzun sessizlik su aşamasında iptal
    sualma_pay_s: float = 6.0               # SUALMA süre + bu kadar içinde KALKIS gelmezse iptal


KABUL_PARAMS = ('FW_SC_A_VMIN', 'FW_SC_A_VMAX', 'FW_SC_OOB_T', 'FW_SC_VMAX')


@dataclass(frozen=True)
class HizSiniri:
    """Su almanın hız sınırları, PX4 parametrelerinden (hiz_siniri_px4)."""
    iptal_alt_ms: float                     # FW_SC_A_VMIN: iptal bandı alt sınırı
    iptal_ust_ms: float                     # FW_SC_A_VMAX: iptal bandı üst sınırı
    disarida_en_cok_s: float                # FW_SC_OOB_T: iptal bandı dışında en çok bu kadar (toplam)
    kontrol_ust_ms: float                   # FW_SC_VMAX: kontrol bandı üstü; dolum ve pencere bunun altında


def hiz_siniri_px4(read_param):
    """read_param(ad) -> değer ya da None. Okunamayan parametre varsa RuntimeError: kabul sessizce varsayılana düşmesin."""
    values = R.read_px4_params(read_param, KABUL_PARAMS)
    return HizSiniri(*(float(values[name]) for name in KABUL_PARAMS))


def hiz_kabul(disarida_s, siniri):
    """İptal bandı dışında geçen toplam süre (s) kabul ediliyor mu: (bool, gerekçe ya da None). PX4'ün iptal kuralıyla aynı."""
    if disarida_s > siniri.disarida_en_cok_s:
        return False, (f'hız {siniri.iptal_alt_ms:.2f}-{siniri.iptal_ust_ms:.2f} m/s dışında {disarida_s:.1f} s, '
                       f'en çok {siniri.disarida_en_cok_s:.1f} s (PX4 FW_SC_OOB_T)')
    return True, None


@dataclass
class SuMesaji:
    asama: str
    ayrinti: str = ''
    degerler: dict = field(default_factory=dict)   # "anahtar=sayı" çiftleri
    neden: str = None                              # IPTAL'de

    @property
    def iptal(self):
        return self.asama == 'IPTAL'


def su_alma_maddeleri(entry, run_start, run_end, climb_end, water_amsl, home_amsl, ayar=SuAlmaAyar()):
    """Su alma görev maddeleri (göreli irtifa). Noktalar (enlem, boylam); su ve ev AMSL.
    Ad sabittir: water_leg ve izleyici maddeleri adla bulur."""
    water_rel = water_amsl - home_amsl
    wp = MAV_CMD_NAV_WAYPOINT
    return [
        dict(name='Yaklaşma', cmd=wp, lat=entry[0], lon=entry[1], alt=water_rel + ayar.yaklasma_ustu_m),
        dict(name='Su alma aç', cmd=MAV_CMD_USER_1, lat=0.0, lon=0.0, alt=water_amsl, frame=MAV_FRAME_MISSION,
             param1=1.0, radius=ayar.hedef_hiz_ms, param3=ayar.sure_s, param4=ayar.bant_alt_ms),
        dict(name='Su hattı başı', cmd=wp, lat=run_start[0], lon=run_start[1], alt=water_rel),
        dict(name='Su hattı sonu', cmd=wp, lat=run_end[0], lon=run_end[1], alt=water_rel),
        dict(name='Tırmanış', cmd=wp, lat=climb_end[0], lon=climb_end[1], alt=water_rel + ayar.tirmanis_ustu_m),
    ]


def su_alma_kapat(water_amsl):
    """COMMAND_LONG parametreleri (param1..7): su alma kapalı."""
    return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, water_amsl)


_SAYI = re.compile(r'(\w+)=(-?\d+(?:\.\d+)?)')
_ARALIK = re.compile(r'\b(v)=(-?\d+(?:\.\d+)?--?\d+(?:\.\d+)?)')
# PX4 kısa biçimi (STATUSTEXT 50 karakter, ee 2026-11-12): t bant içi süre, v en düşük-en yüksek hız, d bant dışı s,
# a iptal bandı dışı s, k kabul, p burun, vz alçalma. Uzun anahtarlar da kabul edilir.
_KISA = {'t': 'sure', 'd': 'bant_disi', 'a': 'iptal_disi', 'k': 'kabul', 'p': 'burun', 'v': 'hiz'}


def statustext_coz(text):
    """'SU: <AŞAMA> <ayrıntı>' → SuMesaji; başka metin → None."""
    text = (text or '').replace('\x00', '').strip()
    if not text.startswith('SU:'):
        return None
    parts = text[3:].split(None, 1)
    if not parts:
        return None
    asama = parts[0].upper()
    ayrinti = parts[1].strip() if len(parts) > 1 else ''
    degerler = {}
    for k, v in _ARALIK.findall(ayrinti):          # kısa biçim: v=22.1-25.1
        lo, hi = v.split('-', 1) if not v.startswith('-') else ('-' + v[1:].split('-', 1)[0], v[1:].split('-', 1)[1])
        degerler[f'{k}min'], degerler[f'{k}max'] = float(lo), float(hi)
    for k, v in _SAYI.findall(_ARALIK.sub('', ayrinti)):
        degerler[_KISA.get(k, k)] = float(v)
    mesaj = SuMesaji(asama, ayrinti, degerler)
    if asama == 'IPTAL':
        neden = ayrinti.split()[0].strip(',').upper() if ayrinti else 'BILINMIYOR'
        if neden == 'TAMAM':          # "IPTAL tamam, bekleme (AUTO.LOITER)": iptal bitti bildirimi, yeni iptal değil
            mesaj.asama = 'IPTAL_TAMAM'
        else:
            mesaj.neden = neden       # MOD: operatör kip değiştirdi (yeniden denenmez)
    return mesaj


def asama_adi(value):
    """NAMED_VALUE_FLOAT SU_ASAMA değeri → ad."""
    if value is None or not math.isfinite(value):
        return None
    return ASAMALAR.get(int(round(value)), f'BILINMEYEN_{value:g}')


def iptal_karari(neden, deneme, ayar=SuAlmaAyar(), izleyici_nedeni=None):
    """İptalden sonra: ('yeniden', açıklama) ya da ('birak', açıklama). deneme: bu iptal dahil yapılan deneme sayısı.
    izleyici_nedeni: iptali izleyici verdiyse (PX4 neden OPERATOR yazar) izleyicinin nedeni."""
    if izleyici_nedeni is not None:
        neden_metni = f'izleyici: {izleyici_nedeni}'
        tekrar = izleyici_nedeni.startswith(('telemetri', 'süre'))   # su bitiyorsa aynı hat yine biter
    else:
        neden_metni = f'PX4: {neden}'
        tekrar = neden in YENIDEN_DENENIR
    if not tekrar:
        return 'birak', f'{neden_metni}; yeniden denenmez'
    if deneme >= ayar.deneme_en_cok:
        return 'birak', f'{neden_metni}; {deneme} denemede su alınamadı'
    return 'yeniden', f'{neden_metni}; deneme {deneme + 1}/{ayar.deneme_en_cok}'


def izleyici_iptal(asama, asama_suresi_s, telemetri_yasi_s, su_onde, ayar=SuAlmaAyar()):
    """Planlayıcının kendi iptal kuralları. İptal nedeni (metin) ya da None.
    asama: SU_ASAMA adı; asama_suresi_s: bu aşamada geçen süre; telemetri_yasi_s: son SU_ASAMA'dan beri;
    su_onde: uçağın önünde su_onde_m boyunca su var mı (None: bilinmiyor)."""
    if asama not in SUDA:
        return None
    if telemetri_yasi_s > ayar.telemetri_en_cok_s:
        return f'telemetri yok ({telemetri_yasi_s:.1f} s)'
    if su_onde is False:
        return f'su bitiyor ({ayar.su_onde_m:.0f} m önde kara)'
    if asama == 'SUALMA' and asama_suresi_s > ayar.sure_s + ayar.sualma_pay_s:
        return f'süre aşıldı: SUALMA {asama_suresi_s:.1f} s'
    return None


def pencere_kabul(scoop_s, disarida_s, siniri, ayar=SuAlmaAyar()):
    """Su alma penceresinin kabul ölçütü: sorunların listesi, boşsa kabul. scoop_s pencere süresi (None: pencere
    tamamlanmadı), disarida_s iptal bandı dışında geçen toplam süre. İki su alma yolu ve PX4 raporu aynı ölçütü kullanır."""
    if scoop_s is None:
        return ['su alma penceresi tamamlanmadı']
    sorun = []
    if not ayar.sure_min_s <= scoop_s <= ayar.sure_max_s:
        sorun.append(f'pencere {scoop_s:.1f} s, şart {ayar.sure_min_s:.0f}-{ayar.sure_max_s:.0f} s')
    ok, why = hiz_kabul(disarida_s, siniri)
    if not ok:
        sorun.append(why)
    return sorun


def bitti_kabul(mesaj, siniri, ayar=SuAlmaAyar()):
    """PX4 BITTI raporu kabul ölçütünü tutuyor mu: (bool, açıklama). Karşılaştırma içindir; asıl karar Gazebo ölçümü.
    siniri: HizSiniri (PX4 parametreleri). Ölçüt pencere_kabul; iptal bandı dışı süre raporun a= alanı."""
    d = mesaj.degerler
    eksik = [k for k in ('sure', 'vmin', 'vmax', 'iptal_disi') if k not in d]
    if eksik:
        return False, f'raporda eksik: {", ".join(eksik)} (STATUSTEXT 50 karakterde kesilmiş olabilir)'
    sorun = pencere_kabul(d['sure'], d['iptal_disi'], siniri, ayar)
    return not sorun, '; '.join(sorun) or 'kabul'
