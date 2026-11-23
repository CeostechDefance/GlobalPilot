# GlobalPilot

GlobalPilot, Ceostech'in [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) v1.17 tabanlı uçuş yazılımıdır. Bu depo
PX4'ün tamamını içerir; üstüne GASI FFA otonom amfibi yangın söndürme uçağı için uçuş kontrolü değişiklikleri ve Gazebo
Harmonic simülasyonu eklenmiştir.

## GASI FFA

GASI FFA, sudan su alıp yangına bırakan, pistten ve sudan kalkıp inebilen sabit kanatlı otonom bir uçaktır. Simülasyonda
uçak SolidWorks STEP montajından üretilir ve gerçek arazili dünyalarda uçar.

### Eklenenler

| Konu | Yer |
|---|---|
| İniş flare yasaları 3–6 (`FW_LND_FL_LAW`; 6: kütleden bağımsız, pist ve su için yüzey profilli) | `src/modules/fw_mode_manager`, `src/modules/fw_lateral_longitudinal_control`, `src/lib/fw_performance_model` |
| PX4 içinde su alma: yaklaşma, kapı kontrolü, flare, değme, hız bandında su alma, kalkış ve iptal (`FW_SC_*`) | `src/modules/fw_mode_manager` |
| Su alma görev maddesi `MAV_CMD_USER_1` | `src/modules/navigator`, `src/modules/mavlink/mavlink_mission.cpp` |
| Gazebo eklentileri: iniş takımı sırası, burun tekeri yönlendirme, tekerlek freni | `src/modules/simulation/gz_plugins` |
| Uçak modeli ve airframe | `Tools/simulation/gz_custom_models/gasi_ffa`, `generate_gasi_ffa.py`, `ROMFS/.../airframes/4022_gz_gasi_ffa` |
| Gerçek arazi, su yüzeyi ve su fiziği, pist ve bina üreticileri (alt modül) | `Tools/simulation/gz_geo_terrain` |
| Yangın modeli, yangın kamerası, ısıl kamera | `Tools/simulation/gz_custom_models/wildfire*`, `thermal/` |
| Yangına müdahale görevi, su bölümü, görev planlayıcı | `Tools/simulation/gz_custom_models/scenarios` |

Ayrıntılar: [gz_custom_models/README.md](Tools/simulation/gz_custom_models/README.md) (uçak modeli, senaryolar) ve
[gz_geo_terrain/README.md](https://github.com/CeostechDefance/gz-geo-terrain) (arazi, su, dünyalar).

### Kurulum

```bash
git clone --recursive git@github.com:CeostechDefance/GlobalPilot.git
# klonlandıysa: git submodule update --init --recursive
```

PX4'ün geliştirme ortamı ([Ubuntu kurulumu](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)) ve Gazebo
Harmonic gerekir. Arazi verisinin hazırlanması `gz_geo_terrain` README'sinde anlatılır.

### Simülasyon

```bash
make px4_sitl gz_gasi_ffa_antalya     # Antalya (LTAI)
make px4_sitl gz_gasi_ffa_dalaman     # Dalaman (LTBS), su alma senaryosu bu dünyada
```

`Tools/simulation/gz_geo_terrain/worlds/` altındaki her dünya `gz_gasi_ffa_<dünya>` adıyla açılır (ör. `rothenburg`,
`welzow`).

Yangına müdahale görevi, simülasyon açıkken:

```bash
python3 Tools/simulation/gz_custom_models/scenarios/fire_attack.py --world dalaman --scoop
```

Yangını kamerayla arar, konumunu hesaplar, sudan su alır ve yangına bırakır, sonra piste döner. Su alma varsayılan olarak
görev bilgisayarı tarafında yürütülür; `GASI_SU_PX4=1` ile PX4 içindeki su alma kullanılır.

Senaryo testleri:

```bash
cd Tools/simulation/gz_custom_models/scenarios && python3 -m pytest -q tests
```

## PX4

Uçuş yazılımının temeli PX4'tür: [px4.io](https://px4.io), [kullanıcı ve geliştirici rehberi](https://docs.px4.io/main/en/).
Lisans BSD 3-clause ([LICENSE](LICENSE)).
