# Custom Gazebo models

Models that are not part of the upstream `Tools/simulation/gz` submodule. `px4-rc.gzsim` looks here
when a model is not found there.

## gasi_ffa

GASI autonomous amphibious firefighting aircraft, built from the SolidWorks STEP assembly (`GASI_FFA_STEP_2_8112026.STEP`, 2026-11-08).

```bash
# Antalya (LTAI) runway 36C, nose on the runway heading, Gazebo camera following the aircraft
make px4_sitl gz_gasi_ffa_antalya
# Dalaman (LTBS), north end of runway 19, heading 195.6. Terrain data comes from gz_geo_terrain/regions/dalaman.json.
make px4_sitl gz_gasi_ffa_dalaman

# Regenerate after changing an assumption or the CAD
python3 Tools/simulation/gz_custom_models/generate_gasi_ffa.py /path/to/GASI_FFA.STEP
```

The spawn pose is in `model.sdf`: z = 0.886 puts the wheels on the ground and yaw is 90° minus the
compass heading. On another world, set it with `PX4_GZ_MODEL_POSE=x,y,z,roll,pitch,yaw`. The camera
follow defaults are in the airframe; `PX4_GZ_FOLLOW=` (empty) turns them off.

### What comes from where

| Taken from the CAD | Estimated (top of `generate_gasi_ffa.py`) |
|---|---|
| Outer shape, colours, moving surfaces and hinge lines | Airframe, engine and equipment masses |
| Span 9 m, MAC 1.96 m, tail and control surface areas (wing area 15 m² is from the design data; the CAD plan area is 17.5 m²) | Thrust 5000 N (from 280 hp, 4400 rpm, figure of merit 0.7), drag CD0 0.04, CL0 0.30 |
| Wheel positions, propeller axis | All stability and control derivatives |
| Water 395 L, fuel tank 100 L | Rolling resistance |

The STEP has no masses. The CG along the fuselage is taken from the wing, at 25 % of the MAC (z = -2.53 in the
CAD, where the CAD water body also sits), and the airframe mass is placed to balance there. The estimated masses alone
put it at about 38 % MAC. Static margin 16 % MAC with a full tank, 15 % after the drop.

The derivatives come from tail-volume and strip theory, not from AVL, CFD or flight data. Treat the
flight behaviour as plausible, not as a prediction of the real aircraft.

### Retractable landing gear

PX4 raises the gear after the takeoff climbout and lowers it when the landing starts. It reaches
Gazebo on `servo_4` (output function 400), and `GearSequencerSystem` runs the sequence: nose doors open
(2 s), gear moves (5 s), nose doors close. The main gear wells in the sponsons have no doors.

The STEP holds the gear in one position and has no joints, so the fold is derived, not measured. Each angle below
is the largest at which the tire stays out of the hull skin (z sections of the CAD hull every 1 cm):

- Nose gear: folds straight aft about the cross axis by 75°. The tire clears the well roof; against the closed
  nose doors it overlaps by up to 2.6 cm.
- Main gear: folds forward about the main gear cross bar (`MLG_MAIN_SHAFT`) by 70° into the open well of the
  sponson. The well is only 0.25 m deep, so the tire touches the roof at 70° and its lower 0.11 m stays below the
  sponson. 75° runs it 5 cm into the roof; folding aft does not fit. The struts clear the skin at 70°.
- Nose doors hinge at their outer edges.

Struts and links of the nose gear were not checked, and each gear folds as one rigid part.

### Fire detection camera

Forward camera `fire_camera` inside the nose, behind a transparent window cut from the hull where the
field of view crosses it. 1920x1080 at 15 Hz, 60° horizontal and 36° vertical field of view, tilted 11°
down from the body axis. At 2° cruise pitch the image centre is 9° below the horizon, so the horizon
and rising smoke stay in frame.

Image topic: `/world/<world>/model/gasi_ffa_0/link/base_link/sensor/fire_camera/image`. In the Gazebo
window, add the Image Display plugin and pick this topic.

On terrain worlds the camera is also streamed as RTP/H.264 to UDP port 5600. In QGroundControl, set
Application Settings > General > Video Source to "UDP h.264 Video Stream". To send it to another computer,
start with `PX4_VIDEO_HOST_IP=<address>`; `PX4_GZ_VIDEO_STREAM=0` turns the stream off.

The window and the camera body are not in the image, because they lie closer than the near clip plane.
The camera is an ordinary visible-light sensor. There is no thermal band and no fire in the worlds, so
detection itself cannot be tested yet.

### Test fire

`make px4_sitl gz_gasi_ffa_antalya` also spawns the `wildfire` model from
`scenarios/antalya_gasi_ffa.sdf`, in forest at 36.9759 N, 30.6779 E (about 14 km north-west of the
runway, ground 281 m). To move it, run `python3 place_wildfire.py <lat> <lon>`; delete the scenario file to
fly without it.

It is 60 x 60 m of flames with a dark and a pale smoke layer, made of Gazebo particles. Only the look is
simulated: no heat, no spreading, no wind and no effect from water. The smoke leans a fixed 15° towards east.

### Fire attack scenario

`python3 scenarios/fire_attack.py` flies to a search area without the fire's position, finds the flames in
`fire_camera`, computes their position from the camera ray and the terrain, flies over that point and releases
water. If the curtain's centre falls within 165 m along and 60 m across the track, the flames go out. The real
fire position is used only for this score. The detector is tuned to the simulated textures and does not
generalise to real fires. With `--scoop` it first lands on the water and scoops (`water_leg.py`). The release
empties the tank in the physics as well (`/model/<model>/water/release`).

### Not modelled


- Water inside the hull: the tank (`gasi_ffa/water_tank.txt`: 400 kg, fills at 40 kg/s while the hull is wet between
  22.2 and 27.5 m/s) changes only the weight and the momentum drag, as forces at the tank position; the water does not
  slosh. Hull buoyancy and drag on terrain worlds with water come from the `gz_geo_terrain` plugin, using the hull surface through `gasi_ffa/hull_columns.txt` (draft about 0.50 m at rest at the current mass, about 0.33 m planing at 93 km/h with 3000 N; the hull counts as solid and the pressure and drag coefficients are estimates, not fitted to test data). Regenerate that file with `gz_geo_terrain/tools/make_hull_columns.py` when the body mesh changes.
- Measured gear data: suspension, brake and nose wheel steering values are assumptions.
- Separate fold kinematics of the sway links: each gear folds as one rigid part. Gear loads and door air loads.
- Thrust loss with airspeed: the motor plugin gives the same thrust at any speed.
- Propeller slipstream on the wing and tail.
- Damage. A crash into terrain shows up only in the logs as a high sink rate.
