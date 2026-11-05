# Custom Gazebo models

Models that are not part of the upstream `Tools/simulation/gz` submodule. `px4-rc.gzsim` looks here
when a model is not found there.

## gasi_ffa

GASI autonomous amphibious firefighting aircraft, built from the SolidWorks STEP assembly.

```bash
# Antalya (LTAI) runway 36C, nose on the runway heading, Gazebo camera following the aircraft
make px4_sitl gz_gasi_ffa_antalya

# Regenerate after changing an assumption or the CAD
python3 Tools/simulation/gz_custom_models/generate_gasi_ffa.py /path/to/GASI_FFA.STEP
```

The spawn pose is in `model.sdf`: z = 0.863 puts the wheels on the ground and yaw is 90° minus the
compass heading. On another world, set it with `PX4_GZ_MODEL_POSE=x,y,z,roll,pitch,yaw`. The camera
follow defaults are in the airframe; `PX4_GZ_FOLLOW=` (empty) turns them off.

### What comes from where

| Taken from the CAD | Estimated (top of `generate_gasi_ffa.py`) |
|---|---|
| Outer shape, colours, moving surfaces and hinge lines | Airframe, engine and equipment masses |
| Wing 17.27 m², span 9 m, MAC 1.94 m, tail areas | Thrust 3000 N, drag CD0 0.04, CL0 0.30 |
| Wheel positions, propeller axis | All stability and control derivatives |
| Water 395 L, fuel tank 100 L | Rolling resistance |

The derivatives come from tail-volume and strip theory, not from AVL, CFD or flight data. Treat the
flight behaviour as plausible, not as a prediction of the real aircraft.

### Retractable landing gear

PX4 raises the gear after the takeoff climbout and lowers it when the landing starts. It reaches
Gazebo on `servo_4` (output function 400), and `GearSequencerSystem` runs the sequence: doors open
(2 s), gear moves (5 s), doors close. The doors stay closed with the gear down, because the main
door's outer edge ends just inboard of the strut.

The STEP holds the gear in one position and has no joints, so the fold is derived, not measured:

- Nose gear: folds straight aft about the cross axis by 73°, where the tire fits between door and well roof.
- Main gear: folds inboard and aft about a diagonal axis by 90°. A numeric search over axes found
  this one puts the tire inside the well. Straight aft or straight inboard does not fit.
- Doors: main doors hinge at the keel, where `MLG_BEAM` runs. Nose doors hinge at their outer edges,
  next to `NLG_BEAM`.

Only the tires were checked against the well. Struts and links were not, and the nose tire overlaps
the placeholder avionics block by about 11 cm.

### Fire detection camera

Forward camera `fire_camera` inside the nose, behind a transparent window cut from the hull where the
field of view crosses it. 1920x1080 at 15 Hz, 60° horizontal and 36° vertical field of view, tilted 11°
down from the body axis. At 2° cruise pitch the image centre is 9° below the horizon, so the horizon
and rising smoke stay in frame.

Image topic: `/world/<world>/model/gasi_ffa_0/link/base_link/sensor/fire_camera/image`. In the Gazebo
window, add the Image Display plugin and pick this topic.

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

### Not modelled


- Water: no scooping, no drop, no change of mass or CG in flight, no water surface or hull buoyancy.
- Wheel brakes and nose wheel steering. Landing gear has no suspension.
- Separate fold kinematics of the sway links: each gear folds as one rigid part. Gear loads and door air loads.
- Thrust loss with airspeed: the motor plugin gives the same thrust at any speed.
- Flaps: the wing surfaces work as ailerons only.
- Damage. A crash into terrain shows up only in the logs as a high sink rate.
