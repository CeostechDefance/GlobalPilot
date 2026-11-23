# Thermal camera for the fire scenario

**Do not merge yet.** Ground tests on 2026-11-06 found that a transparent OBJ mesh shows in `fire_camera` as opaque white, while one transparent plain box stayed hidden and appeared in the thermal image. `wildfire_heat` now uses plain boxes, which have not been tested as a full model or together with the flame and smoke particles. Terrain is flat in the thermal image. Nothing here is loaded until the steps below are applied.

## Why the parts exist

- `wildfire` is made of particles, and Gazebo thermal cameras do not render particles. `wildfire_heat` adds hot boxes at three temperatures with full transparency, which the thermal camera sees and `fire_camera` should not.
- The `thermal_fire` detector was trained on 8-bit DJI images with automatic gain, not on temperatures. `thermal_frames.render` converts L16 frames to that form.
- The terrain has no temperature of its own. In the ground test it rendered almost flat at about 362 K, with the sky clamped to 253 K. Its image texture does not reach the thermal image.

## Apply

1. Add `gasi_ffa_thermal_sensor.sdf.fragment` to `generate_gasi_ffa.py` directly after `fire_camera` and regenerate `gasi_ffa/model.sdf`.
2. Add `model://wildfire_heat` to `scenarios/antalya_gasi_ffa.sdf` at the same pose as `model://wildfire`, and to the generator in `place_wildfire.py` so that it survives regeneration.

## Test

Run these in order. Each one can fail independently.

1. Load: the sensor and model spawn without errors and the topic publishes `L_INT16` at 640 x 512.
2. RGB camera: `fire_camera` shows no boxes and `find_fire` results match a run without `wildfire_heat`.
3. Thermal image: the fire exceeds 420 K, the terrain stays below it, and the terrain is not flat.
4. Load on the render thread: compare `fire_camera` frame rate and real-time factor with and without the sensor.

Then record a flight and score it:

```
python3 record_thermal.py OUT_DIR
python3 eval_thermal_frames.py OUT_DIR --weights .../fireman/det/runs/thermal_fire/weights/best.pt --out RESULT_DIR --device 0
```

## What a result can and cannot show

A simulation result shows that the chain works from sensor to detection. It does not measure real fire detection or real false alarms, because the terrain has no real thermal behaviour and the fire is a fixed set of boxes.
