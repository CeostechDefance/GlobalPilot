"""Finds the simulated fire in a camera frame and turns its pixel into a position on the terrain.

The detector is tuned to the simulated flame and smoke textures. It shows the geolocation chain working
and does not generalise to real fires.
"""
import math
import os
import sys

import numpy as np
from scipy import ndimage

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from place_wildfire import data_root_dir, ground_height  # noqa: E402

CAMERA_HFOV_DEG = 60.0
CAMERA_TILT_DEG = 11.0  # down from the body axis; keep equal to generate_gasi_ffa.py
RAY_STEP_M = 15.0
RAY_MAX_M = 15000.0


def find_fire(rgb):
    """Returns the (u, v) pixel at the base of the fire and its size in pixels, or None."""
    found = find_fire_box(rgb)
    return found[:3] if found else None


def find_fire_box(rgb):
    """Returns (u, v, size, (left, top, right, bottom)): the base pixel, the size and the flame's bounding box, or None."""
    r, g, b = (rgb[..., i].astype(np.int16) for i in range(3))
    # Flames have far less blue than the bright red soil in the imagery: blue under 100 against over 100 there.
    flame = (r > 205) & (b < 105) & (g - b > 55) & (r - g > 55)
    if not flame.any():   # aramada karelerin çoğu: duman maskesi ve etiketleme boşuna olmasın
        return None
    smoke = (r < 80) & (g < 80) & (b < 80) & (np.abs(r - g) < 10) & (np.abs(g - b) < 10)
    labels, count = ndimage.label(ndimage.binary_dilation(flame, iterations=4))
    best = None
    for index, region in enumerate(ndimage.find_objects(labels), start=1):
        mask = (labels[region] == index) & flame[region]
        area = int(mask.sum())
        if area < 3:
            continue
        rows, cols = region
        # A fire has a dense dark smoke column right above it.
        reach = max(40, 5 * (rows.stop - rows.start))
        pad = 10
        above = smoke[max(rows.start - reach, 0):rows.start + 5, max(cols.start - pad, 0):cols.stop + pad]
        if above.size == 0 or above.sum() < 30 or above.mean() < 0.12:
            continue
        if best is None or area > best[0]:
            ys, xs = np.nonzero(mask)
            low = ys >= np.percentile(ys, 75)
            box = (int(xs.min() + cols.start), int(ys.min() + rows.start), int(xs.max() + cols.start), int(ys.max() + rows.start))
            best = (area, float(xs[low].mean() + cols.start), float(ys[low].mean() + rows.start), box)
    if best is None:
        return None
    return best[1], best[2], best[0], best[3]


def pixel_to_ned(u, v, width, height, roll, pitch, yaw):
    """Unit ray of a pixel in north-east-down axes. Angles in radians, as in MAVLink ATTITUDE."""
    f = (width / 2) / math.tan(math.radians(CAMERA_HFOV_DEG) / 2)
    right, down = (u - width / 2) / f, (v - height / 2) / f
    t = math.radians(CAMERA_TILT_DEG)
    body = np.array([math.cos(t) - down * math.sin(t), right, math.sin(t) + down * math.cos(t)])
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    dcm = np.array([[cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
                    [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
                    [-sp, sr * cp, cr * cp]])
    ray = dcm @ body
    return ray / np.linalg.norm(ray)


def ray_to_ground(lat, lon, alt_amsl, ray, region='antalya', data_root=None):
    """Marches the ray until it goes below the terrain. Returns (lat, lon, ground height, range) or None."""
    if ray[2] <= 0.002:
        return None
    root = data_root or data_root_dir()
    cos_lat = math.cos(math.radians(lat))
    s = RAY_STEP_M
    while s < RAY_MAX_M:
        la = lat + ray[0] * s / 111320.0
        lo = lon + ray[1] * s / (111320.0 * cos_lat)
        z = alt_amsl - ray[2] * s
        try:
            ground = ground_height(root, region, la, lo)
        except (OSError, SystemExit):
            return None
        if z <= ground:   # first step under the terrain (RAY_STEP_M resolution, no refinement)
            return la, lo, ground, s
        s += RAY_STEP_M
    return None
