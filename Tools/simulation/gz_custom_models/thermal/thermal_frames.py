"""Turns Gazebo L16 thermal frames into images in the style thermal_fire was trained on.

The detector never saw temperatures. FireMan-UAV-RGBT holds 8-bit DJI thermal images with automatic gain, in
white-hot grey and in a purple to yellow palette. render() copies that gain: it stretches the scene and lets the
hottest pixels saturate. Frames are 640 x 512, which the detector letterboxes to its 640 pixel input.
"""
import numpy as np

WIDTH, HEIGHT = 640, 512
RESOLUTION_K = 0.05  # keep equal to <resolution> in gasi_ffa_thermal_sensor.sdf.fragment
FRAME_BYTES = WIDTH * HEIGHT * 2
HOT_K = 420.0  # above any terrain temperature, below the coolest band of wildfire_heat
GAIN_LOW_PERCENTILE = 1.0
GAIN_HIGH_PERCENTILE = 99.5
MIN_GAIN_SPAN_K = 2.0  # an empty scene should not stretch noise to full contrast

# Colour anchors in RGB, from cold to hot
IRON_ANCHORS = ((0, 0, 20), (70, 0, 130), (170, 0, 140), (235, 70, 40), (255, 190, 30), (255, 255, 210))
PALETTES = ('white', 'iron')


def decode(data):
    """Returns the frame as kelvin from the little-endian bytes of one L16 frame."""
    pixels = np.frombuffer(data, dtype='<u2', count=WIDTH * HEIGHT).reshape(HEIGHT, WIDTH)
    return pixels.astype(np.float32) * RESOLUTION_K


def hot_box(kelvin):
    """Returns the hot pixel count and their bounding box (x1, y1, x2, y2), which is the ground truth for a frame."""
    ys, xs = np.nonzero(kelvin >= HOT_K)
    if len(xs) == 0:
        return 0, None
    return len(xs), (int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1)


def iron_table():
    positions = np.linspace(0, 255, len(IRON_ANCHORS))
    anchors = np.array(IRON_ANCHORS, dtype=np.float32)
    return np.stack([np.interp(np.arange(256), positions, anchors[:, c]) for c in range(3)], axis=1).astype(np.uint8)


def render(kelvin, palette):
    """Returns an RGB uint8 image with automatic gain over the scene."""
    low, high = np.percentile(kelvin, (GAIN_LOW_PERCENTILE, GAIN_HIGH_PERCENTILE))
    high = max(high, low + MIN_GAIN_SPAN_K)
    grey = (np.clip((kelvin - low) / (high - low), 0, 1) * 255).astype(np.uint8)
    if palette == 'white':
        return np.stack([grey] * 3, axis=2)
    if palette == 'iron':
        return iron_table()[grey]
    raise ValueError(f'unknown palette {palette}')
