#!/usr/bin/env python3
"""Runs thermal_fire on a recording from record_thermal.py, offline.

The recording supplies its own ground truth: a frame has fire when it holds at least MIN_HOT_PIXELS pixels above
thermal_frames.HOT_K, no fire when it holds none. Frames in between are not scored. The location check asks whether
the centre of the hot pixels lies in a detection box.

Usage: eval_thermal_frames.py RECORDING_DIR --weights best.pt --out RESULT_DIR --device 0
--device has no default so that a run never takes the GPU by accident.
"""
import argparse
import csv
import json
import os

import cv2
import numpy as np

import thermal_frames as tf

THRESHOLD = 0.4  # chosen on real validation data, see fireman/det/results.json
MIN_HOT_PIXELS = 3
SIZE_BINS = ((3, 30), (30, 300), (300, 3000), (3000, tf.WIDTH * tf.HEIGHT + 1))
EXAMPLES_PER_SET = 6


def frames(path, stride):
    with open(path, 'rb') as handle:
        index = 0
        while True:
            data = handle.read(tf.FRAME_BYTES)
            if len(data) < tf.FRAME_BYTES:
                return
            if index % stride == 0:
                yield index, tf.decode(data)
            index += 1


def frame_set(hot_pixels):
    if hot_pixels >= MIN_HOT_PIXELS:
        return 'fire'
    return 'no_fire' if hot_pixels == 0 else 'unclear'


def centre_in_box(hot, boxes):
    cx, cy = (hot[0] + hot[2]) / 2, (hot[1] + hot[3]) / 2
    return any(b[0] <= cx <= b[2] and b[1] <= cy <= b[3] for b in boxes)


def summarize(rows):
    summary = {}
    for palette in tf.PALETTES:
        scored = [r for r in rows if r['palette'] == palette]
        fire = [r for r in scored if r['set'] == 'fire']
        no_fire = [r for r in scored if r['set'] == 'no_fire']
        by_size = {}
        for low, high in SIZE_BINS:
            subset = [r for r in fire if low <= r['hot_pixels'] < high]
            by_size[f'{low}-{high}'] = {'frames': len(subset), 'detected': sum(r['detected'] for r in subset)}
        summary[palette] = {
            'threshold': THRESHOLD,
            'fire_frames': len(fire),
            'fire_detected': sum(r['detected'] for r in fire),
            'fire_location_hit': sum(r['location_hit'] for r in fire),
            'no_fire_frames': len(no_fire),
            'false_alarms': sum(r['detected'] for r in no_fire),
            'unclear_frames': sum(r['set'] == 'unclear' for r in scored),
            'fire_detected_by_hot_pixels': by_size,
        }
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording')
    parser.add_argument('--weights', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--device', required=True, help='for example 0 or cpu')
    parser.add_argument('--stride', type=int, default=1, help='use every Nth frame')
    args = parser.parse_args()

    from ultralytics import YOLO

    os.makedirs(os.path.join(args.out, 'examples'), exist_ok=True)
    model = YOLO(args.weights)
    rows = []
    top = {}
    for index, kelvin in frames(os.path.join(args.recording, 'thermal.raw'), args.stride):
        hot_pixels, hot = tf.hot_box(kelvin)
        set_name = frame_set(hot_pixels)
        for palette in tf.PALETTES:
            bgr = tf.render(kelvin, palette)[:, :, ::-1].copy()  # ultralytics treats numpy input as BGR
            result = model.predict(bgr, imgsz=640, conf=0.05, device=args.device, verbose=False)[0]
            boxes = result.boxes.data.cpu().tolist()
            kept = [b[:4] for b in boxes if b[4] >= THRESHOLD]
            score = max((b[4] for b in boxes), default=0.0)
            rows.append({
                'frame': index, 'set': set_name, 'palette': palette, 'hot_pixels': hot_pixels,
                'min_k': round(float(kelvin.min()), 1), 'max_k': round(float(kelvin.max()), 1),
                'score': round(score, 4), 'detected': score >= THRESHOLD,
                'location_hit': hot is not None and centre_in_box(hot, kept),
            })
            ranking = top.setdefault((palette, set_name), [])
            if len(ranking) < EXAMPLES_PER_SET or score > ranking[-1][0]:
                ranking.append((score, index, result.plot()))
                ranking.sort(key=lambda item: item[0], reverse=True)
                del ranking[EXAMPLES_PER_SET:]
    if not rows:
        raise SystemExit('no frames in the recording')

    for (palette, set_name), ranking in top.items():
        for rank, (score, index, image) in enumerate(ranking):
            name = f'{palette}_{set_name}_{rank}_frame{index}_{score:.2f}.jpg'
            cv2.imwrite(os.path.join(args.out, 'examples', name), image, [cv2.IMWRITE_JPEG_QUALITY, 85])
    with open(os.path.join(args.out, 'frame_results.csv'), 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    summary = summarize(rows)
    with open(os.path.join(args.out, 'summary.json'), 'w') as handle:
        json.dump(summary, handle, indent=1)
    print(json.dumps(summary, indent=1))
    print('temperature range seen:', np.min([r['min_k'] for r in rows]), np.max([r['max_k'] for r in rows]))


if __name__ == '__main__':
    main()
