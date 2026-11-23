#!/usr/bin/env python3
"""Records the thermal camera to OUT_DIR/thermal.raw (L16 frames) and OUT_DIR/thermal.stamps (simulation seconds).

Stores raw bytes so that recording adds little CPU load during a flight: about 9.8 MB/s at 15 Hz.
Usage: record_thermal.py OUT_DIR [--world antalya] [--model gasi_ffa_0]. Stop with Ctrl+C.
"""
import argparse
import os
import signal
import threading

from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

from thermal_frames import FRAME_BYTES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('out_dir')
    parser.add_argument('--world', default='antalya')
    parser.add_argument('--model', default='gasi_ffa_0')
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    topic = f'/world/{args.world}/model/{args.model}/link/base_link/sensor/thermal_camera/image'
    raw = open(os.path.join(args.out_dir, 'thermal.raw'), 'wb')
    stamps = open(os.path.join(args.out_dir, 'thermal.stamps'), 'w')
    lock = threading.Lock()
    counts = {'frames': 0, 'rejected': 0}

    def on_image(msg):
        if len(msg.data) != FRAME_BYTES:
            counts['rejected'] += 1
            return
        with lock:
            raw.write(msg.data)
            stamps.write(f'{msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9:.3f}\n')
            counts['frames'] += 1

    node = Node()
    if not node.subscribe(Image, topic, on_image):
        raise SystemExit(f'cannot subscribe to {topic}')
    print(f'recording {topic}')
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    stop.wait()
    with lock:
        raw.close()
        stamps.close()
    print(f'{counts["frames"]} frames, {counts["rejected"]} rejected for the wrong size')


if __name__ == '__main__':
    main()
