#!/usr/bin/env python3
"""GASI burun kamerası penceresi: Gazebo görüntüsünü gösterir, alev bulununca çevresine kare çizer.

    python3 Tools/simulation/gz_custom_models/scenarios/fire_camera_view.py

Yalnızca kamera konusunu okur: uçağa komut göndermez, QGroundControl gerektirmez. Görevle aynı alev tespitini
(fire_locator.find_fire_box) kullanır, ama görevin hedef hesabına karışmaz. Esc ya da Q kapatır.
"""
import argparse
import sys
import threading
import time

import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node
from PyQt5.QtCore import QRect, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen
from PyQt5.QtWidgets import QApplication, QWidget

from fire_locator import find_fire_box

TOPIC = '/world/{world}/model/{model}/link/base_link/sensor/fire_camera/image'
REFRESH_MS = 66              # the camera gives 15 frames per second
BOX_HOLD_S = 1.0             # a box older than this is no longer shown
NO_IMAGE_S = 3.0             # no frame for this long counts as lost
BOX_PAD_PX = 12              # on screen, so that a distant fire is still visible
BOX_MIN_PX = 36
DETECT_REST_S = 0.1          # gives the other work on this machine room between detections


class FeedState:
    """Latest frame and latest detection, shared between the transport callback, the detector and the window."""

    def __init__(self):
        self.lock = threading.Lock()
        self.frame = None
        self.frame_time = 0.0
        self.box = None
        self.box_time = 0.0
        self.running = True

    def on_image(self, msg):
        frame = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        with self.lock:
            self.frame = frame
            self.frame_time = time.time()

    def detect_loop(self):
        checked = None
        while self.running:
            with self.lock:
                frame = self.frame
            if frame is None or frame is checked:
                time.sleep(0.02)
                continue
            checked = frame
            found = find_fire_box(frame)
            if found:
                with self.lock:
                    self.box = found[3]
                    self.box_time = time.time()
            time.sleep(DETECT_REST_S)


class CameraWindow(QWidget):
    def __init__(self, state):
        super().__init__()
        self.state = state
        self.setWindowTitle('GASI Burun Kamerası')
        self.resize(1280, 720)
        self.setStyleSheet('background-color: black;')
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(REFRESH_MS)

    def keyPressEvent(self, event):
        if event.key() in (Qt.Key_Escape, Qt.Key_Q):
            self.close()

    def closeEvent(self, event):
        self.state.running = False
        super().closeEvent(event)

    def paintEvent(self, event):
        with self.state.lock:
            frame = self.state.frame
            frame_time = self.state.frame_time
            box = self.state.box if time.time() - self.state.box_time < BOX_HOLD_S else None
        painter = QPainter(self)
        painter.fillRect(self.rect(), Qt.black)
        painter.setFont(QFont('Sans', 14, QFont.Bold))
        if frame is None:
            self.draw_label(painter, 'Kamera bekleniyor...', QColor(255, 200, 0), 16, 16)
            return
        height, width = frame.shape[:2]
        scale = min(self.width() / width, self.height() / height)
        left = (self.width() - width * scale) / 2
        top = (self.height() - height * scale) / 2
        image = QImage(frame.data, width, height, 3 * width, QImage.Format_RGB888)
        painter.setRenderHint(QPainter.SmoothPixmapTransform)
        painter.drawImage(QRect(int(left), int(top), int(width * scale), int(height * scale)), image)
        if box:
            self.draw_box(painter, box, scale, left, top)
            self.draw_label(painter, 'YANGIN BULUNDU', QColor(230, 30, 30), left + 16, top + 16)
        else:
            self.draw_label(painter, 'Alev aranıyor', QColor(40, 40, 40), left + 16, top + 16)
        if time.time() - frame_time > NO_IMAGE_S:
            self.draw_label(painter, 'Görüntü kesildi', QColor(255, 140, 0), left + 16, top + 60)

    @staticmethod
    def draw_box(painter, box, scale, left, top):
        x1, y1 = box[0] * scale + left - BOX_PAD_PX, box[1] * scale + top - BOX_PAD_PX
        x2, y2 = box[2] * scale + left + BOX_PAD_PX, box[3] * scale + top + BOX_PAD_PX
        center_x, center_y = (x1 + x2) / 2, (y1 + y2) / 2
        half_w, half_h = max((x2 - x1) / 2, BOX_MIN_PX / 2), max((y2 - y1) / 2, BOX_MIN_PX / 2)
        painter.setPen(QPen(QColor(255, 40, 40), 3))
        painter.setBrush(Qt.NoBrush)
        painter.drawRect(int(center_x - half_w), int(center_y - half_h), int(2 * half_w), int(2 * half_h))

    @staticmethod
    def draw_label(painter, text, color, x, y):
        metrics = painter.fontMetrics()
        area = QRect(int(x), int(y), metrics.horizontalAdvance(text) + 20, metrics.height() + 12)
        painter.fillRect(area, color)
        painter.setPen(Qt.white)
        painter.drawText(area, Qt.AlignCenter, text)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', default='antalya')
    ap.add_argument('--model', default='gasi_ffa_0')
    args = ap.parse_args()

    state = FeedState()
    node = Node()
    if not node.subscribe(Image, TOPIC.format(world=args.world, model=args.model), state.on_image):
        sys.exit('Kamera konusuna abone olunamadı')
    threading.Thread(target=state.detect_loop, daemon=True).start()

    app = QApplication(sys.argv[:1])
    window = CameraWindow(state)
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
