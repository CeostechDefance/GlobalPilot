"""Ölçüm arayüzleri. Rehber ve durum makinesi yalnız bu arayüzleri bilir; gerçek uçakta yalnız bu dosya değişir.

YukseklikSensoru.read() -> (gövde altının su/yer üstü yüksekliği m ya da None, ölçüm zamanı)
KutleKaynagi.read()     -> uçağın anlık kütlesi kg ya da None
"""
import threading
import time


class YukseklikSensoru:
    def read(self):
        raise NotImplementedError


class KutleKaynagi:
    def read(self):
        raise NotImplementedError


class SimYukseklik(YukseklikSensoru):
    """Gazebo: modelin gerçek konumu ve su yüzeyi mesh'i (water_leg.Telemetry + WaterSurface.clearance)."""

    def __init__(self, telemetry, surface):
        self.telemetry, self.surface = telemetry, surface

    def read(self):
        snap = self.telemetry.snapshot()
        pose = snap[1]
        if pose is None:
            return None, time.time()
        return self.surface.clearance(pose), time.time()


class SimTankKutlesi(KutleKaynagi):
    """WaterPhysics tankı: /model/<model>/water/mass + boş kütle. Konu gelmiyorsa None (çağıran varsayılanı seçer)."""

    def __init__(self, model, empty_mass_kg):
        from gz.msgs10.double_pb2 import Double
        from gz.transport13 import Node
        self.empty = empty_mass_kg
        self._lock = threading.Lock()
        self._water, self._time = None, 0.0
        self._node = Node()
        self._node.subscribe(Double, f'/model/{model}/water/mass', self._on)

    def _on(self, msg):
        with self._lock:
            self._water, self._time = msg.data, time.time()

    def read(self, max_age_s=2.0):
        with self._lock:
            if self._water is None or time.time() - self._time > max_age_s:
                return None
            return self.empty + self._water
