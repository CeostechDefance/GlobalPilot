"""Durum, komut ve karar kaydı: jsonl, satır başına bir kayıt. Yol çağırandan gelir (simde SSD ekdepo altı)."""
import json
import os
import threading
import time


class Kayit:
    def __init__(self, path, period_s=0.1):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.path = path
        self.period = period_s
        self._f = open(path, 'a', encoding='utf-8')
        self._lock = threading.Lock()
        self._last = 0.0

    def _write(self, row):
        with self._lock:
            self._f.write(json.dumps(row, ensure_ascii=False, default=float) + '\n')
            self._f.flush()

    def sample(self, **row):
        """Periyodik durum satırı; period_s'den sık çağrılırsa atlanır."""
        now = time.time()
        if now - self._last < self.period:
            return
        self._last = now
        self._write(dict(t=round(now, 3), tur='durum', **row))

    def karar(self, text, **row):
        """Aşama geçişi ya da PAS GEÇ: her zaman yazılır."""
        self._write(dict(t=round(time.time(), 3), tur='karar', karar=text, **row))

    def close(self):
        with self._lock:
            self._f.close()
