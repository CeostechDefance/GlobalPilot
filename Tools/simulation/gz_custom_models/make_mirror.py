#!/usr/bin/env python3
"""Bir modelin su yansıması için ayna kopyasını üretir: <model klasörü>/ayna.sdf

Modeldeki bütün bağlantıların görsel parçaları (mesh) ilk duruşlarıyla tek bir bağlantıda toplanır ve model
çerçevesinin z ekseninde aynalanır (konum z → -z, dönme M·R·M, mesh ölçeği z → -z). Kütle neredeyse sıfır, çarpışma
yok, yerçekimi kapalı. Gazebo'daki WaterPhysics bu dosyayı modelin yanında bulursa kopyayı dünyaya ekler ve her adımda
su yüzeyine göre aynalanmış konuma taşır (yalnız görüntü; fiziği etkilemez).

Kullanım: make_mirror.py <model klasörü>
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np


def pose_matrix(text):
    v = [float(x) for x in (text or '0 0 0 0 0 0').split()]
    x, y, z, r, p, yw = v
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(yw), math.sin(yw)
    R = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                  [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                  [-sp, cp * sr, cp * cr]])
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = (x, y, z)
    return T


def rpy(R):
    p = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    if abs(math.cos(p)) > 1e-9:
        return math.atan2(R[2, 1], R[2, 2]), p, math.atan2(R[1, 0], R[0, 0])
    return math.atan2(-R[1, 2], R[1, 1]), p, 0.0


def main(model_dir):
    tree = ET.parse(os.path.join(model_dir, 'model.sdf'))
    model = tree.getroot().find('model')
    M = np.diag([1.0, 1.0, -1.0])
    visuals = []
    for link in model.findall('link'):
        # iniş takımı suya inişte kapalıdır: takım, teker ve kapak parçaları kopyaya girmez
        if any(k in link.get('name', '') for k in ('gear', 'wheel', 'axle', 'steer', 'door')):
            continue
        TL = pose_matrix(link.findtext('pose'))
        for vis in link.findall('visual'):
            mesh = vis.find('geometry/mesh')
            if mesh is None:
                continue
            T = TL @ pose_matrix(vis.findtext('pose'))
            R = M @ T[:3, :3] @ M
            pos = M @ T[:3, 3]
            scale = [float(s) for s in (mesh.findtext('scale') or '1 1 1').split()]
            # z aynası: dönmeyi M·R·M ile, geometriyi mesh ölçeğinin z'si ile çevir. Ölçek mesh'in kendi çerçevesinde
            # uygulandığı için aynanın mesh çerçevesindeki karşılığı R^T·M·R; parçalar çoğunlukla dönmesiz ya da
            # yalnız z çevresinde dönmüş olduğundan z ölçeği yeterli (pervane gibi dönmüş parçada eksen düzeltilir).
            Rm = T[:3, :3].T @ M @ T[:3, :3]
            axis = int(np.argmin(np.diag(Rm)))   # aynanın mesh çerçevesinde ters çevirdiği eksen
            scale[axis] = -scale[axis]
            r, p, y = rpy(R)
            visuals.append((vis.get('name'), mesh.findtext('uri'), pos, (r, p, y), scale))
    body = ''.join(f'''
      <visual name="{name}">
        <pose>{pos[0]:.5f} {pos[1]:.5f} {pos[2]:.5f} {r:.6f} {p:.6f} {y:.6f}</pose>
        <geometry><mesh><uri>{uri}</uri><scale>{s[0]:g} {s[1]:g} {s[2]:g}</scale></mesh></geometry>
        <cast_shadows>false</cast_shadows>
      </visual>''' for name, uri, pos, (r, p, y), s in visuals)
    sdf = f'''<?xml version="1.0"?>
<!-- make_mirror.py çıktısı: {os.path.basename(os.path.abspath(model_dir))} modelinin su yansıması için ayna kopyası (yalnız görüntü) -->
<sdf version="1.9">
  <model name="ayna">
    <link name="link">
      <gravity>false</gravity>
      <kinematic>true</kinematic>
      <inertial><mass>0.001</mass><inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz></inertia></inertial>{body}
    </link>
  </model>
</sdf>
'''
    out = os.path.join(model_dir, 'ayna.sdf')
    with open(out, 'w') as f:
        f.write(sdf)
    print(f'{out}: {len(visuals)} görsel parça')


if __name__ == '__main__':
    main(sys.argv[1])
