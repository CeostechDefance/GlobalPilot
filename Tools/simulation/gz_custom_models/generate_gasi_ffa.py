#!/usr/bin/env python3
"""Builds the gasi_ffa Gazebo model (meshes + model.sdf) from the SolidWorks STEP assembly.

    python3 generate_gasi_ffa.py /path/to/GASI_FFA_STEP_21102026.STEP

Needs OCP (cadquery-ocp), trimesh and numpy. Geometry comes from the CAD. The STEP has no
mass, power or aerodynamic data, so everything under "Assumptions" is an engineering estimate:
replace those values with measured ones and rerun.

Frames: CAD has nose +Z, up +Y, left wing +X. SDF has X forward, Y left, Z up, so
X_sdf = Z_cad, Y_sdf = X_cad, Z_sdf = Y_cad. base_link sits on the estimated CG.
"""
import argparse
import math
import os
import tempfile

import numpy as np
import trimesh
from trimesh.visual.material import PBRMaterial

# --- Assumptions (not in the STEP) -----------------------------------------------------------
WATER_LITERS = 395.1       # full tank; CAD water volume. 0 = after the drop (see static margin output)
FUEL_KG = 72.0             # 100 L at 0.72 kg/L
AIRFRAME_KG = 207.0        # chosen so full-load mass matches the parachute class limit (990 kg)
THRUST_MAX_N = 3000.0      # about 100 kW on the 1.76 m disc, figure of merit 0.7
CL0 = 0.30                 # at body-level attitude, zero flap
CD0 = 0.040                # hull and fixed gear
OSWALD = 0.80
TAIL_LIFT_SLOPE = 3.4      # per rad, finite horizontal tail
FIN_LIFT_SLOPE = 2.8       # per rad, finite fin
FLAP_TAU = 0.55            # surface effectiveness for flaperons and rudder
ELEV_TAU = 0.50
DOWNWASH = 0.35            # d(epsilon)/d(alpha) at the tail

# name -> (kg, CAD node, is a separate link); position is taken from the CAD part's bbox centre
COMPONENTS = {
    'engine': (110.0, 'NAUO2'),
    'engine_accessories': (30.0, 'NAUO6'),
    'avionics': (40.0, 'NAUO3'),
    'lights_camera': (3.0, 'NAUO4'),
    'battery': (8.0, 'NAUO5'),
    'parachute': (15.0, 'NAUO7'),
    'fuel': (FUEL_KG, 'NAUO8'),
    'water_system_dry': (35.0, 'NAUO103'),
    'water': (WATER_LITERS * 0.998, 'NAUO104'),
    'main_gear': (45.0, 'NAUO53'),
}
NOSE_GEAR_KG = 15.0
MAIN_STRUT_KG = 12.5
NOSE_STRUT_KG = 5.0
DOOR_KG = 1.0
PROP_KG = 15.0
WHEEL_KG = 10.0
WHEEL_R = 0.175
WHEEL_FRICTION_NM = 15.0   # rolling resistance of about 3 %; there are no brakes

# --- Geometry taken from the CAD --------------------------------------------------------------
NOSE_GEAR_NODES = {f'NAUO{i}' for i in range(24, 53)}  # strut, links, hubs, wheel and fasteners
BODY_SKIP = {'NAUO2', 'NAUO3', 'NAUO4', 'NAUO5', 'NAUO6', 'NAUO7', 'NAUO8', 'NAUO22', 'NAUO53',
             'NAUO14', 'NAUO15', 'NAUO18', 'NAUO19', *NOSE_GEAR_NODES,
             'NAUO9', 'NAUO10', 'NAUO11', 'NAUO12', 'NAUO13', 'NAUO54', 'NAUO55'}
PROP_NODES = {'NAUO9', 'NAUO10'}
GROUND_Y = -0.75           # tire contact height in the CAD frame
SPAWN_YAW_RAD = 1.4835     # nose on Antalya runway 36C (heading 005); PX4_GZ_MODEL_POSE overrides it
PROP_AXIS = (1.02, -4.225)  # CAD y, z of the propeller axis and disc plane
SURFACES = {
    # link: (CAD nodes, hinge end A, hinge end B in the CAD frame)
    'left_flap': ({'NAUO54'}, (0.9, 0.631, -3.262), (4.08, 0.798, -3.449)),
    'right_flap': ({'NAUO55'}, (-0.9, 0.631, -3.262), (-4.08, 0.798, -3.449)),
    'left_elevator': ({'NAUO13'}, (0.265, 1.8, -6.851), (1.355, 1.8, -6.859)),
    'right_elevator': ({'NAUO12'}, (-0.265, 1.8, -6.851), (-1.355, 1.797, -6.859)),
    'rudder': ({'NAUO11'}, (0.0, 1.629, -6.863), (0.0, 0.39, -6.493)),
}
# Retraction. The STEP has no kinematics, so the pivots and axes below are derived from the CAD
# parts and the well they must fit in (see README). A positive angle always means gear up / door
# open. A door axis runs from `a` to `b`, so its direction sets which way that door swings.
GEAR = {
    # link: (CAD nodes, pivot, axis, travel in degrees)
    'left_main_gear': ({'NAUO58'}, (0.56, -0.07, -3.39), (0.785, 0.5, -0.366), 90.0),
    'right_main_gear': ({'NAUO57'}, (-0.56, -0.07, -3.39), (0.785, -0.5, 0.366), 90.0),
    'nose_gear': (NOSE_GEAR_NODES, (0.0, -0.025, -0.60), (1.0, 0.0, 0.0), 73.0),
}
DOORS = {
    # link: (CAD nodes, hinge a, hinge b, travel in degrees)
    'left_main_door': ({'NAUO18'}, (0.0, -0.48, -3.15), (0.0, -0.455, -4.0), 105.0),
    'right_main_door': ({'NAUO19'}, (0.0, -0.455, -4.0), (0.0, -0.48, -3.15), 105.0),
    'left_nose_door': ({'NAUO14'}, (0.14, -0.352, -1.307), (0.14, -0.202, -0.597), 80.0),
    'right_nose_door': ({'NAUO15'}, (-0.14, -0.202, -0.597), (-0.14, -0.352, -1.307), 80.0),
}
GEAR_TIME_S = 5.0
DOOR_TIME_S = 2.0
GEAR_DRAG_CD = 0.012       # part of CD0 that disappears with the gear up

# Fire detection camera, inside the nose at the front of the CAD LIGHTS_CAMERA block.
CAMERA_POS = (0.0, 0.012, -0.16)  # CAD frame
CAMERA_TILT_DEG = 11.0     # down from the body axis: 9 deg below the horizon at 2 deg cruise pitch
CAMERA_HFOV_DEG = 60.0
CAMERA_SIZE = (1920, 1080)
CAMERA_RATE_HZ = 15
WINDOW_MARGIN_DEG = 6.0    # how far the transparent nose extends past the camera field of view

# Planform values measured by projecting CAD triangles (see README).
WING = dict(S=17.27, b=9.0, mac=1.944, mac_le_z=-2.06)
HTAIL = dict(S=2.17, z=-6.81, elev_S=0.655)
VTAIL = dict(S=1.54, z=-6.52, rudder_S=0.523, y=1.1)
FLAP = dict(S=2.05, y=2.5, z=-3.50)
RHO = 1.2041
TO_SDF = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], dtype=float)


def step_to_glb(step_path, glb_path, deflection=5.0, angle=0.5):
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.Message import Message_ProgressRange
    from OCP.RWGltf import RWGltf_CafWriter, RWGltf_WriterTrsfFormat
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_AsciiString, TCollection_ExtendedString
    from OCP.TColStd import TColStd_IndexedDataMapOfStringString
    from OCP.TDF import TDF_LabelSequence
    from OCP.TDocStd import TDocStd_Document
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    doc = TDocStd_Document(TCollection_ExtendedString("XCAF"))
    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.SetNameMode(True)
    reader.ReadFile(step_path)
    reader.Transfer(doc)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    BRepMesh_IncrementalMesh(tool.GetShape_s(roots.Value(1)), deflection, False, angle, True)
    writer = RWGltf_CafWriter(TCollection_AsciiString(glb_path), True)
    writer.SetMergeFaces(True)
    writer.SetTransformationFormat(RWGltf_WriterTrsfFormat.RWGltf_WriterTrsfFormat_Compact)
    if not writer.Perform(doc, TColStd_IndexedDataMapOfStringString(), Message_ProgressRange()):
        raise RuntimeError('glTF export failed')


class Cad:
    """Triangle meshes of the assembly, addressed by top-level instance node (NAUOn)."""

    def __init__(self, glb_path):
        self.scene = trimesh.load(glb_path, force='scene')
        self.parts = {}
        parents = self.scene.graph.transforms.parents
        for node in self.scene.graph.nodes_geometry:
            transform, name = self.scene.graph[node]
            mesh = self.scene.geometry[name].copy()
            mesh.apply_transform(transform)
            chain = [node]
            while chain[-1] in parents:
                chain.append(parents[chain[-1]])
            for key in chain:
                self.parts.setdefault(key.split('_')[0], []).append(mesh)

    def vertices(self, key):
        return np.vstack([m.vertices for m in self.parts[key]])

    def centre(self, key):
        v = self.vertices(key)
        return (v.min(0) + v.max(0)) / 2

    def size(self, key):
        v = self.vertices(key)
        return v.max(0) - v.min(0)


def export_group(cad, keys, origin_cad, path):
    """Writes the parts as one GLB in SDF axes, with origin_cad at the mesh origin."""
    scene = trimesh.Scene()
    seen = set()
    rot = np.eye(4)
    rot[:3, :3] = TO_SDF
    shift = np.eye(4)
    shift[:3, 3] = -np.asarray(origin_cad)
    for key in sorted(keys):
        for mesh in cad.parts[key]:
            if id(mesh) in seen:
                continue
            seen.add(id(mesh))
            out = mesh.copy()
            out.apply_transform(rot @ shift)
            color = getattr(getattr(out.visual, 'material', None), 'baseColorFactor', None)
            if color is None:
                color = [0.6, 0.6, 0.6, 1.0]
            # CAD colours carry no metalness and glTF would read that as full metal.
            out.visual = trimesh.visual.TextureVisuals(material=PBRMaterial(
                baseColorFactor=color, metallicFactor=0.0, roughnessFactor=0.55))
            scene.add_geometry(out, geom_name=f'{key}_{len(seen)}')
    scene.export(path)
    return sum(len(g.faces) for g in scene.geometry.values())


def split_nose_window(cad):
    """Cuts the part of the hull in front of the camera into its own mesh for the transparent window.

    Returns (hull meshes without the window, window mesh). The nose is subdivided first so that the
    window edge is smooth instead of following the CAD triangles.
    """
    tilt = math.radians(CAMERA_TILT_DEG)
    forward = np.array([0.0, -math.sin(tilt), math.cos(tilt)])
    right = np.array([-1.0, 0.0, 0.0])
    up = np.cross(right, forward)
    half_h = math.tan(math.radians(CAMERA_HFOV_DEG / 2))
    half_v = half_h * CAMERA_SIZE[1] / CAMERA_SIZE[0]
    limit_h = math.tan(math.atan(half_h) + math.radians(WINDOW_MARGIN_DEG))
    limit_v = math.tan(math.atan(half_v) + math.radians(WINDOW_MARGIN_DEG))
    camera = np.asarray(CAMERA_POS)
    hull, window_parts = [], []
    for mesh in cad.parts['NAUO1']:
        # Only triangles around the window edge get subdivided, to keep the mesh small.
        d = mesh.triangles_center - camera
        f = d @ forward
        near_nose = (f > 0) & (np.abs(d @ right) < 1.4 * limit_h * f) & (np.abs(d @ up) < 1.4 * limit_v * f)
        if not near_nose.any():
            hull.append(mesh)
            continue
        vertices, faces = trimesh.remesh.subdivide_to_size(mesh.vertices, mesh.faces[near_nose], max_edge=0.01)
        nose = trimesh.Trimesh(vertices, faces, process=False)
        d = nose.triangles_center - camera
        f = d @ forward
        in_view = (f > 0) & (np.abs(d @ right) < limit_h * f) & (np.abs(d @ up) < limit_v * f)
        rest = [trimesh.Trimesh(vertices, faces[~in_view], process=False)] if (~in_view).any() else []
        if (~near_nose).any():
            rest.append(trimesh.Trimesh(mesh.vertices, mesh.faces[~near_nose], process=False))
        for part in rest:
            part.visual = trimesh.visual.TextureVisuals(material=mesh.visual.material)
            hull.append(part)
        if in_view.any():
            window_parts.append(trimesh.Trimesh(vertices, faces[in_view], process=False))
    return hull, trimesh.util.concatenate(window_parts)


def to_sdf(p):
    return TO_SDF @ np.asarray(p, dtype=float)


def mass_properties(cad):
    """CG and inertia about the CG, in SDF axes, from point masses plus a CAD skin shell for the airframe."""
    points = []
    for name, (kg, key) in COMPONENTS.items():
        if kg > 0:
            points.append((name, kg, cad.centre(key), cad.size(key)))
    points.append(('nose_gear', NOSE_GEAR_KG, cad.centre('NAUO30'), cad.size('NAUO30')))
    points.append(('prop', PROP_KG, np.array([0.0, *PROP_AXIS]), np.array([1.2, 1.2, 0.1])))

    skin = np.vstack([np.asarray(m.triangles) for m in cad.parts['NAUO1']])
    area = np.linalg.norm(np.cross(skin[:, 1] - skin[:, 0], skin[:, 2] - skin[:, 0]), axis=1) / 2
    centers = skin.mean(1)
    centroid = (area[:, None] * centers).sum(0) / area.sum()
    spread = np.sqrt((area[:, None] * (centers - centroid) ** 2).sum(0) / area.sum())
    total = AIRFRAME_KG + sum(p[1] for p in points)
    cg = (AIRFRAME_KG * centroid + sum(kg * c for _, kg, c, _ in points)) / total

    inertia = np.zeros((3, 3))

    def add(kg, c, diag_second_moment):
        nonlocal inertia
        d = to_sdf(c - cg)
        s = TO_SDF @ np.diag(diag_second_moment) @ TO_SDF.T
        inertia += kg * ((d @ d + np.trace(s)) * np.eye(3) - np.outer(d, d) - s)

    add(AIRFRAME_KG, centroid, spread ** 2)
    for _, kg, c, size in points:
        add(kg, c, size ** 2 / 12)
    return total, cg, inertia


def aero(cg):
    """Stability and control derivatives about the CG from CAD planforms (strip and tail-volume theory)."""
    S, b, c = WING['S'], WING['b'], WING['mac']
    ar = b * b / S
    a_wing = 2 * math.pi * ar / (2 + math.sqrt(4 + ar * ar))
    ac_z = WING['mac_le_z'] - 0.25 * c
    arm_h = ac_z - HTAIL['z']
    vh = HTAIL['S'] * arm_h / (S * c)
    arm_v = cg[2] - VTAIL['z']
    vv = VTAIL['S'] * arm_v / (S * b)
    cla = a_wing + TAIL_LIFT_SLOPE * HTAIL['S'] / S * (1 - DOWNWASH)
    np_frac = 0.25 + vh * TAIL_LIFT_SLOPE / a_wing * (1 - DOWNWASH)
    np_z = WING['mac_le_z'] - np_frac * c
    margin = (cg[2] - np_z) / c
    cma = -cla * margin

    q_cruise = 0.5 * RHO * 45.0 ** 2
    drag_cruise = q_cruise * S * (CD0 + (990 * 9.81 / (q_cruise * S)) ** 2 / (math.pi * ar * OSWALD))
    thrust_arm = PROP_AXIS[0] - cg[1]
    alpha_cruise = (990 * 9.81 / (q_cruise * S) - CL0) / cla
    cm0 = -cma * alpha_cruise + drag_cruise * thrust_arm / (q_cruise * S * c)

    flap_cl = FLAP['S'] * a_wing * FLAP_TAU / S
    flap_roll = flap_cl * FLAP['y'] / b
    flap_pitch = flap_cl * (cg[2] - FLAP['z']) / c
    elev_cl = TAIL_LIFT_SLOPE * HTAIL['S'] / S * ELEV_TAU
    elev_cm = TAIL_LIFT_SLOPE * vh * ELEV_TAU
    rud_cy = VTAIL['rudder_S'] * FIN_LIFT_SLOPE * FLAP_TAU / S
    rud_cn = rud_cy * arm_v / b
    d = math.pi / 180  # the plugin takes control derivatives per degree
    return dict(
        AR=ar, cla=cla, vh=vh, vv=vv, margin=margin, np_z=np_z, ac_z=ac_z, cm0=cm0, cma=cma,
        drag_cruise=drag_cruise, thrust_arm=thrust_arm,
        cmq=-2 * TAIL_LIFT_SLOPE * vh * arm_h / c - 0.5, clq=2 * TAIL_LIFT_SLOPE * vh,
        cnb=vv * FIN_LIFT_SLOPE - 0.04, cyb=-VTAIL['S'] / S * FIN_LIFT_SLOPE - 0.05,
        cnr=-2 * vv * FIN_LIFT_SLOPE * arm_v / b - 0.02, cyr=2 * vv * FIN_LIFT_SLOPE,
        flap_cl=flap_cl * d, flap_roll=flap_roll * d, flap_pitch=flap_pitch * d,
        elev_cl=elev_cl * d, elev_cm=elev_cm * d,
        rud_cy=rud_cy * d, rud_cn=rud_cn * d, rud_cl=rud_cy * VTAIL['y'] / b * d,
    )


SENSORS = """
      <sensor name="imu_sensor" type="imu">
        <gz_frame_id>base_link</gz_frame_id>
        <always_on>1</always_on>
        <update_rate>250</update_rate>
        <imu>
          <angular_velocity>
            <x><noise type="gaussian"><mean>0</mean><stddev>0.0003394</stddev><dynamic_bias_stddev>3.8785e-05</dynamic_bias_stddev><dynamic_bias_correlation_time>1000</dynamic_bias_correlation_time></noise></x>
            <y><noise type="gaussian"><mean>0</mean><stddev>0.0003394</stddev><dynamic_bias_stddev>3.8785e-05</dynamic_bias_stddev><dynamic_bias_correlation_time>1000</dynamic_bias_correlation_time></noise></y>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.0003394</stddev><dynamic_bias_stddev>3.8785e-05</dynamic_bias_stddev><dynamic_bias_correlation_time>1000</dynamic_bias_correlation_time></noise></z>
          </angular_velocity>
          <linear_acceleration>
            <x><noise type="gaussian"><mean>0</mean><stddev>0.004</stddev><dynamic_bias_stddev>0.006</dynamic_bias_stddev><dynamic_bias_correlation_time>300</dynamic_bias_correlation_time></noise></x>
            <y><noise type="gaussian"><mean>0</mean><stddev>0.004</stddev><dynamic_bias_stddev>0.006</dynamic_bias_stddev><dynamic_bias_correlation_time>300</dynamic_bias_correlation_time></noise></y>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.004</stddev><dynamic_bias_stddev>0.006</dynamic_bias_stddev><dynamic_bias_correlation_time>300</dynamic_bias_correlation_time></noise></z>
          </linear_acceleration>
        </imu>
      </sensor>
      <sensor name="navsat_sensor" type="navsat">
        <gz_frame_id>base_link</gz_frame_id>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
      </sensor>
      <sensor name="air_pressure_sensor" type="air_pressure">
        <gz_frame_id>base_link</gz_frame_id>
        <always_on>1</always_on>
        <update_rate>50</update_rate>
        <air_pressure><pressure><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></pressure></air_pressure>
      </sensor>
      <sensor name="magnetometer_sensor" type="magnetometer">
        <gz_frame_id>base_link</gz_frame_id>
        <always_on>1</always_on>
        <update_rate>100</update_rate>
        <magnetometer>
          <x><noise type="gaussian"><stddev>0.0001</stddev></noise></x>
          <y><noise type="gaussian"><stddev>0.0001</stddev></noise></y>
          <z><noise type="gaussian"><stddev>0.0001</stddev></noise></z>
        </magnetometer>
      </sensor>"""


def vec(v):
    return ' '.join(f'{x:.5f}' for x in v)


def box_collision(name, center, size):
    return f"""
      <collision name="{name}">
        <pose>{vec(center)} 0 0 0</pose>
        <geometry><box><size>{vec(size)}</size></box></geometry>
      </collision>"""


def surface_link(name, hinge_a, hinge_b, cg, joint, travel, topic):
    a, b = to_sdf(hinge_a), to_sdf(hinge_b)
    mid = (a + b) / 2
    axis = (b - a) / np.linalg.norm(b - a)
    # Positive joint angle must mean trailing edge up (wing, tail) or right (rudder) on every
    # surface, so the axis always points along +Y or +Z whichever side the surface is on.
    if axis[np.argmax(np.abs(axis))] < 0:
        axis = -axis
    pose = mid - to_sdf(cg)
    return f"""
    <link name="{name}">
      <pose>{vec(pose)} 0 0 0</pose>
      <inertial><mass>1.0</mass><inertia><ixx>0.05</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.05</iyy><iyz>0</iyz><izz>0.05</izz></inertia></inertial>
      <visual name="{name}_visual">
        <geometry><mesh><uri>model://gasi_ffa/meshes/{name}.glb</uri></mesh></geometry>
      </visual>
    </link>
    <joint name="{joint}" type="revolute">
      <parent>base_link</parent>
      <child>{name}</child>
      <axis>
        <xyz expressed_in="__model__">{vec(axis)}</xyz>
        <limit><lower>{-travel:.4f}</lower><upper>{travel:.4f}</upper></limit>
        <dynamics><damping>1.0</damping></dynamics>
      </axis>
    </joint>
    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>{joint}</joint_name>
      <sub_topic>{topic}</sub_topic>
      <p_gain>10</p_gain>
      <i_gain>0</i_gain>
      <d_gain>0</d_gain>
    </plugin>"""


def gear_link(name, nodes_pivot, axis, travel, cg, mass, length):
    pose = to_sdf(np.asarray(nodes_pivot)) - to_sdf(cg)
    direction = to_sdf(axis)
    direction = direction / np.linalg.norm(direction)
    i = mass * length ** 2 / 12
    return f"""
    <link name="{name}">
      <pose>{vec(pose)} 0 0 0</pose>
      <inertial>
        <pose>0 0 {-length / 2:.3f} 0 0 0</pose>
        <mass>{mass}</mass>
        <inertia><ixx>{i:.4f}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{i:.4f}</iyy><iyz>0</iyz><izz>{0.1 * i:.4f}</izz></inertia>
      </inertial>
      <visual name="{name}_visual">
        <geometry><mesh><uri>model://gasi_ffa/meshes/{name}.glb</uri></mesh></geometry>
      </visual>
    </link>
    <joint name="{name}_joint" type="revolute">
      <parent>base_link</parent>
      <child>{name}</child>
      <axis>
        <xyz expressed_in="__model__">{vec(direction)}</xyz>
        <limit><lower>-0.02</lower><upper>{travel + 0.02:.4f}</upper></limit>
      </axis>
    </joint>"""


def door_link(name, hinge_a, hinge_b, travel, cg):
    a, b = to_sdf(hinge_a), to_sdf(hinge_b)
    pose = (a + b) / 2 - to_sdf(cg)
    direction = (b - a) / np.linalg.norm(b - a)
    return f"""
    <link name="{name}">
      <pose>{vec(pose)} 0 0 0</pose>
      <inertial><mass>{DOOR_KG}</mass><inertia><ixx>0.05</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.05</iyy><iyz>0</iyz><izz>0.05</izz></inertia></inertial>
      <visual name="{name}_visual">
        <geometry><mesh><uri>model://gasi_ffa/meshes/{name}.glb</uri></mesh></geometry>
      </visual>
    </link>
    <joint name="{name}_joint" type="revolute">
      <parent>base_link</parent>
      <child>{name}</child>
      <axis>
        <xyz expressed_in="__model__">{vec(direction)}</xyz>
        <limit><lower>-0.02</lower><upper>{travel + 0.02:.4f}</upper></limit>
      </axis>
    </joint>"""


def wheel_link(name, center_sdf, parent):
    i = 0.4 * WHEEL_KG * WHEEL_R ** 2
    return f"""
    <link name="{name}">
      <pose>{vec(center_sdf)} 0 0 0</pose>
      <inertial><mass>{WHEEL_KG}</mass><inertia><ixx>{i:.5f}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{i:.5f}</iyy><iyz>0</iyz><izz>{i:.5f}</izz></inertia></inertial>
      <collision name="{name}_collision">
        <geometry><sphere><radius>{WHEEL_R}</radius></sphere></geometry>
        <surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface>
      </collision>
    </link>
    <joint name="{name}_joint" type="revolute">
      <parent>{parent}</parent>
      <child>{name}</child>
      <axis><xyz>0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit><dynamics><friction>{WHEEL_FRICTION_NM}</friction></dynamics></axis>
    </joint>"""


def control_surface(joint, index, direction, cd, cy, cl, cell, cem, cen):
    return f"""
      <control_surface>
        <name>{joint}</name><index>{index}</index><direction>{direction}</direction>
        <CD_ctrl>{cd:.7f}</CD_ctrl><CY_ctrl>{cy:.7f}</CY_ctrl><CL_ctrl>{cl:.7f}</CL_ctrl>
        <Cell_ctrl>{cell:.7f}</Cell_ctrl><Cem_ctrl>{cem:.7f}</Cem_ctrl><Cen_ctrl>{cen:.7f}</Cen_ctrl>
      </control_surface>"""


def build_sdf(cad, out_dir, mass_data):
    total, cg, inertia = mass_data
    a = aero(cg)
    cg_sdf = to_sdf(cg)
    moving_kg = PROP_KG + 3 * WHEEL_KG + 2 * MAIN_STRUT_KG + NOSE_STRUT_KG + 5 * 1.0 + 4 * DOOR_KG
    base_kg = total - moving_kg

    wheels = {
        'nose_wheel': np.array([0.0, GROUND_Y + WHEEL_R, cad.centre('NAUO30')[2]]),
        'left_wheel': np.array([0.685, GROUND_Y + WHEEL_R, cad.centre('NAUO53')[2]]),
        'right_wheel': np.array([-0.685, GROUND_Y + WHEEL_R, cad.centre('NAUO53')[2]]),
    }
    links = ''
    sequencer = ''
    for name, (_, pivot, axis, degrees) in GEAR.items():
        travel = math.radians(degrees)
        mass, length = (NOSE_STRUT_KG, 0.55) if name == 'nose_gear' else (MAIN_STRUT_KG, 0.52)
        links += gear_link(name, pivot, axis, travel, cg, mass, length)
        sequencer += f'\n      <gear_joint name="{name}_joint" travel="{travel:.4f}" rate="{travel / GEAR_TIME_S:.4f}"/>'
    for name, (_, hinge_a, hinge_b, degrees) in DOORS.items():
        travel = math.radians(degrees)
        links += door_link(name, hinge_a, hinge_b, travel, cg)
        sequencer += f'\n      <door_joint name="{name}_joint" travel="{travel:.4f}" rate="{travel / DOOR_TIME_S:.4f}"/>'
    for wheel, center in wheels.items():
        links += wheel_link(wheel, to_sdf(center - cg), wheel.replace('_wheel', '_main_gear') if wheel != 'nose_wheel' else 'nose_gear')
    travel_flap, travel_tail = math.radians(30), math.radians(25)
    for name, joint, travel, topic in [
            ('left_flap', 'servo_0', travel_flap, 'servo_0'),
            ('right_flap', 'servo_1', travel_flap, 'servo_1'),
            ('left_elevator', 'servo_2', travel_tail, 'servo_2'),
            ('right_elevator', 'servo_2_right', travel_tail, 'servo_2'),
            ('rudder', 'servo_3', travel_tail, 'servo_3')]:
        _, ha, hb = SURFACES[name]
        links += surface_link(name, ha, hb, cg, joint, travel, topic)

    camera_pose = to_sdf(np.asarray(CAMERA_POS)) - cg_sdf
    camera_tilt = math.radians(CAMERA_TILT_DEG)
    prop_pose = to_sdf(np.array([0.0, *PROP_AXIS])) - cg_sdf
    collisions = ''.join([
        box_collision('hull', to_sdf(np.array([0, 0.12, -3.5])) - cg_sdf, (6.9, 1.1, 1.3)),
        box_collision('wing', to_sdf(np.array([0, 0.72, -3.0])) - cg_sdf, (1.8, 9.0, 0.3)),
        box_collision('tail', to_sdf(np.array([0, 1.8, -6.75])) - cg_sdf, (0.8, 2.9, 0.15)),
    ])
    spawn_z = -(GROUND_Y - cg[1])
    k_motor = THRUST_MAX_N / 1000.0 ** 2

    sdf = f"""<?xml version="1.0"?>
<!-- Generated by Tools/simulation/gz_custom_models/generate_gasi_ffa.py. Edit the generator, not this file. -->
<sdf version="1.9">
  <model name="gasi_ffa">
    <pose>0 0 {spawn_z:.4f} 0 0 {SPAWN_YAW_RAD}</pose>
    <link name="base_link">
      <inertial>
        <mass>{base_kg:.1f}</mass>
        <inertia>
          <ixx>{inertia[0, 0]:.1f}</ixx><ixy>{inertia[0, 1]:.1f}</ixy><ixz>{inertia[0, 2]:.1f}</ixz>
          <iyy>{inertia[1, 1]:.1f}</iyy><iyz>{inertia[1, 2]:.1f}</iyz>
          <izz>{inertia[2, 2]:.1f}</izz>
        </inertia>
      </inertial>
      <visual name="body_visual">
        <geometry><mesh><uri>model://gasi_ffa/meshes/body.glb</uri></mesh></geometry>
      </visual>
      <visual name="nose_window_visual">
        <geometry><mesh><uri>model://gasi_ffa/meshes/nose_window.glb</uri></mesh></geometry>
        <material><ambient>0.45 0.6 0.7 1</ambient><diffuse>0.45 0.6 0.7 1</diffuse><specular>0.9 0.9 0.9 1</specular></material>
        <transparency>0.72</transparency>
      </visual>
      <visual name="camera_body_visual">
        <pose>{vec(camera_pose - np.array([0.045, 0, 0]))} 0 {camera_tilt + math.pi / 2:.4f} 0</pose>
        <geometry><cylinder><radius>0.035</radius><length>0.09</length></cylinder></geometry>
        <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material>
      </visual>{collisions}{SENSORS}
      <sensor name="fire_camera" type="camera">
        <gz_frame_id>base_link</gz_frame_id>
        <pose>{vec(camera_pose)} 0 {camera_tilt:.4f} 0</pose>
        <always_on>1</always_on>
        <update_rate>{CAMERA_RATE_HZ}</update_rate>
        <camera>
          <horizontal_fov>{math.radians(CAMERA_HFOV_DEG):.4f}</horizontal_fov>
          <image><width>{CAMERA_SIZE[0]}</width><height>{CAMERA_SIZE[1]}</height><format>R8G8B8</format></image>
          <!-- The near plane lies past the window and the nose, so neither appears in the image. -->
          <clip><near>0.5</near><far>20000</far></clip>
        </camera>
      </sensor>
    </link>{links}
    <link name="rotor">
      <pose>{vec(prop_pose)} 0 1.5707963 0</pose>
      <inertial><mass>{PROP_KG}</mass><inertia><ixx>1.5</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1.5</iyy><iyz>0</iyz><izz>3.0</izz></inertia></inertial>
      <visual name="prop_visual">
        <pose>0 0 0 0 -1.5707963 0</pose>
        <geometry><mesh><uri>model://gasi_ffa/meshes/prop.glb</uri></mesh></geometry>
      </visual>
    </link>
    <joint name="rotor_joint" type="revolute">
      <parent>base_link</parent>
      <child>rotor</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
    </joint>
    <plugin filename="libGearSequencerSystem.so" name="custom::GearSequencerSystem">
      <command_topic>servo_4</command_topic>{sequencer}
    </plugin>
    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_joint</jointName>
      <linkName>rotor</linkName>
      <turningDirection>cw</turningDirection>
      <timeConstantUp>0.5</timeConstantUp>
      <timeConstantDown>0.5</timeConstantDown>
      <maxRotVelocity>1000</maxRotVelocity>
      <motorConstant>{k_motor:.6e}</motorConstant>
      <momentConstant>0.016</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>0</motorNumber>
      <rotorDragCoefficient>1e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>25</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>
    <plugin filename="gz-sim-advanced-lift-drag-system" name="gz::sim::systems::AdvancedLiftDrag">
      <a0>0.0</a0>
      <CL0>{CL0}</CL0>
      <AR>{a['AR']:.3f}</AR>
      <eff>{OSWALD}</eff>
      <CLa>{a['cla']:.3f}</CLa>
      <CD0>{CD0}</CD0>
      <Cem0>{a['cm0']:.4f}</Cem0>
      <Cema>{a['cma']:.4f}</Cema>
      <CYb>{a['cyb']:.4f}</CYb>
      <Cellb>-0.08</Cellb>
      <Cenb>{a['cnb']:.4f}</Cenb>
      <CDp>0</CDp><CYp>0.05</CYp><CLp>0</CLp><Cellp>-0.42</Cellp><Cemp>0</Cemp><Cenp>-0.03</Cenp>
      <CDq>0</CDq><CYq>0</CYq><CLq>{a['clq']:.3f}</CLq><Cellq>0</Cellq><Cemq>{a['cmq']:.3f}</Cemq><Cenq>0</Cenq>
      <CDr>0</CDr><CYr>{a['cyr']:.4f}</CYr><CLr>0</CLr><Cellr>0.08</Cellr><Cemr>0</Cemr><Cenr>{a['cnr']:.4f}</Cenr>
      <alpha_stall>0.28</alpha_stall>
      <CLa_stall>-3.85</CLa_stall>
      <CDa_stall>-0.92</CDa_stall>
      <Cema_stall>-0.5</Cema_stall>
      <cp>0 0 0</cp>
      <area>{WING['S']}</area>
      <mac>{WING['mac']}</mac>
      <air_density>{RHO}</air_density>
      <forward>1 0 0</forward>
      <upward>0 0 1</upward>
      <link_name>base_link</link_name>
      <num_ctrl_surfaces>5</num_ctrl_surfaces>{control_surface('servo_0', 0, 1, 0, 0, -a['flap_cl'], -a['flap_roll'], a['flap_pitch'], -0.00006)}{control_surface('servo_1', 1, 1, 0, 0, -a['flap_cl'], a['flap_roll'], a['flap_pitch'], 0.00006)}{control_surface('servo_2', 2, -1, 0, 0, a['elev_cl'], 0, -a['elev_cm'], 0)}{control_surface('servo_3', 3, 1, 0, -a['rud_cy'], 0, -a['rud_cl'], 0, a['rud_cn'])}{control_surface('left_main_gear_joint', 4, 1, -GEAR_DRAG_CD / GEAR['left_main_gear'][3], 0, 0, 0, 0, 0)}
    </plugin>
  </model>
</sdf>
"""
    with open(os.path.join(out_dir, 'model.sdf'), 'w') as f:
        f.write(sdf)

    print(f'mass {total:.0f} kg, CG (CAD) x={cg[0]:.3f} y={cg[1]:.3f} z={cg[2]:.3f}')
    print(f'inertia Ixx={inertia[0, 0]:.0f} Iyy={inertia[1, 1]:.0f} Izz={inertia[2, 2]:.0f} kg m2')
    print(f'AR={a["AR"]:.2f} CLa={a["cla"]:.2f} V_H={a["vh"]:.3f} V_V={a["vv"]:.3f}')
    print(f'neutral point z={a["np_z"]:.3f}, static margin {a["margin"] * 100:.1f}% MAC')
    print(f'Cm0={a["cm0"]:.4f} Cma={a["cma"]:.3f} Cmq={a["cmq"]:.2f} Cnb={a["cnb"]:.3f} Cmde/deg={a["elev_cm"]:.5f}')
    print(f'cruise drag at 45 m/s {a["drag_cruise"]:.0f} N, thrust line {a["thrust_arm"]:.2f} m above CG')
    print(f'spawn height {spawn_z:.3f} m')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('step')
    args = ap.parse_args()
    here = os.path.dirname(os.path.abspath(__file__))
    out_dir = os.path.join(here, 'gasi_ffa')
    meshes = os.path.join(out_dir, 'meshes')
    os.makedirs(meshes, exist_ok=True)

    with tempfile.TemporaryDirectory() as tmp:
        glb = os.path.join(tmp, 'assembly.glb')
        step_to_glb(args.step, glb)
        cad = Cad(glb)

    mass_data = mass_properties(cad)
    cg = mass_data[1]
    roots = {k for k in cad.parts if k.startswith('NAUO') and int(k[4:]) <= 55}
    body = (roots - BODY_SKIP) | {'NAUO56'}  # the main gear cross bar stays with the hull
    cad.parts['NAUO1'], window = split_nose_window(cad)
    print('body tris', export_group(cad, body, cg, os.path.join(meshes, 'body.glb')))
    cad.parts['window'] = [window]
    print('window tris', export_group(cad, {'window'}, cg, os.path.join(meshes, 'nose_window.glb')))
    span = window.vertices.max(0) - window.vertices.min(0)
    print(f'window size width {span[0]:.3f} height {span[1]:.3f} depth {span[2]:.3f} m')
    print('prop tris', export_group(cad, PROP_NODES, np.array([0.0, *PROP_AXIS]), os.path.join(meshes, 'prop.glb')))
    for name, (nodes, ha, hb) in SURFACES.items():
        mid = (np.asarray(ha) + np.asarray(hb)) / 2
        export_group(cad, nodes, mid, os.path.join(meshes, f'{name}.glb'))
    for name, (nodes, pivot, _, _) in GEAR.items():
        export_group(cad, nodes, np.asarray(pivot), os.path.join(meshes, f'{name}.glb'))
    for name, (nodes, hinge_a, hinge_b, _) in DOORS.items():
        export_group(cad, nodes, (np.asarray(hinge_a) + np.asarray(hinge_b)) / 2, os.path.join(meshes, f'{name}.glb'))
    build_sdf(cad, out_dir, mass_data)

    with open(os.path.join(out_dir, 'model.config'), 'w') as f:
        f.write("""<?xml version="1.0"?>
<model>
  <name>gasi_ffa</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>Ceostech</name></author>
  <description>GASI autonomous amphibious firefighting aircraft. Geometry from CAD; mass, thrust and aerodynamics are estimates.</description>
</model>
""")


if __name__ == '__main__':
    main()
