#!/usr/bin/env python3
"""Builds the gasi_ffa Gazebo model (meshes + model.sdf) from the SolidWorks STEP assembly.

    python3 generate_gasi_ffa.py /path/to/GASI_FFA_STEP_2_8112026.STEP

Needs OCP (cadquery-ocp), trimesh and numpy. Geometry comes from the CAD. The STEP has no
mass, power or aerodynamic data, so everything under "Assumptions" is an engineering estimate:
replace those values with measured ones and rerun.

Frames: CAD has nose +Z, up +Y, left wing +X. SDF has X forward, Y left, Z up, so
X_sdf = Z_cad, Y_sdf = X_cad, Z_sdf = Y_cad. base_link sits on the estimated CG.
"""
import argparse
import math
import os
import re
import tempfile

import numpy as np
import trimesh
from trimesh.visual.material import PBRMaterial

# --- Assumptions (not in the STEP) -----------------------------------------------------------
WATER_FULL_LITERS = 395.1  # CAD water volume
WATER_LITERS = WATER_FULL_LITERS  # 0 = after the drop (see static margin output)
# The STEP has no masses, and the estimated masses put the CG at about 38 % MAC, well behind the wing. The CG along the
# fuselage is therefore taken from the wing: the airframe mass, whose distribution is not known, is placed so that the
# full aircraft balances at this fraction of the MAC. The water body of the CAD sits at 24 % MAC, so the designer put the
# tank on the CG and scooping or dropping hardly moves it.
CG_MAC_FRACTION = 0.25
FUEL_KG = 72.0             # 100 L at 0.72 kg/L
AIRFRAME_KG = 207.0        # chosen so full-load mass matches the parachute class limit (990 kg)
ENGINE_POWER_HP = 280.0    # given by the user; maximum shaft power
ENGINE_RPM = 4400.0        # given by the user; used as propeller speed (direct drive, no reduction gear)
PROP_DIAMETER_M = 1.76
FIGURE_OF_MERIT = 0.70     # static propeller efficiency; sets the static thrust from the power
CL0 = 0.30                 # camber: wing at zero incidence, zero flap. Assumption, the airfoil is not known
WING_INCIDENCE_DEG = 5.0   # design data. The CAD wing chord lies at 0 deg to the body axis, so the incidence is set in the aerodynamics only:
                           # CL0 and the stall angle move by it, which does not depend on how the plugin treats a0
ALPHA_STALL = 0.28         # wing angle of attack at the stall, rad
# gz-sim8 AdvancedLiftDrag does not read CLa_stall, CDa_stall or a0. It blends the linear lift into a flat plate with a
# sigmoid of fixed sharpness M = 15 /rad centred on alpha_stall, so lift already drops well below alpha_stall: with
# alpha_stall at the intended stall angle the maximum CL was 0.95 instead of 1.61 (measured in flight logs). The
# alpha_stall written to the SDF is therefore solved so that the sigmoid's maximum CL equals the linear CL at the
# intended stall angle (gz_alpha_stall). The CL maximum then lies a few degrees above the intended stall angle.
GZ_SIGMOID_M = 15.0
MTOW_KG = 1200.0           # design data. The CAD components add up to about 989 kg; the rest is spread like the base link
                           # and placed at the CG of the whole model, so the CG does not move (set_design_mass)
# Pitching moment of a wing surface deflection, as a fraction of the lift it adds. Assumption, not measured: the camber moment of a
# plain flap about the wing aerodynamic centre is about -0.25 dCL (thin airfoil, cf/c about 0.35), and the lift acts at the
# aerodynamic centre, 10.6 % MAC ahead of the CG (+0.106 dCL). The tail downwash change is left out. The earlier model put the
# lift at the hinge, 0.75 m behind the CG (-0.39 dCL), and the aircraft dived when the landing flaps came out.
SURFACE_CM_PER_CL = -0.15
CD0 = 0.040                # hull and fixed gear
OSWALD = 0.80
TAIL_LIFT_SLOPE = 3.4      # per rad, finite horizontal tail
FIN_LIFT_SLOPE = 2.8       # per rad, finite fin
SURFACE_TAU = 0.55         # effectiveness of the flaps, ailerons and rudder
# NAUO48 and NAUO49 each hold two solids with a 10 mm gap at 2.97-2.98 m along the span. The names do not say which is which: inboard flap, outboard aileron is inferred.
WING_SURFACE_GAP_X = 2.975
FLAP_TRAVEL_DEG = 40.0     # trailing edge down at the end of the joint; the landing setting comes from FW_FLAPS_LND_SCL
FLAP_DRAG_A = 1.7          # plain flap; Cd = A (cf/c)^1.38 (Sf/S) sin^2(d). The plugin is linear, so this is fitted at FLAP_DRAG_FIT_DEG
FLAP_DRAG_FIT_DEG = 30.0
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
    'water_system_dry': (35.0, 'NAUO98'),
    'water': (WATER_LITERS * 0.998, 'NAUO99'),
    'main_gear': (45.0, 'NAUO47'),
}
NOSE_GEAR_KG = 15.0
MAIN_STRUT_KG = 12.5
NOSE_STRUT_KG = 5.0
DOOR_KG = 1.0
PROP_KG = 15.0
WHEEL_KG = 10.0
WHEEL_R = 0.175
WHEEL_FRICTION_NM = 15.0   # rolling resistance of about 3 %
NOSE_STEER_DEG = 30.0      # estimate; the joint limit and the angle at full command
NOSE_STEER_RATE_DEG = 20.0  # estimate; gentle enough that the wheel cannot snap over at speed
NOSE_STEER_LATERAL = 1.5   # m/s2; estimate. The angle limit falls with speed to keep the turn below this: about 0.1 deg at 45 m/s
NOSE_STEER_SPIN_RAD_S = 5.0  # about 0.9 m/s: a nose wheel turning faster is on the ground
NOSE_STEER_STANDING = 0.5  # m/s: below this the aircraft stands on its wheels
NOSE_STEER_KG = 1.0
NOSE_STEER_FADE_START_M_S = 10.0  # assumption, as a tiller on a real aircraft: the angle limit falls from here
NOSE_STEER_CUTOFF_M_S = 15.0      # and the wheel stays straight above this ground speed
NOSE_STEER_LIFTOFF_M = 10.0       # height above the start that counts as having flown: the wheel stays straight on the takeoff run until then
HULL_TAIL_CLEARANCE_DEG = 11.9  # measured on the body mesh: nose up angle about the main wheels at which it touches
HULL_AFT_HEIGHT = 0.9      # the top is not reached by anything, it only has to stay inside the hull
# Suspension: assumptions, the real gear data is not known.
SUSPENSION = True
SUSPENSION_STATIC_M = 0.04  # sink under the full load. The stroke below is what the 0.13 m keel clearance leaves room for
SUSPENSION_STROKE_M = 0.12  # compression from the full load position
SUSPENSION_DROOP_M = 0.03   # extension from there: less than the static sink, so the spring presses the axle on its stop when unloaded
SUSPENSION_DAMPING = 0.4    # ratio of critical damping at full load
# Measured standing on the runway at 989 kg with the GASI_FFA_STEP_21102026 CG (z = -2.752), scaled to MTOW (the CG is the
# same) and to the static split of the current CG, z = -2.5315 (CG_MAC_FRACTION) between the wheels at z = -0.60 and -3.39:
# main 0.6923 instead of 0.7713, nose 0.3077 instead of 0.2287.
SUSPENSION_LOAD_SPLIT = (0.6923 / 0.7713, 0.3077 / 0.2287)
SUSPENSION_MAIN_LOAD_N = 3780.0 * 1200 / 989 * SUSPENSION_LOAD_SPLIT[0]
SUSPENSION_NOSE_LOAD_N = 2130.0 * 1200 / 989 * SUSPENSION_LOAD_SPLIT[1]
# With this much damping the physics engine gives a softer spring than the SDF value, so these come from ground tests:
# they hold the static sink above, which is an effective rate of about 92 and 50 kN/m. They are not the rates themselves.
# Scaled from the 989 kg ground tests by the same mass and split ratios, which keeps the sink.
SUSPENSION_MAIN_SPRING_SDF = 320000.0 * 1200 / 989 * SUSPENSION_LOAD_SPLIT[0]
SUSPENSION_NOSE_SPRING_SDF = 111000.0 * 1200 / 989 * SUSPENSION_LOAD_SPLIT[1]
AXLE_KG = 1.0
BRAKE_DECEL = 2.5          # m/s2 at full load; estimate. About half the 5.3 m/s2 at which the main wheels would lock (mu 0.9)
BRAKE_TOUCH_FRACTION = 0.3  # of the torque, until the nose wheel is down
BRAKE_ENABLED = True
BRAKE_RAMP_S = 3.0
BRAKE_NOSE_CONFIRM_S = 1.0  # a bounce right after touchdown also spins the nose wheel for a moment
BRAKE_SLIP_RELEASE = 0.15  # estimate; wheel speed this far below the ground speed releases both wheels
BRAKE_SLIP_RESUME = 0.05
BRAKE_RELEASE_S = 0.05
BRAKE_SPIN_RAD_S = 5.0     # about 0.9 m/s: a wheel turning faster is on the ground
BRAKE_FADE_RAD_S = 10.0    # about 1.75 m/s: the torque fades below this, so the wheels cannot lock while stopping
BRAKE_CONFIRM_S = 0.2
BRAKE_MOTOR_OFF = 0.2      # of the motor command range: above this the brakes release (takeoff run, go-around)

# --- Geometry taken from the CAD --------------------------------------------------------------
# Node numbers of GASI_FFA_STEP_2_8112026 (the top-level parts are NAUO1-NAUO50).
NOSE_GEAR_NODES = {f'NAUO{i}' for i in range(18, 47)}  # strut, links, hubs, wheel and fasteners
NOSE_STEER_NODES = {'NAUO21', 'NAUO24', 'NAUO25', 'NAUO26', *(f'NAUO{i}' for i in range(33, 47))}  # fork, wheel and its hub: they turn together
NOSE_WHEEL_NODE, LEFT_WHEEL_NODE, RIGHT_WHEEL_NODE = 'NAUO24', 'NAUO67', 'NAUO88'
TOP_LEVEL_NODES = 50
MAIN_GEAR_BAR_NODE = 'NAUO51'  # MLG_MAIN_SHAFT: the cross bar stays with the hull
BODY_SKIP = {'NAUO2', 'NAUO3', 'NAUO4', 'NAUO5', 'NAUO6', 'NAUO7', 'NAUO8', 'NAUO16', 'NAUO47',
             'NAUO14', 'NAUO15', *NOSE_GEAR_NODES,
             'NAUO9', 'NAUO10', 'NAUO11', 'NAUO12', 'NAUO13', 'NAUO48', 'NAUO49'}
PROP_NODES = {'NAUO9', 'NAUO10'}
GROUND_Y = -0.77           # tire contact height in the CAD frame
SPAWN_YAW_RAD = 1.4835     # nose on Antalya runway 36C (heading 005); PX4_GZ_MODEL_POSE overrides it
PROP_AXIS = (1.02, -4.225)  # CAD y, z of the propeller axis and disc plane
# The hinge line is the arc centre of the surface noses. The CAD has no hinge pin, so this is inferred too.
LEFT_HINGE_A, LEFT_HINGE_B = (0.9, 0.6425, -3.3585), (4.08, 0.8057, -3.5095)
RIGHT_HINGE_A, RIGHT_HINGE_B = (-0.9, 0.6425, -3.3585), (-4.08, 0.8057, -3.5095)


def wing_hinge_point(a, b):
    return tuple(p + (WING_SURFACE_GAP_X - a[0] * np.sign(a[0])) / (abs(b[0]) - abs(a[0])) * (q - p) for p, q in zip(a, b))


SURFACES = {
    # link: (CAD nodes, hinge end A, hinge end B in the CAD frame)
    # The wing surfaces of the CAD, NAUO48 and NAUO49, are separated into these by split_wing_surfaces().
    'left_flap': ({'left_flap_part'}, LEFT_HINGE_A, wing_hinge_point(LEFT_HINGE_A, LEFT_HINGE_B)),
    'right_flap': ({'right_flap_part'}, RIGHT_HINGE_A, wing_hinge_point(RIGHT_HINGE_A, RIGHT_HINGE_B)),
    'left_aileron': ({'left_aileron_part'}, wing_hinge_point(LEFT_HINGE_A, LEFT_HINGE_B), LEFT_HINGE_B),
    'right_aileron': ({'right_aileron_part'}, wing_hinge_point(RIGHT_HINGE_A, RIGHT_HINGE_B), RIGHT_HINGE_B),
    'left_elevator': ({'NAUO13'}, (0.265, 1.8, -6.851), (1.355, 1.8, -6.859)),
    'right_elevator': ({'NAUO12'}, (-0.265, 1.8, -6.851), (-1.355, 1.797, -6.859)),
    'rudder': ({'NAUO11'}, (0.0, 1.629, -6.863), (0.0, 0.39, -6.493)),
}
# Retraction. The STEP has no kinematics, so the pivots and axes below are derived from the CAD
# parts and the well they must fit in (see README). A positive angle always means gear up / door
# open. A door axis runs from `a` to `b`, so its direction sets which way that door swings.
GEAR = {
    # link: (CAD nodes, pivot, axis, travel in degrees)
    # Main gear: forward about the cross bar into the open well of the sponson. At 70 deg the tire touches the well roof
    # and still stands 0.11 m below the sponson; 75 deg runs it 5 cm into the roof, folding aft does not fit at all.
    'left_main_gear': ({'NAUO53'}, (0.745, -0.09, -3.39), (-1.0, 0.0, 0.0), 70.0),
    'right_main_gear': ({'NAUO52'}, (-0.745, -0.09, -3.39), (-1.0, 0.0, 0.0), 70.0),
    'nose_gear': (NOSE_GEAR_NODES, (0.0, -0.045, -0.60), (1.0, 0.0, 0.0), 75.0),
}
DOORS = {
    # link: (CAD nodes, hinge a, hinge b, travel in degrees)
    # The sponson wells of the main gear have no doors.
    'left_nose_door': ({'NAUO14'}, (0.126, -0.354, -1.274), (0.126, -0.204, -0.580), 80.0),
    'right_nose_door': ({'NAUO15'}, (-0.126, -0.204, -0.580), (-0.126, -0.354, -1.274), 80.0),
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
# Radar altimeter (user decision 2026-11-10: the aircraft carries a radar): 24 GHz class, e.g. Ainstein US-D1, 0.5-50 m,
# about +-5 cm. Simulated as a one-ray gpu_lidar: it renders the water surface (a visual mesh, no collision), measured
# 10.00 m over the sea; a physics ray sensor returned nothing. PX4's gz_bridge subscribes to
# /world/<w>/model/<m>/link/lidar_sensor_link/sensor/lidar/scan (SIM_GZ_EN_LIDAR) and reports it as downward facing when the
# sensor sits rotated as in the advanced_plane model (link pitched 90 deg, sensor rolled 180 deg).
RADAR_POS = (0.0, 0.0, -0.20)   # body frame from the CG, m: under the CG, 0.54 m above the hull bottom (z -0.736)
RADAR_RANGE = (0.5, 50.0)
RADAR_NOISE_M = 0.05
RADAR_RATE_HZ = 50
WINDOW_MARGIN_DEG = 6.0    # how far the transparent nose extends past the camera field of view

# Water release at the bottom of the CAD 150 mm pipe.
WATER_OUTLET = (0.0, -0.62, -3.32)  # CAD frame

# Planform values measured by projecting CAD triangles (see README).
# S from the design data. Span and MAC from the CAD: leading edge z = -1.7754 - 0.1246 |x| (7.1 deg sweep), trailing
# edge of the flaps and ailerons at z = -4.0, root chord 2.23 m, tip chord 1.66 m, plan area 17.5 m2.
WING = dict(S=15.0, b=9.0, mac=1.958, mac_le_z=-2.042)
HTAIL = dict(S=2.17, z=-6.81, elev_S=0.655)
VTAIL = dict(S=1.54, z=-6.52, rudder_S=0.523, y=1.1)
# Plan areas and spanwise centres from the CAD solids.
FLAP = dict(S=1.404, y=1.904, chord=0.678)
AILERON = dict(S=0.643, y=3.520)
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


def split_wing_surfaces(cad):
    """Separates the two solids of each wing surface part. They do not touch, so no triangle has to be cut."""
    for side, key in (('left', 'NAUO48'), ('right', 'NAUO49')):
        parts = {'flap': [], 'aileron': []}
        for mesh in cad.parts[key]:
            inboard = np.abs(mesh.triangles_center[:, 0]) < WING_SURFACE_GAP_X
            parts['flap'].append(mesh.submesh([np.nonzero(inboard)[0]], append=True))
            parts['aileron'].append(mesh.submesh([np.nonzero(~inboard)[0]], append=True))
        cad.parts[f'{side}_flap_part'] = parts['flap']
        cad.parts[f'{side}_aileron_part'] = parts['aileron']


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
    points.append(('nose_gear', NOSE_GEAR_KG, cad.centre(NOSE_WHEEL_NODE), cad.size(NOSE_WHEEL_NODE)))
    points.append(('prop', PROP_KG, np.array([0.0, *PROP_AXIS]), np.array([1.2, 1.2, 0.1])))

    skin = np.vstack([np.asarray(m.triangles) for m in cad.parts['NAUO1']])
    area = np.linalg.norm(np.cross(skin[:, 1] - skin[:, 0], skin[:, 2] - skin[:, 0]), axis=1) / 2
    centers = skin.mean(1)
    centroid = (area[:, None] * centers).sum(0) / area.sum()
    spread = np.sqrt((area[:, None] * (centers - centroid) ** 2).sum(0) / area.sum())

    # Airframe mass along the fuselage: the full aircraft (full tank) balances at CG_MAC_FRACTION of the MAC.
    water_kg, water_c = WATER_FULL_LITERS * 0.998, cad.centre('NAUO99')
    full = [(kg, c) for name, kg, c, _ in points if name != 'water'] + [(water_kg, water_c)]
    full_kg = AIRFRAME_KG + sum(kg for kg, _ in full)
    cg_target_z = WING['mac_le_z'] - CG_MAC_FRACTION * WING['mac']
    skin_z = centroid[2]
    centroid[2] = (cg_target_z * full_kg - sum(kg * c[2] for kg, c in full)) / AIRFRAME_KG
    print(f'airframe centroid z {centroid[2]:.3f} (skin area centroid {skin_z:.3f}) for the CG at {CG_MAC_FRACTION:.0%} MAC, z={cg_target_z:.3f}')

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


def gz_lift(alpha, cl0, cla, alpha_stall):
    """CL of gz-sim8 AdvancedLiftDrag (no control surfaces) at body angle of attack alpha, rad."""
    e1 = math.exp(-GZ_SIGMOID_M * (alpha - alpha_stall))
    e2 = math.exp(GZ_SIGMOID_M * (alpha + alpha_stall))
    sigma = (1 + e1 + e2) / ((1 + e1) * (1 + e2))
    flat_plate = 2 * math.copysign(1.0, alpha) * math.sin(alpha) ** 2 * math.cos(alpha) if alpha else 0.0
    return (1 - sigma) * (cl0 + cla * alpha) + sigma * flat_plate


def gz_alpha_stall(cl0, cla, alpha_stall_body):
    """alpha_stall for the plugin so that its maximum CL equals cl0 + cla * alpha_stall_body (bisection)."""
    target = cl0 + cla * alpha_stall_body
    grid = [math.radians(0.1 * i) for i in range(0, 451)]
    def peak(s): return max(gz_lift(a, cl0, cla, s) for a in grid)
    lo, hi = alpha_stall_body, alpha_stall_body + 0.5
    for _ in range(40):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if peak(mid) < target else (lo, mid)
    return (lo + hi) / 2


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
    drag_cruise = q_cruise * S * (CD0 + (MTOW_KG * 9.81 / (q_cruise * S)) ** 2 / (math.pi * ar * OSWALD))
    thrust_arm = PROP_AXIS[0] - cg[1]
    cl0_body = CL0 + cla * math.radians(WING_INCIDENCE_DEG)
    alpha_cruise = (MTOW_KG * 9.81 / (q_cruise * S) - cl0_body) / cla
    cm0 = -cma * alpha_cruise + drag_cruise * thrust_arm / (q_cruise * S * c)

    flap_cl = FLAP['S'] * a_wing * SURFACE_TAU / S
    flap_pitch = -SURFACE_CM_PER_CL * flap_cl
    flap_cd = FLAP_DRAG_A * (FLAP['chord'] / c) ** 1.38 * (2 * FLAP['S'] / S) * math.sin(math.radians(FLAP_DRAG_FIT_DEG)) ** 2 / FLAP_DRAG_FIT_DEG
    ail_cl = AILERON['S'] * a_wing * SURFACE_TAU / S
    ail_roll = ail_cl * AILERON['y'] / b
    ail_pitch = -SURFACE_CM_PER_CL * ail_cl
    elev_cl = TAIL_LIFT_SLOPE * HTAIL['S'] / S * ELEV_TAU
    elev_cm = TAIL_LIFT_SLOPE * vh * ELEV_TAU
    rud_cy = VTAIL['rudder_S'] * FIN_LIFT_SLOPE * SURFACE_TAU / S
    rud_cn = rud_cy * arm_v / b
    d = math.pi / 180  # the plugin takes control derivatives per degree
    return dict(
        AR=ar, cla=cla, cl0_body=cl0_body, alpha_cruise=alpha_cruise, vh=vh, vv=vv, margin=margin, np_z=np_z, ac_z=ac_z, cm0=cm0, cma=cma,
        drag_cruise=drag_cruise, thrust_arm=thrust_arm,
        cmq=-2 * TAIL_LIFT_SLOPE * vh * arm_h / c - 0.5, clq=2 * TAIL_LIFT_SLOPE * vh,
        cnb=vv * FIN_LIFT_SLOPE - 0.04, cyb=-VTAIL['S'] / S * FIN_LIFT_SLOPE - 0.05,
        cnr=-2 * vv * FIN_LIFT_SLOPE * arm_v / b - 0.02, cyr=2 * vv * FIN_LIFT_SLOPE,
        flap_cl=flap_cl * d, flap_pitch=flap_pitch * d, flap_cd=flap_cd,
        ail_cl=ail_cl * d, ail_roll=ail_roll * d, ail_pitch=ail_pitch * d,
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


def box_collision(name, center, size, pitch=0.0):
    return f"""
      <collision name="{name}">
        <pose>{vec(center)} 0 {pitch:.4f} 0</pose>
        <geometry><box><size>{vec(size)}</size></box></geometry>
      </collision>
      <sensor name="{name}_contact" type="contact"><always_on>true</always_on><update_rate>250</update_rate><contact><collision>{name}</collision></contact></sensor>"""


def surface_link(name, hinge_a, hinge_b, cg, joint, travel, topic, mass=1.0):
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
      <inertial><mass>{mass}</mass><inertia><ixx>{0.05 * mass:.4f}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{0.05 * mass:.4f}</iyy><iyz>0</iyz><izz>{0.05 * mass:.4f}</izz></inertia></inertial>
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


def suspension_links(name, center_sdf, parent, load_n, spring_sdf):
    """An axle on a vertical sliding joint between the gear and the wheel. The wheel stays where it is under full load."""
    stiffness = load_n / SUSPENSION_STATIC_M
    damping = 2 * SUSPENSION_DAMPING * math.sqrt(stiffness * load_n / 9.81)
    i = AXLE_KG * 0.05 ** 2 / 6
    return f"""
    <link name="{name}_axle">
      <pose>{vec(center_sdf)} 0 0 0</pose>
      <inertial><mass>{AXLE_KG}</mass><inertia><ixx>{i:.6f}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{i:.6f}</iyy><iyz>0</iyz><izz>{i:.6f}</izz></inertia></inertial>
    </link>
    <joint name="{name}_suspension" type="prismatic">
      <parent>{parent}</parent>
      <child>{name}_axle</child>
      <axis>
        <xyz expressed_in="__model__">0 0 1</xyz>
        <limit><lower>{-SUSPENSION_DROOP_M}</lower><upper>{SUSPENSION_STROKE_M}</upper></limit>
        <dynamics><damping>{damping:.1f}</damping><spring_reference>{-SUSPENSION_STATIC_M}</spring_reference><spring_stiffness>{spring_sdf:.1f}</spring_stiffness></dynamics>
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
      <sensor name="{name.replace('_wheel', '')}_contact" type="contact"><always_on>true</always_on><update_rate>250</update_rate><contact><collision>{name}_collision</collision></contact></sensor>
    </link>
    <joint name="{name}_joint" type="revolute">
      <parent>{parent}</parent>
      <child>{name}</child>
      <axis><xyz>0 1 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit><dynamics><friction>{WHEEL_FRICTION_NM}</friction></dynamics></axis>
    </joint>"""


def nose_steer_link(center_sdf):
    i = NOSE_STEER_KG * 0.2 ** 2 / 12
    limit = math.radians(NOSE_STEER_DEG)
    return f"""
    <link name="nose_steer">
      <pose>{vec(center_sdf)} 0 0 0</pose>
      <inertial><mass>{NOSE_STEER_KG}</mass><inertia><ixx>{i:.5f}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{i:.5f}</iyy><iyz>0</iyz><izz>{i:.5f}</izz></inertia></inertial>
      <visual name="nose_steer_visual">
        <geometry><mesh><uri>model://gasi_ffa/meshes/nose_steer.glb</uri></mesh></geometry>
      </visual>
    </link>
    <joint name="nose_steer_joint" type="revolute">
      <parent>nose_gear</parent>
      <child>nose_steer</child>
      <axis><xyz expressed_in="__model__">0 0 1</xyz><limit><lower>{-limit:.4f}</lower><upper>{limit:.4f}</upper></limit></axis>
    </joint>"""


def control_surface(joint, index, direction, cd, cy, cl, cell, cem, cen):
    return f"""
      <control_surface>
        <name>{joint}</name><index>{index}</index><direction>{direction}</direction>
        <CD_ctrl>{cd:.7f}</CD_ctrl><CY_ctrl>{cy:.7f}</CY_ctrl><CL_ctrl>{cl:.7f}</CL_ctrl>
        <Cell_ctrl>{cell:.7f}</Cell_ctrl><Cem_ctrl>{cem:.7f}</Cem_ctrl><Cen_ctrl>{cen:.7f}</Cen_ctrl>
      </control_surface>"""


def set_design_mass(sdf):
    """Raises the model to MTOW_KG: the base link takes the extra mass at the CG of all links, and its inertia grows with its mass."""
    total, moment = 0.0, np.zeros(3)
    for m in re.finditer(r'<link name="(\w+)">\s*<pose>([^<]*)</pose>\s*<inertial>(.*?)</inertial>', sdf, re.S):
        kg = float(re.search(r'<mass>(.*?)</mass>', m.group(3)).group(1))
        local = re.search(r'<pose>(.*?)</pose>', m.group(3))
        pos = np.array(m.group(2).split()[:3], float) + (np.array(local.group(1).split()[:3], float) if local else 0.0)
        total, moment = total + kg, moment + kg * pos
    base = re.search(r'(<link name="base_link">\s*<inertial>\s*)<mass>([\d.]+)</mass>(.*?)</inertia>', sdf, re.S)
    base_kg = float(base.group(2))
    total += base_kg  # the base link has no pose: it sits at the model origin
    extra = MTOW_KG - total
    cg = moment / total
    new_kg = base_kg + extra
    offset = cg * extra / new_kg
    scale = new_kg / base_kg
    inertia = re.sub(r'<(i\w\w)>([-\d.]+)</i\w\w>', lambda x: f'<{x.group(1)}>{float(x.group(2)) * scale:.1f}</{x.group(1)}>', base.group(3))
    return sdf[:base.start()] + f'{base.group(1)}<pose>{vec(offset)} 0 0 0</pose>\n        <mass>{new_kg:.1f}</mass>{inertia}</inertia>' + sdf[base.end():]


def build_sdf(cad, out_dir, mass_data):
    total, cg, inertia = mass_data
    a = aero(cg)
    cg_sdf = to_sdf(cg)
    moving_kg = PROP_KG + 3 * WHEEL_KG + 2 * MAIN_STRUT_KG + NOSE_STRUT_KG + NOSE_STEER_KG + (3 * AXLE_KG if SUSPENSION else 0.0) + 5 * 1.0 + len(DOORS) * DOOR_KG
    base_kg = total - moving_kg

    wheels = {
        'nose_wheel': np.array([0.0, GROUND_Y + WHEEL_R, cad.centre(NOSE_WHEEL_NODE)[2]]),
        'left_wheel': np.array([cad.centre(LEFT_WHEEL_NODE)[0], GROUND_Y + WHEEL_R, cad.centre(LEFT_WHEEL_NODE)[2]]),
        'right_wheel': np.array([cad.centre(RIGHT_WHEEL_NODE)[0], GROUND_Y + WHEEL_R, cad.centre(RIGHT_WHEEL_NODE)[2]]),
    }
    # Suya temas izleri: WaterPhysics gövde ıslakken bunları hızla orantılı açar (particle_emitter/<ad>/cmd). Konumlar
    # su çıkışına göre (gövdenin redan bölgesi, en alt nokta): serpinti redandan geriye ve yukarı, burun dalgası öne,
    # iz kuyruğun arkasında. Yayıcı kendi +X ekseni boyunca yayar; yaw 180° geriye çevirir, eksi pitch yukarı kaldırır.
    step_ref = to_sdf(np.asarray(WATER_OUTLET)) - cg_sdf
    def spray(name, offset, pitch, size, part, life, vmin, vmax, scale):
        return f'''
      <particle_emitter name="{name}" type="box">
        <pose>{vec(step_ref + np.asarray(offset))} 0 {pitch} 3.14159</pose>
        <emitting>false</emitting>
        <size>{size}</size>
        <particle_size>{part} {part} {part}</particle_size>
        <lifetime>{life}</lifetime>
        <min_velocity>{vmin}</min_velocity>
        <max_velocity>{vmax}</max_velocity>
        <scale_rate>{scale}</scale_rate>
        <rate>0</rate>
        <material>
          <diffuse>1 1 1</diffuse>
          <pbr><metal><albedo_map>model://gasi_ffa/materials/textures/water.png</albedo_map></metal></pbr>
        </material>
      </particle_emitter>'''
    spray_emitters = (spray('spray_aft', (-0.2, 0.0, 0.05), -0.45, '0.3 1.4 0.05', 0.5, 1.0, 5, 11, 2.0)
                      + spray('spray_bow', (1.6, 0.0, 0.15), -0.9, '0.2 1.0 0.05', 0.3, 0.8, 2, 5, 1.5)
                      + spray('wake', (-2.9, 0.0, 0.45), 0.0, '0.5 1.4 0.05', 0.8, 4.0, 0.5, 1.5, 0.6))
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
    links += nose_steer_link(to_sdf(wheels['nose_wheel'] - cg))
    for wheel, center in wheels.items():
        parent = wheel.replace('_wheel', '_main_gear') if wheel != 'nose_wheel' else 'nose_steer'
        if SUSPENSION:
            nose = wheel == 'nose_wheel'
            links += suspension_links(wheel, to_sdf(center - cg), parent, SUSPENSION_NOSE_LOAD_N if nose else SUSPENSION_MAIN_LOAD_N,
                                      SUSPENSION_NOSE_SPRING_SDF if nose else SUSPENSION_MAIN_SPRING_SDF)
            parent = f'{wheel}_axle'
        links += wheel_link(wheel, to_sdf(center - cg), parent)
    wheelbase = abs(to_sdf(wheels['nose_wheel'] - wheels['left_wheel'])[0])
    travel_aileron, travel_flap, travel_tail = math.radians(30), math.radians(FLAP_TRAVEL_DEG), math.radians(25)
    flap_kg = round(FLAP['S'] / (FLAP['S'] + AILERON['S']), 3)  # the 1 kg of the old single link, by area
    aileron_kg = round(1 - flap_kg, 3)
    for name, joint, travel, topic, kg in [
            ('left_aileron', 'servo_0', travel_aileron, 'servo_0', aileron_kg),
            ('right_aileron', 'servo_1', travel_aileron, 'servo_1', aileron_kg),
            ('left_elevator', 'servo_2', travel_tail, 'servo_2', 1.0),
            ('right_elevator', 'servo_2_right', travel_tail, 'servo_2', 1.0),
            ('rudder', 'servo_3', travel_tail, 'servo_3', 1.0),
            ('left_flap', 'servo_6', travel_flap, 'servo_6', flap_kg),
            ('right_flap', 'servo_6_right', travel_flap, 'servo_6', flap_kg)]:
        _, ha, hb = SURFACES[name]
        links += surface_link(name, ha, hb, cg, joint, travel, topic, kg)

    camera_pose = to_sdf(np.asarray(CAMERA_POS)) - cg_sdf
    camera_tilt = math.radians(CAMERA_TILT_DEG)
    prop_pose = to_sdf(np.array([0.0, *PROP_AXIS])) - cg_sdf
    # The hull is two boxes. A single box runs level to the tail and touches the runway at 3.5 deg nose up, where the
    # real hull clears it until HULL_TAIL_CLEARANCE_DEG. The aft box rises at that angle from the main wheel contact.
    hull_center = to_sdf(np.array([0, 0.12, -3.5])) - cg_sdf
    hull_length, hull_width, hull_height = 6.9, 1.1, 1.3
    wheel_x, wheel_bottom = to_sdf(wheels['left_wheel'] - cg)[0], to_sdf(wheels['left_wheel'] - cg)[2] - WHEEL_R
    hull_bottom = hull_center[2] - hull_height / 2
    hull_rear, hull_front = hull_center[0] - hull_length / 2, hull_center[0] + hull_length / 2
    tilt = math.radians(HULL_TAIL_CLEARANCE_DEG)
    hull_break = wheel_x - (hull_bottom - wheel_bottom) / math.tan(tilt)
    aft_end_z = wheel_bottom + math.tan(tilt) * (wheel_x - hull_rear)
    aft_length = math.hypot(hull_break - hull_rear, aft_end_z - hull_bottom)
    aft_center = (np.array([hull_break + hull_rear, 0.0, hull_bottom + aft_end_z]) / 2
                  + np.array([math.sin(tilt), 0.0, math.cos(tilt)]) * HULL_AFT_HEIGHT / 2)
    aft_center[1] = hull_center[1]
    fore_center = np.array([(hull_break + hull_front) / 2, hull_center[1], hull_center[2]])
    collisions = ''.join([
        box_collision('hull', fore_center, (hull_front - hull_break, hull_width, hull_height)),
        box_collision('hull_aft', aft_center, (aft_length, hull_width, HULL_AFT_HEIGHT), tilt),
        box_collision('wing', to_sdf(np.array([0, 0.72, -3.0])) - cg_sdf, (1.8, 9.0, 0.3)),
        box_collision('tail', to_sdf(np.array([0, 1.8, -6.75])) - cg_sdf, (0.8, 2.9, 0.15)),
    ])
    spawn_z = -(GROUND_Y - cg[1])
    # The plugin has no power input: power sets the static thrust (momentum theory) and the torque.
    power_w = ENGINE_POWER_HP * 745.7
    omega_max = ENGINE_RPM * 2 * math.pi / 60
    disc_area = math.pi * PROP_DIAMETER_M ** 2 / 4
    thrust_max = (FIGURE_OF_MERIT * power_w * math.sqrt(2 * RHO * disc_area)) ** (2 / 3)
    k_motor = thrust_max / omega_max ** 2
    k_moment = power_w / omega_max / thrust_max

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
      <!-- Water curtain, started and stopped on the emitter's cmd topic. It emits along +X, pitched straight down. -->
      <particle_emitter name="water" type="box">
        <pose>{vec(to_sdf(np.asarray(WATER_OUTLET)) - cg_sdf)} 0 1.5708 0</pose>
        <emitting>false</emitting>
        <size>0.5 1.0 1.0</size>
        <particle_size>3 3 3</particle_size>
        <lifetime>6</lifetime>
        <min_velocity>14</min_velocity>
        <max_velocity>22</max_velocity>
        <scale_rate>3</scale_rate>
        <rate>160</rate>
        <material>
          <diffuse>1 1 1</diffuse>
          <pbr><metal><albedo_map>model://gasi_ffa/materials/textures/water.png</albedo_map></metal></pbr>
        </material>
      </particle_emitter>{spray_emitters}
    </link>{links}
    <link name="lidar_sensor_link">
      <pose>{RADAR_POS[0]:.3f} {RADAR_POS[1]:.3f} {RADAR_POS[2]:.3f} 0 1.5707963 0</pose>
      <inertial><mass>0.001</mass><inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <sensor name="lidar" type="gpu_lidar">
        <gz_frame_id>lidar_sensor_link</gz_frame_id>
        <pose>0 0 0 3.1415927 0 0</pose>
        <always_on>1</always_on>
        <update_rate>{RADAR_RATE_HZ}</update_rate>
        <lidar>
          <scan>
            <horizontal><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></horizontal>
            <vertical><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></vertical>
          </scan>
          <range><min>{RADAR_RANGE[0]}</min><max>{RADAR_RANGE[1]}</max><resolution>0.01</resolution></range>
          <noise><type>gaussian</type><mean>0</mean><stddev>{RADAR_NOISE_M}</stddev></noise>
        </lidar>
      </sensor>
    </link>
    <joint name="lidar_sensor_joint" type="fixed">
      <parent>base_link</parent>
      <child>lidar_sensor_link</child>
    </joint>
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
    <plugin filename="libNoseSteeringSystem.so" name="custom::NoseSteeringSystem">
      <steer_joint>nose_steer_joint</steer_joint>
      <wheel_joint>nose_wheel_joint</wheel_joint>
      <gear_joint>nose_gear_joint</gear_joint>
      <command_topic>servo_5</command_topic>
      <gear_topic>servo_4</gear_topic>
      <motor_topic>command/motor_speed</motor_topic>
      <motor_off>{BRAKE_MOTOR_OFF * omega_max:.1f}</motor_off>
      <full_angle>{math.radians(NOSE_STEER_DEG):.4f}</full_angle>
      <wheelbase>{wheelbase:.3f}</wheelbase>
      <lateral_accel>{NOSE_STEER_LATERAL}</lateral_accel>
      <steer_fade_start_m_s>{NOSE_STEER_FADE_START_M_S}</steer_fade_start_m_s>
      <steer_cutoff_m_s>{NOSE_STEER_CUTOFF_M_S}</steer_cutoff_m_s>
      <liftoff_height_m>{NOSE_STEER_LIFTOFF_M}</liftoff_height_m>
      <rate>{math.radians(NOSE_STEER_RATE_DEG):.4f}</rate>
      <spin_rad_s>{NOSE_STEER_SPIN_RAD_S}</spin_rad_s>
      <standing_m_s>{NOSE_STEER_STANDING}</standing_m_s>
      <gain>5.0</gain>
      <sign>-1.0</sign>
    </plugin>
    <plugin filename="libWheelBrakeSystem.so" name="custom::WheelBrakeSystem">
      <left_wheel_joint>left_wheel_joint</left_wheel_joint>
      <right_wheel_joint>right_wheel_joint</right_wheel_joint>
      <nose_wheel_joint>nose_wheel_joint</nose_wheel_joint>
      <left_gear_joint>left_main_gear_joint</left_gear_joint>
      <right_gear_joint>right_main_gear_joint</right_gear_joint>
      <gear_topic>servo_4</gear_topic>
      <motor_topic>command/motor_speed</motor_topic>
      <max_torque>{total * BRAKE_DECEL * WHEEL_R / 2:.1f}</max_torque>
      <touch_fraction>{BRAKE_TOUCH_FRACTION}</touch_fraction>
      <ramp_s>{BRAKE_RAMP_S}</ramp_s>
      <spin_rad_s>{BRAKE_SPIN_RAD_S}</spin_rad_s>
      <fade_rad_s>{BRAKE_FADE_RAD_S}</fade_rad_s>
      <confirm_s>{BRAKE_CONFIRM_S}</confirm_s>
      <nose_confirm_s>{BRAKE_NOSE_CONFIRM_S}</nose_confirm_s>
      <motor_off>{BRAKE_MOTOR_OFF * omega_max:.1f}</motor_off>
      <wheel_radius>{WHEEL_R}</wheel_radius>
      <slip_release>{BRAKE_SLIP_RELEASE}</slip_release>
      <slip_resume>{BRAKE_SLIP_RESUME}</slip_resume>
      <release_s>{BRAKE_RELEASE_S}</release_s>
      <enabled>{str(BRAKE_ENABLED).lower()}</enabled>
    </plugin>
    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_joint</jointName>
      <linkName>rotor</linkName>
      <turningDirection>cw</turningDirection>
      <timeConstantUp>0.5</timeConstantUp>
      <timeConstantDown>0.5</timeConstantDown>
      <maxRotVelocity>{omega_max:.2f}</maxRotVelocity>
      <motorConstant>{k_motor:.6e}</motorConstant>
      <momentConstant>{k_moment:.5f}</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>0</motorNumber>
      <rotorDragCoefficient>1e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>20</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>
    <plugin filename="gz-sim-advanced-lift-drag-system" name="gz::sim::systems::AdvancedLiftDrag">
      <a0>0.0</a0>
      <CL0>{a['cl0_body']:.4f}</CL0>
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
      <alpha_stall>{gz_alpha_stall(a['cl0_body'], a['cla'], ALPHA_STALL - math.radians(WING_INCIDENCE_DEG)):.4f}</alpha_stall>
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
      <num_ctrl_surfaces>6</num_ctrl_surfaces>{control_surface('servo_0', 0, 1, 0, 0, -a['ail_cl'], -a['ail_roll'], a['ail_pitch'], -0.00003)}{control_surface('servo_1', 1, 1, 0, 0, -a['ail_cl'], a['ail_roll'], a['ail_pitch'], 0.00003)}{control_surface('servo_2', 2, -1, 0, 0, a['elev_cl'], 0, -a['elev_cm'], 0)}{control_surface('servo_3', 3, 1, 0, -a['rud_cy'], 0, -a['rud_cl'], 0, a['rud_cn'])}{control_surface('left_main_gear_joint', 4, 1, -GEAR_DRAG_CD / GEAR['left_main_gear'][3], 0, 0, 0, 0, 0)}{control_surface('servo_6', 5, 1, -a['flap_cd'], 0, -2 * a['flap_cl'], 0, 2 * a['flap_pitch'], 0)}
    </plugin>
  </model>
</sdf>
"""
    with open(os.path.join(out_dir, 'model.sdf'), 'w') as f:
        f.write(set_design_mass(sdf))
    # Su yansıması için ayna kopyası (WaterPhysics kullanır; yalnız görüntü), model.sdf ile aynı görsellerden
    import make_mirror
    make_mirror.main(out_dir)

    print(f'mass {total:.0f} kg, CG (CAD) x={cg[0]:.3f} y={cg[1]:.3f} z={cg[2]:.3f}')
    print(f'inertia Ixx={inertia[0, 0]:.0f} Iyy={inertia[1, 1]:.0f} Izz={inertia[2, 2]:.0f} kg m2')
    print(f'AR={a["AR"]:.2f} CLa={a["cla"]:.2f} V_H={a["vh"]:.3f} V_V={a["vv"]:.3f}')
    print(f'neutral point z={a["np_z"]:.3f}, static margin {a["margin"] * 100:.1f}% MAC')
    print(f'Cm0={a["cm0"]:.4f} Cma={a["cma"]:.3f} Cmq={a["cmq"]:.2f} Cnb={a["cnb"]:.3f} Cmde/deg={a["elev_cm"]:.5f}')
    print(f'body angle of attack in cruise at 45 m/s {math.degrees(a["alpha_cruise"]):.2f} deg, CL0 at body level {a["cl0_body"]:.3f}')
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
    split_wing_surfaces(cad)
    roots = {k for k in cad.parts if k.startswith('NAUO') and int(k[4:]) <= TOP_LEVEL_NODES}
    body = (roots - BODY_SKIP) | {MAIN_GEAR_BAR_NODE}
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
        export_group(cad, nodes - NOSE_STEER_NODES, np.asarray(pivot), os.path.join(meshes, f'{name}.glb'))
    export_group(cad, NOSE_STEER_NODES, (0.0, GROUND_Y + WHEEL_R, cad.centre(NOSE_WHEEL_NODE)[2]), os.path.join(meshes, 'nose_steer.glb'))
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
