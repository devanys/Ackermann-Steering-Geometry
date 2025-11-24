import pybullet as p
import pybullet_data
import time
import math
import numpy as np

def forward_ackermann(v, delta, L):
    omega = v * math.tan(delta) / L
    return v, omega

def inverse_ackermann(v, omega, L, D, eps=1e-6):
    if abs(v) < eps or abs(omega) < eps:
        return 0.0, 0.0
    R = v / omega
    delta_L = math.atan(L / (R - D / 2.0))
    delta_R = math.atan(L / (R + D / 2.0))
    return delta_L, delta_R

def compute_icc(x, y, yaw, v, omega, eps=1e-6):
    if abs(omega) < eps:
        return None
    R = v / omega
    icc_x = x - R * math.sin(yaw)
    icc_y = y + R * math.cos(yaw)
    return icc_x, icc_y, R

BODY_LEN = 0.5
BODY_WID = 0.3
WHEELBASE = 0.4
WHEEL_RADIUS = 0.05
V_BASE = 3.0
MAX_STEER = math.radians(30)

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])
p.loadURDF("plane.urdf")

body_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[BODY_LEN / 2, BODY_WID / 2, 0.05], rgbaColor=[1, 1, 0, 1])
body_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[BODY_LEN / 2, BODY_WID / 2, 0.05])
robot = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=body_col, baseVisualShapeIndex=body_visual, basePosition=[0, 0, 0.05])

wheel_radius = WHEEL_RADIUS
wheel_len = 0.02
wheel_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=wheel_radius, length=wheel_len, rgbaColor=[1, 0.5, 0, 1])
wheel_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=wheel_radius, height=wheel_len)

wheel_positions = [
    [-WHEELBASE/2, -BODY_WID/2, wheel_radius],
    [-WHEELBASE/2,  BODY_WID/2, wheel_radius],
    [ WHEELBASE/2, -BODY_WID/2, wheel_radius],
    [ WHEELBASE/2,  BODY_WID/2, wheel_radius],
]

wheels = []
for wp in wheel_positions:
    wid = p.createMultiBody(baseMass=0.2, baseCollisionShapeIndex=wheel_col, baseVisualShapeIndex=wheel_visual, basePosition=wp)
    wheels.append(wid)

camera_distance = 5
camera_yaw = 45
camera_pitch = -30
camera_update_rate = 1

x, y, yaw = 0.0, 0.0, 0.0
steer = 0.0

timestep = 1/60
p.setTimeStep(timestep)
p.setGravity(0, 0, -9.81)

v_slider = p.addUserDebugParameter("v", -V_BASE, V_BASE, 0.5)
steer_slider = p.addUserDebugParameter("steer (deg)", -30, 30, 0)

icc_line_id = None

circle_ids = []
last_icc_key = None
redraw_every_n_frames = 2
frame_count = 0
circle_segments = 18

while True:
    frame_count += 1

    v_in = p.readUserDebugParameter(v_slider)
    steer_deg = p.readUserDebugParameter(steer_slider)
    steer = math.radians(steer_deg)

    v, omega = forward_ackermann(v_in, steer, WHEELBASE)
    delta_L, delta_R = inverse_ackermann(v, omega, WHEELBASE, BODY_WID)

    x += v * math.cos(yaw) * timestep
    y += v * math.sin(yaw) * timestep
    yaw += omega * timestep

    quat = p.getQuaternionFromEuler([0, 0, yaw])
    p.resetBasePositionAndOrientation(robot, [x, y, 0.05], quat)

    for i, wid in enumerate(wheels):
        wx = x + wheel_positions[i][0] * math.cos(yaw) - wheel_positions[i][1] * math.sin(yaw)
        wy = y + wheel_positions[i][0] * math.sin(yaw) + wheel_positions[i][1] * math.cos(yaw)

        if i == 2:
            rot = yaw + delta_L
        elif i == 3:
            rot = yaw + delta_R
        else:
            rot = yaw

        quat_w = p.getQuaternionFromEuler([math.pi / 2, 0, rot])
        p.resetBasePositionAndOrientation(wid, [wx, wy, wheel_radius], quat_w)

    icc_data = compute_icc(x, y, yaw, v, omega)
    if icc_data is not None:
        icc_x, icc_y, R = icc_data

        if icc_line_id is not None:
            p.removeUserDebugItem(icc_line_id)
        icc_line_id = p.addUserDebugLine(
            [x - (WHEELBASE/2)*math.cos(yaw), y - (WHEELBASE/2)*math.sin(yaw), 0.05],
            [icc_x, icc_y, 0.05],
            [0, 1, 0],
            2
        )

        icc_key = (round(icc_x, 3), round(icc_y, 3), round(R, 3))
        need_redraw = (icc_key != last_icc_key) and (frame_count % redraw_every_n_frames == 0)

        if need_redraw:
            for cid in circle_ids:
                p.removeUserDebugItem(cid)
            circle_ids = []

            circle_points = []
            for a in range(0, 360, int(360 / circle_segments)):
                ang = math.radians(a)
                cx = icc_x + R * math.cos(ang)
                cy = icc_y + R * math.sin(ang)
                circle_points.append([cx, cy, 0.05])

            n = len(circle_points)
            for i in range(n):
                cid = p.addUserDebugLine(circle_points[i], circle_points[(i+1) % n], [1, 0, 0], 1)
                circle_ids.append(cid)

            last_icc_key = icc_key

    else:
        if icc_line_id is not None:
            p.removeUserDebugItem(icc_line_id)
            icc_line_id = None
        for cid in circle_ids:
            p.removeUserDebugItem(cid)
        circle_ids = []
        last_icc_key = None

    if frame_count % camera_update_rate == 0:
        p.resetDebugVisualizerCamera(
            cameraDistance=camera_distance,
            cameraYaw=camera_yaw,
            cameraPitch=camera_pitch,
            cameraTargetPosition=[x, y, 0.05]
        )

    p.stepSimulation()
    time.sleep(1/60)
