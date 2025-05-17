#!/usr/bin/env python3
"""
humanoid_sim.py – 17-DOF mini-humanoid simulation
-------------------------------------------------
fixed-base + stronger joint torques so the robot *stands* while you iterate!

Keys (focus the Bullet window)
 ⏎ / r : reset to standing_position
 w     : tiny walk demo (works even with fixed base)
 q / Esc : quit
"""

import math, os, time, tempfile, pathlib
from collections import OrderedDict
import pybullet as p, pybullet_data as pd

# ────────────────────────────────────────────────────────────────────────────
# 1.  REAL-ROBOT SERVO TABLES
# ────────────────────────────────────────────────────────────────────────────
standing_position = {
    0: 6000, 1: 6000, 2: 5700, 3: 5700,  # Ankles
    4: 6700, 5: 6700,                    # Knees
    6: 6000, 7: 6000,                    # Hip Tilts
    8: 5500, 9: 5500,                    # Hips F/B
    10: 8650, 11: 8650,                  # Shoulder Abduction (arms out)
    12: 7500, 13: 7500,                  # Elbows
    14: 6500, 15: 6500,                  # Shoulder Pitch (arms fwd)
    16: 6000,                            # Head
}
channel_to_joint_map = OrderedDict([
    (0,"LAT"),(1,"RAT"),(2,"LA"),(3,"RA"),(4,"LK"),(5,"RK"),
    (6,"LHT"),(7,"RHT"),(8,"LH"),(9,"RH"),(10,"LS"),(11,"RS"),
    (12,"LE"),(13,"RE"),(14,"LP"),(15,"RP"),(16,"HEAD"),
])

# µs  →  rad
RANGE_U, RANGE_RAD = 2000, math.radians(60)
maestro_u_to_rad = lambda u, c=6000: (u-c)/RANGE_U*RANGE_RAD


def maestro_u_to_rad(µs: int,
                     centre: int = 6000,
                     us_range: int = RANGE_U,
                     rad_range: float = RANGE_RAD) -> float:
    return (µs - centre) / us_range * rad_range


# ────────────────────────────────────────────────────────────────────────────
# 3.  QUICK-AND-CLEAN URDF GENERATOR (unchanged)
# ────────────────────────────────────────────────────────────────────────────
def _link(name: str, geom: str, mass=0.05) -> str:
    return f"""  <link name="{name}">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="{mass}"/>
      <inertia ixx="3e-4" ixy="0" ixz="0"
               iyy="3e-4" iyz="0" izz="3e-4"/>
    </inertial>
    <visual><origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>{geom}</geometry>
      <material name="dark"><color rgba="0.15 0.15 0.15 1"/></material>
    </visual>
    <collision><geometry>{geom}</geometry></collision>
  </link>"""

def _rev_joint(name, parent, child, xyz, axis,
               low=-math.pi/2, high=math.pi/2,
               effort=2, vel=4) -> str:
    x,y,z   = xyz
    ax,ay,az = axis
    return f"""  <joint name="{name}" type="revolute">
    <parent link="{parent}"/><child link="{child}"/>
    <origin xyz="{x:.3f} {y:.3f} {z:.3f}" rpy="0 0 0"/>
    <axis xyz="{ax} {ay} {az}"/>
    <limit lower="{low:.4f}" upper="{high:.4f}"
           effort="{effort}" velocity="{vel}"/>
  </joint>"""

def generate_urdf(path: str) -> None:
    cyl_leg   = '<cylinder radius="0.015" length="0.08"/>'
    cyl_arm   = '<cylinder radius="0.012" length="0.07"/>'
    cube_tiny = '<box size="0.04 0.04 0.04"/>'
    cube_big  = '<box size="0.06 0.04 0.10"/>'
    foot_box  = '<box size="0.07 0.04 0.02"/>'
    head_sph  = '<sphere radius="0.035"/>'

    hip_y, hip_z, leg_dz = 0.04, 0.0, -0.08
    urdf = ['<?xml version="1.0"?>', '<robot name="miniHumanoid">']

    # Pelvis → torso → head
    urdf += [_link("pelvis", cube_big),
             _link("torso", cube_big),
             _rev_joint("TORSO", "pelvis", "torso", (0,0,0.10), (1,0,0)),
             _link("head_link", head_sph),
             _rev_joint("HEAD", "torso", "head_link", (0,0,0.12), (0,1,0))]

    # Legs
    def add_leg(side):
        s = 1 if side=="L" else -1
        parent = "pelvis"
        chain = [
            (f"{side}H",  (0, s*hip_y, hip_z),          (1,0,0), cube_tiny),
            (f"{side}HT", (0, 0,       leg_dz/2),       (0,0,1), cyl_leg),
            (f"{side}K",  (0, 0,       leg_dz/2),       (0,0,1), cyl_leg),
            (f"{side}A",  (0, 0,       leg_dz/2),       (0,0,1), cyl_leg),
            (f"{side}AT", (0, 0,      -0.045),          (1,0,0), foot_box)]
        for j, off, ax, geom in chain:
            child = j+"_link"
            urdf.append(_link(child, geom))
            urdf.append(_rev_joint(j, parent, child, off, ax))
            parent = child
    add_leg("L"); add_leg("R")

    # Arms
    def add_arm(side):
        s = 1 if side=="L" else -1
        parent = "torso"
        chain = [
            (f"{side}S", (0.03, s*hip_y, 0.10), (0,0,1), cyl_arm),
            (f"{side}E", (0,     0,     -0.07), (0,0,1), cyl_arm),
            (f"{side}P", (0,     0,     -0.07), (1,0,0), cube_tiny)]
        for j, off, ax, geom in chain:
            child = j+"_link"
            urdf.append(_link(child, geom))
            urdf.append(_rev_joint(j, parent, child, off, ax))
            parent = child
    add_arm("L"); add_arm("R")

    urdf.append('</robot>')
    pathlib.Path(path).write_text("\n".join(urdf))
    print(f"[URDF]  written → {path}")


# ────────────────────────────────────────────────────────────────────────────
# 4.  SIM WRAPPER
# ────────────────────────────────────────────────────────────────────────────
class MiniHumanoidSim:
    def __init__(self):
        tmp = os.path.join(tempfile.gettempdir(),"miniHumanoid_auto.urdf")
        if not os.path.exists(tmp): generate_urdf(tmp)

        p.connect(p.GUI)
        p.setAdditionalSearchPath(pd.getDataPath())
        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        p.resetDebugVisualizerCamera(1.4,40,-20,[0,0,0.25])
        p.setGravity(0,0,-9.81)
        p.loadURDF("plane.urdf")

        # **useFixedBase=True** -> cannot topple
        self.robot = p.loadURDF(tmp, basePosition=(0,0,0.30),
                                useFixedBase=True,   #  ←←   key change
                                flags=p.URDF_MAINTAIN_LINK_ORDER)

        self.name_to_id = {p.getJointInfo(self.robot,i)[1].decode(): i
                           for i in range(p.getNumJoints(self.robot))}
        self.apply_servo_dict(standing_position, immediate=True)

    def apply_servo_dict(self, pose, speed=5, immediate=False):
        for ch,µs in pose.items():
            jid = self.name_to_id[channel_to_joint_map[ch]]
            rad = maestro_u_to_rad(µs)
            if immediate: p.resetJointState(self.robot,jid,rad)
            p.setJointMotorControl2(self.robot,jid,p.POSITION_CONTROL,rad,
                                    maxVelocity=speed, force=30)  #  ←← strong!

    step = lambda self, rt=True: (p.stepSimulation(),
                                  time.sleep(1/240) if rt else None)


# ────────────────────────────────────────────────────────────────────────────
# 5.  TINY WALK DEMO (unchanged)
# ────────────────────────────────────────────────────────────────────────────
def demo_walk(sim, n=2):
    leanL, leanR = {8:7000,9:4700}, {8:5900,9:5600}
    bendL, bendR = {4:4400,6:6200}, {5:5000,7:4800}
    for _ in range(n):
        sim.apply_servo_dict({**leanL,**bendL},3); [sim.step() for _ in range(80)]
        sim.apply_servo_dict(standing_position);    [sim.step() for _ in range(80)]
        sim.apply_servo_dict({**leanR,**bendR},3); [sim.step() for _ in range(80)]
        sim.apply_servo_dict(standing_position);    [sim.step() for _ in range(80)]

# ────────────────────────────────────────────────────────────────────────────
# 6.  MAIN LOOP  – with robust key fall-backs
# ────────────────────────────────────────────────────────────────────────────
def main():
    sim = MiniHumanoidSim()
    KEY_ESC  = getattr(p,"B3G_ESCAPE",27)
    KEY_ENT  = getattr(p,"B3G_RETURN",13)

    print("\nKeys: ⏎/r reset • w walk • q/Esc quit\n")
    while True:
        keys = p.getKeyboardEvents()
        if (KEY_ENT in keys and keys[KEY_ENT]&p.KEY_WAS_TRIGGERED) or \
           (ord('r')in keys and keys[ord('r')]&p.KEY_WAS_TRIGGERED):
            sim.apply_servo_dict(standing_position); print("[reset]")
        if ord('w') in keys and keys[ord('w')]&p.KEY_WAS_TRIGGERED:
            print("[walk]"); demo_walk(sim)
        if (ord('q') in keys and keys[ord('q')]&p.KEY_WAS_TRIGGERED) or \
           (KEY_ESC in keys and keys[KEY_ESC]&p.KEY_WAS_TRIGGERED):
            break
        sim.step()
    p.disconnect(); print("bye")

if __name__ == "__main__":
    main()
