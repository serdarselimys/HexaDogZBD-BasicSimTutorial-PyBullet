"""
HexaDog ZBD - Joint & Physics Testbench
=======================================

A stripped-down companion to the full control script.

No inverse kinematics, no gait library, no gamepad. The robot is spawned in the
air on a fixed base and each joint sweeps back and forth between two angles, so
you can see exactly what each simulation setting does without a walking gait
hiding the effect.

Everything you might want to change lives in the CONFIG block below.
Run it, edit one number, run it again.

    pip install pybullet numpy
    python hexadog_joint_testbench.py
"""

import math
import time

import numpy as np
import pybullet as p
import pybullet_data

# ============================================================
# CONFIG - everything you need to touch is in this block
# ============================================================

URDF_PATH = "HexaDog_ZBD.urdf"


# ------------------------------------------------------------
# PER-JOINT SWEEP
# ------------------------------------------------------------
# One line per joint:
#
#     "JOINT_NAME": (min_deg, max_deg, period_seconds, phase),
#
#   min_deg / max_deg  one end of the travel and the other
#   period_seconds     time for one full there-and-back cycle
#   phase              0.0 to 1.0, offsets where this joint starts
#                      in its cycle. Leave every joint at 0.0 and
#                      they all move together.
#
# Joint naming on HexaDog: <leg><number>
#   legs    FL ML RL  (left)   FR MR RR  (right)
#   number  1 = shin/knee   2 = thigh   3 = hip
#
# Any joint in the URDF that is not listed here is held still.
# Set a joint to None to keep it in the list but hold it still.

JOINT_SWEEPS = {

    # --- Front right ---
    "FR1": (-30.0, 30.0, 2.0, 0.0),
    "FR2": (-30.0, 30.0, 2.0, 0.0),
    "FR3": (-75.0, 10.0, 2.0, 0.0),

    # --- Front left ---
    "FL1": (30.0, -30.0, 2.0, 0.0),
    "FL2": (30.0, -30.0, 2.0, 0.0),
    "FL3": (75.0, -10.0, 2.0, 0.0),
    
        # --- Middle right ---
    "MR1": (-30.0, 30.0, 2.0, 0.0),
    "MR2": (-30.0, 30.0, 2.0, 0.0),
    "MR3": (-75.0, 10.0, 2.0, 0.0),

    # --- Middle left ---
    "ML1": (30.0, -30.0, 2.0, 0.0),
    "ML2": (30.0, -30.0, 2.0, 0.0),
    "ML3": (75.0, -10.0, 2.0, 0.0),

    # --- Rear right ---
    "RR1": (-30.0, 30.0, 2.0, 0.0),
    "RR2": (-30.0, 30.0, 2.0, 0.0),
    "RR3": (-75.0, 10.0, 2.0, 0.0),

    # --- Rear left ---
    "RL1": (30.0, -30.0, 2.0, 0.0),
    "RL2": (30.0, -30.0, 2.0, 0.0),
    "RL3": (75.0, -10.0, 2.0, 0.0),
}

# Only sweep joints whose names contain one of these strings.
# Empty list = use everything above. e.g. ["FL"] to isolate the front-left leg.
JOINT_FILTER = []


# ------------------------------------------------------------
# WORLD
# ------------------------------------------------------------
GRAVITY = -9.81           # m/s^2. Try 0.0 to see joint behaviour with no load.
TIMESTEP = 1.0 / 240.0    # seconds per physics step. Bigger = faster but sloppier.
FIXED_BASE = True         # True holds the robot in the air. False drops it.
SPAWN_HEIGHT = 0.35       # metres


# ------------------------------------------------------------
# CONTACT DYNAMICS
# ------------------------------------------------------------
# These do nothing while FIXED_BASE is True and the robot never touches the
# ground. Set FIXED_BASE = False to feel them.
GROUND_FRICTION = 1.0     # lateral friction of the plane
ROBOT_FRICTION = 1.0      # lateral friction of every robot link
RESTITUTION = 0.0         # bounciness, 0.0 to 1.0
LINEAR_DAMPING = 0.04     # resists straight-line motion of the links
ANGULAR_DAMPING = 0.04    # resists rotation of the links


# ------------------------------------------------------------
# MOTOR MODEL - match these to your real servos
# ------------------------------------------------------------
MAX_TORQUE = 1.8          # N-m. Roughly your servo's stall torque.
MAX_VELOCITY = 6.0        # rad/s. How fast a joint is allowed to travel.
POSITION_GAIN = 0.3       # how hard the joint pulls toward its target
VELOCITY_GAIN = 1.0       # how much it resists overshooting


# ------------------------------------------------------------
# DISPLAY
# ------------------------------------------------------------
USE_GUI = True
PRINT_TRACKING_ERROR = True
PRINT_EVERY = 1.0         # seconds between console lines


# ============================================================
# SETUP
# ============================================================

p.connect(p.GUI if USE_GUI else p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, GRAVITY)
p.setTimeStep(TIMESTEP)

# A clean view of the robot, no side panels.
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.resetDebugVisualizerCamera(
    cameraDistance=0.8, cameraYaw=50, cameraPitch=-25, cameraTargetPosition=[0, 0, 0.2]
)

plane_id = p.loadURDF("plane.urdf")
p.changeDynamics(plane_id, -1, lateralFriction=GROUND_FRICTION, restitution=RESTITUTION)

robot_id = p.loadURDF(
    URDF_PATH,
    [0, 0, SPAWN_HEIGHT],
    useFixedBase=FIXED_BASE,
    flags=p.URDF_USE_INERTIA_FROM_FILE | p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES,
)

# Apply contact dynamics to the base and every link.
for link in range(-1, p.getNumJoints(robot_id)):
    p.changeDynamics(
        robot_id,
        link,
        lateralFriction=ROBOT_FRICTION,
        restitution=RESTITUTION,
        linearDamping=LINEAR_DAMPING,
        angularDamping=ANGULAR_DAMPING,
    )


# ============================================================
# JOINT DISCOVERY
# ============================================================

joints = []
urdf_names = []

print("\n--- Joints found in URDF ---")
for j in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, j)
    name = info[1].decode()
    jtype = info[2]
    urdf_names.append(name)

    if jtype not in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
        print(f"  {name:<10} skipped, fixed joint")
        continue

    if JOINT_FILTER and not any(f in name for f in JOINT_FILTER):
        print(f"  {name:<10} skipped, filtered out")
        continue

    if name not in JOINT_SWEEPS or JOINT_SWEEPS[name] is None:
        # Not configured above, so hold it at zero.
        p.resetJointState(robot_id, j, 0.0)
        joints.append(
            {"name": name, "index": j, "lo": 0.0, "hi": 0.0, "period": 1.0, "phase": 0.0}
        )
        print(f"  {name:<10} held still")
        continue

    sweep = JOINT_SWEEPS[name]

    lo_deg, hi_deg, period, phase = sweep
    lo, hi = math.radians(lo_deg), math.radians(hi_deg)

    # Respect the limits you set back in Fusion 360 and clamp to them.
    urdf_lo, urdf_hi = info[8], info[9]
    note = ""
    if urdf_lo < urdf_hi:
        clamped_lo = float(np.clip(lo, urdf_lo, urdf_hi))
        clamped_hi = float(np.clip(hi, urdf_lo, urdf_hi))
        if (clamped_lo, clamped_hi) != (lo, hi):
            note = (
                f"  <- clamped to URDF limits "
                f"[{math.degrees(urdf_lo):.0f}, {math.degrees(urdf_hi):.0f}]"
            )
        lo, hi = clamped_lo, clamped_hi
    else:
        note = "  <- continuous joint, no limits in URDF"

    joints.append(
        {
            "name": name,
            "index": j,
            "lo": lo,
            "hi": hi,
            "period": max(float(period), 0.05),
            "phase": float(phase),
        }
    )
    p.resetJointState(robot_id, j, (lo + hi) / 2.0)

    print(
        f"  {name:<10} {math.degrees(lo):6.1f} to {math.degrees(hi):6.1f} deg  "
        f"every {period:.2f}s  phase {phase:.2f}{note}"
    )

# Warn about names in the config that the URDF does not actually have.
unknown = [n for n in JOINT_SWEEPS if n not in urdf_names]
if unknown:
    print(f"\n  Warning: not found in URDF, ignored -> {', '.join(unknown)}")

print(f"\n--- {len(joints)} joint(s) configured ---\n")

if not joints:
    print("Nothing to sweep. Check URDF_PATH and JOINT_FILTER.")
    p.disconnect()
    raise SystemExit


# ============================================================
# MAIN LOOP
# ============================================================

sim_time = 0.0
next_print = PRINT_EVERY

print("Running. Ctrl+C in this terminal to stop.\n")

try:
    while True:
        for joint in joints:
            # Smooth 0 -> 1 -> 0 blend over this joint's own period.
            cycles = sim_time / joint["period"] + joint["phase"]
            blend = 0.5 * (1.0 - math.cos(2.0 * math.pi * cycles))

            target = joint["lo"] + blend * (joint["hi"] - joint["lo"])
            p.setJointMotorControl2(
                robot_id,
                joint["index"],
                p.POSITION_CONTROL,
                targetPosition=target,
                force=MAX_TORQUE,
                maxVelocity=MAX_VELOCITY,
                positionGain=POSITION_GAIN,
                velocityGain=VELOCITY_GAIN,
            )
            joint["target"] = target

        p.stepSimulation()
        sim_time += TIMESTEP
        time.sleep(TIMESTEP)

        if PRINT_TRACKING_ERROR and sim_time >= next_print:
            next_print += PRINT_EVERY
            errors = []
            for joint in joints:
                actual = p.getJointState(robot_id, joint["index"])[0]
                errors.append(abs(math.degrees(joint["target"] - actual)))
            worst = joints[int(np.argmax(errors))]["name"]
            print(
                f"t={sim_time:6.1f}s | tracking error  "
                f"mean {np.mean(errors):5.2f} deg   max {np.max(errors):5.2f} deg ({worst})  "
                f"| torque {MAX_TORQUE:.2f}  vel {MAX_VELOCITY:.2f}  kp {POSITION_GAIN:.2f}"
            )

except KeyboardInterrupt:
    pass
finally:
    p.disconnect()
