import copy
import os
import sys
import time
from datetime import datetime
from math import comb

import numpy as np

# Add paths
config_dir = os.path.abspath(os.path.join(__file__, "../../../.."))
sys.path.append(config_dir)
libraries_dir = os.path.abspath(os.path.join(__file__, "../../../../libraries/python"))
sys.path.append(libraries_dir)

from kinematics import inverse_kinematics
from serial_comm import SerialComm

from config import BAUD_RATE_REC
from config import BAUD_RATE_SEND
from config import DELAY
from config import FOOT_POSITIONS_WALK
from config import HIP_KNEE_LENGTH
from config import INTERPOLATION_STEPS_X
from config import INTERPOLATION_STEPS_Y
from config import KNEE_FOOT_LENGTH
from config import NUM_LEGS
from config import SERIAL_PORT_REC
from config import SERIAL_PORT_SEND
from config import STEP_LENGTH_X
from config import STEP_LENGTH_Y

try:
    ser = SerialComm(SERIAL_PORT_SEND, BAUD_RATE_SEND, timeout=0.01)
    time.sleep(2)
except Exception as e:
    ser = None
    print(f"⚠️ Could not connect to motor serial port: {e}")


class WalkingMechanism:

    def __init__(
        self,
        arm1,
        arm2,
        start,
        step_lengthX,
        step_lengthY,
        offset,
        inter_stepsX,
        inter_stepsY,
    ):
        self.arm1 = arm1
        self.arm2 = arm2
        self.x1, self.y1 = start[0], start[1]
        self.step_lengthX = step_lengthX
        self.step_lengthY = step_lengthY
        self.swing_steps = inter_stepsX
        self.stance_steps = inter_stepsY
        self.gait_cycle_len = self.swing_steps + self.stance_steps
        self.offset = offset
        self.trajectory = self.elliptical_path()

    def elliptical_path(self):
        a = self.step_lengthX / 2
        b = self.step_lengthY
        swing_theta = np.linspace(np.pi, 0, self.swing_steps)
        swing_x = self.x1 + a * np.cos(swing_theta)
        swing_y = self.y1 + b * np.sin(swing_theta)

        stance_theta = np.linspace(0, -np.pi, self.stance_steps)
        stance_x = self.x1 + a * np.cos(stance_theta)
        stance_y = self.y1 + b * np.sin(stance_theta)

        x = np.concatenate([swing_x, stance_x])
        y = np.concatenate([swing_y, stance_y])
        return x, y

    def ik(self, x, y, pX=0, pY=0):
        d = np.hypot(x, y)
        if d > (self.arm1 + self.arm2) or d < abs(self.arm1 - self.arm2):
            return None
        cos_theta2 = (x**2 + y**2 - self.arm1**2 - self.arm2**2) / (
            2 * self.arm1 * self.arm2
        )
        cos_theta2 = np.clip(cos_theta2, -1, 1)
        sin_theta2 = np.sqrt(1 - cos_theta2**2)
        sin_theta2 = -sin_theta2
        theta2 = np.arctan2(sin_theta2, cos_theta2)
        k1 = self.arm1 + self.arm2 * cos_theta2
        k2 = self.arm2 * sin_theta2
        theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

        theta1 = int(np.degrees(theta1))
        theta2 = int(180 + np.degrees(theta2))
        return theta1, theta1 + theta2


class LegMovement:

    def __init__(self, legs, fp, steps):
        self.legs = legs
        self.foot_positions = fp
        self.steps = steps * 8  # total steps for full cycle (2 swings * 4 legs)
        self.generate_angles()

    def generate_angles(self):
        self.angles = []

        for step in range(self.steps):  # move one leg at a time
            active_leg = step % 4
            command = []

            for i in range(4):
                if i == active_leg:
                    leg = self.legs[i]
                    x_path, y_path = leg.trajectory
                    idx = (
                        step // 4 + leg.offset * leg.swing_steps
                    ) % leg.gait_cycle_len
                    x, y = x_path[idx], y_path[idx]
                else:
                    x, y = self.foot_positions[i]

                ik_result = self.legs[i].ik(x, y)
                if ik_result is None:
                    command.append(["0", "0", "0"])
                else:
                    a, b = ik_result
                    command.append([f"{int(b)}", f"{int(a)}", "0"])

            command_flat = [angle for leg_angles in command for angle in leg_angles]
            self.angles.append(f"<{','.join(command_flat)}>")

    def start(self, delay):
        self.generate_angles()
        time.sleep(2)
        steps = 0

        while True:
            command = self.angles[steps % len(self.angles)]
            if ser and ser.is_open:
                try:
                    ser.write_line(command.encode())
                    now = datetime.now()
                    minute = now.strftime("%M")
                    second = now.strftime("%S")
                    millisecond = int(now.microsecond / 1000)
                    print(f"Sent: {minute}:{second}:{millisecond}:- {command}")
                except Exception as e:
                    print(f"Serial error: {e}")
            else:
                print(f"Error: Serial port {SERIAL_PORT_SEND} not available")

            time.sleep(0.1)
            steps += 1


# Initialize WalkingMechanisms for all 4 legs
wm = [
    WalkingMechanism(
        HIP_KNEE_LENGTH,
        KNEE_FOOT_LENGTH,
        FOOT_POSITIONS_WALK[0],
        STEP_LENGTH_X,
        STEP_LENGTH_Y,
        0,
        INTERPOLATION_STEPS_X,
        INTERPOLATION_STEPS_Y,
    ),
    WalkingMechanism(
        HIP_KNEE_LENGTH,
        KNEE_FOOT_LENGTH,
        FOOT_POSITIONS_WALK[1],
        STEP_LENGTH_X,
        STEP_LENGTH_Y,
        1,
        INTERPOLATION_STEPS_X,
        INTERPOLATION_STEPS_Y,
    ),
    WalkingMechanism(
        HIP_KNEE_LENGTH,
        KNEE_FOOT_LENGTH,
        FOOT_POSITIONS_WALK[2],
        STEP_LENGTH_X,
        STEP_LENGTH_Y,
        1,
        INTERPOLATION_STEPS_X,
        INTERPOLATION_STEPS_Y,
    ),
    WalkingMechanism(
        HIP_KNEE_LENGTH,
        KNEE_FOOT_LENGTH,
        FOOT_POSITIONS_WALK[3],
        STEP_LENGTH_X,
        STEP_LENGTH_Y,
        0,
        INTERPOLATION_STEPS_X,
        INTERPOLATION_STEPS_Y,
    ),
]

# Start the Leg Movement
Lm = LegMovement(wm, FOOT_POSITIONS_WALK, INTERPOLATION_STEPS_X)
Lm.start(DELAY)
