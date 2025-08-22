import os
import sys

import numpy as np

# Add the parent directory to the system path
config_dir = os.path.abspath(os.path.join(__file__, "../../.."))
sys.path.append(config_dir)

from config import HIP_KNEE_LENGTH
from config import KNEE_FOOT_LENGTH


# Compute offset for (0, 0) at midpoint angles
def forward_kinematics(knee_joint_angle, hip_leg_joint_angle):
    theta1 = np.radians(hip_leg_joint_angle)  # Hip angle from X-axis
    theta2 = np.radians(knee_joint_angle)  # Inner knee angle

    # Position of knee
    x1 = HIP_KNEE_LENGTH * np.cos(theta1)
    y1 = HIP_KNEE_LENGTH * np.sin(theta1)

    # Angle of the lower leg relative to horizontal
    theta_lower = theta1 + theta2 - np.pi

    # Position of foot
    x2 = x1 + KNEE_FOOT_LENGTH * np.cos(theta_lower)
    y2 = y1 + KNEE_FOOT_LENGTH * np.sin(theta_lower)

    return x1, y1, x2, y2


# ==== Inverse Kinematics ====
def inverse_kinematics(x, y):
    # Convert display coordinates back to original coordinates
    x_orig = x
    y_orig = y

    # Distance from origin to target
    L = np.hypot(x_orig, y_orig)

    # Check reachability
    if L > HIP_KNEE_LENGTH + KNEE_FOOT_LENGTH or L < abs(
        HIP_KNEE_LENGTH - KNEE_FOOT_LENGTH
    ):
        return None, None

    # Law of cosines for knee angle (inner angle)
    cos_knee = (HIP_KNEE_LENGTH**2 + KNEE_FOOT_LENGTH**2 - L**2) / (
        2 * HIP_KNEE_LENGTH * KNEE_FOOT_LENGTH
    )
    cos_knee = np.clip(cos_knee, -1.0, 1.0)
    theta_knee = np.arccos(cos_knee)

    # Law of cosines for angle between thigh and target line
    cos_phi = (HIP_KNEE_LENGTH**2 + L**2 - KNEE_FOOT_LENGTH**2) / (
        2 * HIP_KNEE_LENGTH * L
    )
    cos_phi = np.clip(cos_phi, -1.0, 1.0)
    phi = np.arccos(cos_phi)
    # Angle from knee to target point
    angle_to_knee = np.arctan2(y_orig, x_orig) - theta_knee

    # Angle from origin to target point
    angle_to_target = np.arctan2(y_orig, x_orig)

    # Hip angle is angle to target minus φ
    theta_hip = angle_to_target + phi

    # Convert to degrees
    theta_knee_deg = np.degrees(theta_knee)
    theta_hip_deg = np.degrees(theta_hip)

    return theta_knee_deg, theta_hip_deg
