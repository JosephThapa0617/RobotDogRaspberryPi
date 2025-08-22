import os
import sys
import time

import matplotlib.pyplot as plt
import numpy as np

# Add the parent directory to the system path
config_dir = os.path.abspath(os.path.join(__file__, "../../../.."))
sys.path.append(config_dir)

from matplotlib.widgets import Slider

from config import BAUD_RATE_REC
from config import BAUD_RATE_SEND
from config import JOINT_ANGLES_INIT
from config import NUM_LEGS
from config import SERIAL_PORT_REC
from config import SERIAL_PORT_SEND

libraries_dir = os.path.abspath(os.path.join(__file__, "../../../../libraries/python"))
sys.path.append(libraries_dir)

from kinematics import forward_kinematics
from kinematics import inverse_kinematics
from pid_leg_stabilizer import PIDLegStabilizer
from serial_comm import SerialComm

# Initialize serial communication
imu_uc = SerialComm(SERIAL_PORT_REC, BAUD_RATE_REC)
if not imu_uc.is_open():
    print(
        f"[ERROR] Could not open serial port {SERIAL_PORT_REC} for IMU communication."
    )
    sys.exit(1)

motor_uc = SerialComm(SERIAL_PORT_SEND, BAUD_RATE_SEND)
if not motor_uc.is_open():
    print(
        f"[ERROR] Could not open serial port {SERIAL_PORT_SEND} for motor communication."
    )
    sys.exit(1)

# Initialize PID stabilizer
pid_leg = PIDLegStabilizer()
pid_leg.set_setpoint(0.0, 0.0)  # desired pitch and roll setpoints


# ==== Set up Plot ====
fig, axes = plt.subplots(2, 2, figsize=(14, 12))
plt.subplots_adjust(
    left=0.05, right=0.95, top=0.95, bottom=0.35, wspace=0.2, hspace=0.2
)

lines = [[None for _ in range(2)] for _ in range(2)]
target_dots = [[None for _ in range(2)] for _ in range(2)]

for leg in range(NUM_LEGS):
    row = leg // 2
    col = leg % 2
    ax = axes[row][col]
    ax.set_aspect("equal")
    ax.set_xlim(-250, 250)
    ax.set_ylim(-400, 100)
    ax.set_title(f"Leg {leg+1}")
    (lines[row][col],) = ax.plot([], [], "o-", lw=4, markersize=8, color="blue")
    (target_dots[row][col],) = ax.plot([], [], "rx", markersize=10)

# Initial foot positions
# foot_positions = [
#     forward_kinematics(JOINT_ANGLES_INIT[0], JOINT_ANGLES_INIT[1])[2:]
#     for _ in range(NUM_LEGS)
# ]

foot_positions = [[-95, -195], [-95, -195], [95, -195], [95, -195]]

print(foot_positions)


def update_leg(leg, knee_angle, hip_leg_angle):
    x0, y0 = 0, 0
    x1, y1, x2, y2 = forward_kinematics(knee_angle, hip_leg_angle)

    row = leg // 2
    col = leg % 2
    lines[row][col].set_data([x0, x1, x2], [y0, y1, y2])
    target_dots[row][col].set_data([x2], [y2])


# === Create Sliders ===
slider_ax_kp_pitch = plt.axes([0.1, 0.25, 0.3, 0.02])
slider_ax_ki_pitch = plt.axes([0.1, 0.22, 0.3, 0.02])
slider_ax_kd_pitch = plt.axes([0.1, 0.19, 0.3, 0.02])

slider_ax_kp_roll = plt.axes([0.6, 0.25, 0.3, 0.02])
slider_ax_ki_roll = plt.axes([0.6, 0.22, 0.3, 0.02])
slider_ax_kd_roll = plt.axes([0.6, 0.19, 0.3, 0.02])

slider_kp_pitch = Slider(
    slider_ax_kp_pitch, "Kp Pitch", 0.0, 5.0, valinit=pid_leg.kp_pitch
)
slider_ki_pitch = Slider(
    slider_ax_ki_pitch, "Ki Pitch", 0.0, 1.0, valinit=pid_leg.ki_pitch
)
slider_kd_pitch = Slider(
    slider_ax_kd_pitch, "Kd Pitch", 0.0, 1.0, valinit=pid_leg.kd_pitch
)

slider_kp_roll = Slider(slider_ax_kp_roll, "Kp Roll", 0.0, 5.0, valinit=pid_leg.kp_roll)
slider_ki_roll = Slider(slider_ax_ki_roll, "Ki Roll", 0.0, 1.0, valinit=pid_leg.ki_roll)
slider_kd_roll = Slider(slider_ax_kd_roll, "Kd Roll", 0.0, 1.0, valinit=pid_leg.kd_roll)


# === Function to update PID values ===
def update_pid_gains(val):
    pid_leg.kp_pitch = slider_kp_pitch.val
    pid_leg.ki_pitch = slider_ki_pitch.val
    pid_leg.kd_pitch = slider_kd_pitch.val

    pid_leg.kp_roll = slider_kp_roll.val
    pid_leg.ki_roll = slider_ki_roll.val
    pid_leg.kd_roll = slider_kd_roll.val


# === Connect sliders to callback ===
slider_kp_pitch.on_changed(update_pid_gains)
slider_ki_pitch.on_changed(update_pid_gains)
slider_kd_pitch.on_changed(update_pid_gains)
slider_kp_roll.on_changed(update_pid_gains)
slider_ki_roll.on_changed(update_pid_gains)
slider_kd_roll.on_changed(update_pid_gains)

# ==== Main Loop ====
dt = 0.05  # 50ms
loop_counter = 0
last_print_time = time.time()

try:
    while plt.fignum_exists(fig.number):
        start_time = time.time()

        # Read and parse IMU data in format <yaw, pitch, roll>
        try:
            imu_uc.ser.reset_input_buffer()
            time.sleep(0.002)
            ypr_data = imu_uc.read_line().strip()
            if ypr_data.startswith("<") and ypr_data.endswith(">"):
                ypr_data = ypr_data[1:-1]  # remove the angle brackets
                yaw, pitch, roll = map(float, ypr_data.split(","))
                print(f"[INFO] IMU Data: Yaw={yaw}, Pitch={pitch}, Roll={roll}")
            else:
                print(f"[WARN] IMU data not in expected format: {ypr_data}")
                continue
        except Exception as e:
            print(f"[WARN] Failed to parse IMU data: {e}")
            continue

        # Compute leg deltas
        deltas = pid_leg.compute_leg_deltas(pitch, roll, dt=dt)

        # Collect all angles and send in <val1,val2,...> format
        angle_values = []
        ik_failed = False  # Track IK failures

        for leg in range(NUM_LEGS):
            # Original foot x, y
            x, y = foot_positions[leg]
            new_y = y + deltas[leg]  # Apply pitch/roll stabilization offset
            foot_positions[leg][1] = new_y  # Update y position

            # Compute IK for new position
            try:
                knee_angle, hip_leg_angle = inverse_kinematics(x, y)
                if knee_angle is None or hip_leg_angle is None:
                    raise ValueError("IK returned None")
            except Exception as e:
                print(f"[WARN] IK failed for leg {leg+1}: {e}")
                # Send safe values in case of failure
                # angle_values.extend(["0", "0", "0"])
                ik_failed = True
                break
                # continue

            # Update plot
            update_leg(leg, knee_angle, hip_leg_angle)

            # Append integer angles (can also round instead of int)
            angle_values.extend(
                [
                    str(int(knee_angle)),
                    str(int(hip_leg_angle)),
                    "0",  # Hip-Body fixed at 0
                ]
            )
        if not ik_failed:
            # Format and send to Arduino
            command = "<" + ",".join(angle_values) + ">"
            if motor_uc and motor_uc.is_open:
                try:
                    motor_uc.write_line(command)
                    print(f"[SEND] {command}")
                except Exception as e:
                    print(f"[ERROR] Sending to motor_uc failed: {e}")

        loop_counter += 1
        current_time = time.time()
        if current_time - last_print_time >= 1.0:
            frequency = loop_counter / (current_time - last_print_time)
            print(f"[FREQ] Loop running at {frequency:.2f} Hz")
            loop_counter = 0
            last_print_time = current_time

        fig.canvas.flush_events()
        plt.pause(0.001)

        # Wait remaining time if loop is faster than 10ms
        elapsed = time.time() - start_time
        if elapsed < dt:
            time.sleep(dt - elapsed)

except KeyboardInterrupt:
    print("[INFO] Interrupted by user.")


# ==== Cleanup ====
if imu_uc and imu_uc.is_open:
    imu_uc.close()
# if motor_uc and motor_uc.is_open:
#     motor_uc.close()
plt.close(fig)
