import multiprocessing as mp
import os
import sys
import time
import tkinter as tk
from tkinter import ttk

# Add paths
config_dir = os.path.abspath(os.path.join(__file__, "../../../.."))
sys.path.append(config_dir)
libraries_dir = os.path.abspath(os.path.join(__file__, "../../../../libraries/python"))
sys.path.append(libraries_dir)

from kinematics import inverse_kinematics
from pid_leg_stabilizer import PIDLegStabilizer
from serial_comm import SerialComm

from config import BAUD_RATE_REC
from config import BAUD_RATE_SEND
from config import FOOT_POSITIONS_REST
from config import NUM_LEGS
from config import SERIAL_PORT_REC
from config import SERIAL_PORT_SEND


def pid_process(shared_vars, stop_event):
    imu_uc = SerialComm(SERIAL_PORT_REC, BAUD_RATE_REC)
    motor_uc = SerialComm(SERIAL_PORT_SEND, BAUD_RATE_SEND)
    pid_leg = PIDLegStabilizer()
    pid_leg.set_setpoint(0.0, 0.0)
    foot_positions_rest = FOOT_POSITIONS_REST
    foot_positions = [pos.copy() for pos in foot_positions_rest]

    loop_counter = 0
    last_print_time = time.time()

    freq = shared_vars.freq if shared_vars.freq > 0 else 50
    dt = 1.0 / freq
    next_time = time.perf_counter()

    while not stop_event.is_set():
        freq = shared_vars.freq if shared_vars.freq > 0 else 50
        dt = 1.0 / freq

        # Reset PID state and foot positions on transition to enabled
        if shared_vars.pid_enabled and not shared_vars.prev_pid_enabled:
            print("[PID] Re-enabled: Resetting controller state and leg positions.")
            shared_vars.prev_pid_enabled = True
            pid_leg.reset()
            foot_positions = [pos.copy() for pos in foot_positions_rest]

        # Reset foot positions on transition to disabled
        if not shared_vars.pid_enabled and shared_vars.prev_pid_enabled:
            print("[PID] Disabled: Setting legs to initial positions.")
            shared_vars.prev_pid_enabled = False
            foot_positions = [pos.copy() for pos in foot_positions_rest]

        pid_leg.kp_pitch = shared_vars.kp_pitch
        pid_leg.ki_pitch = shared_vars.ki_pitch
        pid_leg.kd_pitch = shared_vars.kd_pitch
        pid_leg.kp_roll = shared_vars.kp_roll
        pid_leg.ki_roll = shared_vars.ki_roll
        pid_leg.kd_roll = shared_vars.kd_roll

        if shared_vars.pid_enabled:
            try:
                imu_uc.ser.reset_input_buffer()
                time.sleep(0.002)
                ypr_data = imu_uc.read_line().strip()
                if ypr_data.startswith("<") and ypr_data.endswith(">"):
                    yaw, pitch, roll = map(float, ypr_data[1:-1].split(","))
                    print(f"[IMU] Yaw: {yaw:.2f}, Pitch: {pitch:.2f}, Roll: {roll:.2f}")
                else:
                    raise ValueError("Bad IMU data")
            except Exception as e:
                print(f"[WARN] IMU read error: {e}")
                next_time += dt
                sleep_time = next_time - time.perf_counter()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                continue
        else:
            pitch, roll = 0.0, 0.0

        deltas = (
            pid_leg.compute_leg_deltas(pitch, roll, dt)
            if shared_vars.pid_enabled
            else [0, 0, 0, 0]
        )
        angle_values = []
        ik_failed = False

        for leg in range(NUM_LEGS):
            x, y = foot_positions[leg]
            if shared_vars.pid_enabled:
                new_y = y + deltas[leg]
                foot_positions[leg][1] = new_y
            else:
                new_y = foot_positions_rest[leg][1]

            try:
                knee, hip = inverse_kinematics(x, new_y)
                if knee is None or hip is None:
                    raise ValueError("IK failed")
            except Exception as e:
                print(f"[WARN] IK leg {leg+1} failed: {e}")
                ik_failed = True
                break

            angle_values.extend([str(int(knee)), str(int(hip)), "0"])

        if not ik_failed:
            command = "<" + ",".join(angle_values) + ">"
            try:
                motor_uc.write_line(command)
            except Exception as e:
                print(f"[ERROR] Send failed: {e}")

        loop_counter += 1
        now = time.time()
        if now - last_print_time >= 1.0:
            print(
                f"[FREQ] PID Process running at {loop_counter / (now - last_print_time):.2f} Hz"
            )
            loop_counter = 0
            last_print_time = now

        next_time += dt
        sleep_time = next_time - time.perf_counter()
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            next_time = time.perf_counter()

    imu_uc.close()
    motor_uc.close()


def start_gui():
    root = tk.Tk()
    root.title("PID Tuner")

    manager = mp.Manager()
    shared_vars = manager.Namespace()
    shared_vars.kp_pitch = 0.4
    shared_vars.ki_pitch = 0.04
    shared_vars.kd_pitch = 0.04
    shared_vars.kp_roll = 0.1
    shared_vars.ki_roll = 0.01
    shared_vars.kd_roll = 0.01
    shared_vars.freq = 25.0
    shared_vars.pid_enabled = False
    shared_vars.prev_pid_enabled = False

    stop_event = mp.Event()
    pid_proc = mp.Process(target=pid_process, args=(shared_vars, stop_event))
    pid_proc.start()

    kp_pitch = tk.DoubleVar(value=shared_vars.kp_pitch)
    ki_pitch = tk.DoubleVar(value=shared_vars.ki_pitch)
    kd_pitch = tk.DoubleVar(value=shared_vars.kd_pitch)
    kp_roll = tk.DoubleVar(value=shared_vars.kp_roll)
    ki_roll = tk.DoubleVar(value=shared_vars.ki_roll)
    kd_roll = tk.DoubleVar(value=shared_vars.kd_roll)
    freq_var = tk.StringVar(value=str(shared_vars.freq))

    def create_slider(parent, label, variable, row, max_val=5.0, step=0.01):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w")
        slider = ttk.Scale(
            parent,
            variable=variable,
            from_=0.0,
            to=max_val,
            orient="horizontal",
            length=250,
        )
        slider.grid(row=row, column=1, sticky="ew")
        btn_frame = ttk.Frame(parent)
        btn_frame.grid(row=row, column=2, sticky="ns", padx=(5, 0))

        def increment():
            val = variable.get() + step
            if val > max_val:
                val = max_val
            variable.set(round(val, 4))

        def decrement():
            val = variable.get() - step
            if val < 0:
                val = 0
            variable.set(round(val, 4))

        ttk.Button(btn_frame, text="^", width=2, command=increment).pack(
            side="top", fill="x"
        )
        ttk.Button(btn_frame, text="v", width=2, command=decrement).pack(
            side="top", fill="x"
        )
        entry = ttk.Entry(parent, width=6, textvariable=variable)
        entry.grid(row=row, column=3, padx=(5, 0))

        def validate_entry(*args):
            try:
                val = float(variable.get())
                if val < 0:
                    variable.set(0.0)
                elif val > max_val:
                    variable.set(max_val)
            except Exception:
                pass

        variable.trace_add("write", validate_entry)

    frame = ttk.Frame(root, padding="10")
    frame.grid(row=0, column=0, sticky="nsew")
    root.columnconfigure(0, weight=1)

    create_slider(frame, "Kp Pitch", kp_pitch, 0, max_val=5.0, step=0.05)
    create_slider(frame, "Ki Pitch", ki_pitch, 1, max_val=1.0, step=0.01)
    create_slider(frame, "Kd Pitch", kd_pitch, 2, max_val=1.0, step=0.01)
    create_slider(frame, "Kp Roll", kp_roll, 3, max_val=5.0, step=0.05)
    create_slider(frame, "Ki Roll", ki_roll, 4, max_val=1.0, step=0.01)
    create_slider(frame, "Kd Roll", kd_roll, 5, max_val=1.0, step=0.01)

    ttk.Label(frame, text="Frequency (Hz)").grid(row=6, column=0, sticky="w")
    freq_entry = ttk.Entry(frame, textvariable=freq_var, width=10)
    freq_entry.grid(row=6, column=1, sticky="w")

    toggle_button = ttk.Button(frame, text="Resume PID")
    toggle_button.grid(row=7, column=0, columnspan=2, pady=(10, 0))

    def toggle_pid():
        shared_vars.prev_pid_enabled = shared_vars.pid_enabled
        shared_vars.pid_enabled = not shared_vars.pid_enabled
        toggle_button.config(
            text="Resume PID" if not shared_vars.pid_enabled else "Pause PID"
        )

    toggle_button.config(command=toggle_pid)

    def update_shared_vars():
        try:
            shared_vars.kp_pitch = kp_pitch.get()
            shared_vars.ki_pitch = ki_pitch.get()
            shared_vars.kd_pitch = kd_pitch.get()
            shared_vars.kp_roll = kp_roll.get()
            shared_vars.ki_roll = ki_roll.get()
            shared_vars.kd_roll = kd_roll.get()
            shared_vars.freq = float(freq_var.get())
        except Exception:
            pass
        root.after(100, update_shared_vars)

    root.after(100, update_shared_vars)

    def on_close():
        stop_event.set()
        pid_proc.join()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    mp.set_start_method("spawn")
    start_gui()
