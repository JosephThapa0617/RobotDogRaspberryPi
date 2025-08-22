# from flask import Flask, render_template, request
# import sys
# from datetime import datetime
# import socket
# import time
# import threading
# import copy
# import os
# import numpy as np
# from math import comb

# # Add your robot control imports here
# # Assuming your robot files are in the same directory or adjust paths accordingly
# try:
#     # Add paths (adjust these paths to match your project structure)
#     config_dir = os.path.abspath(os.path.join(__file__))
#     sys.path.append(config_dir)
#     libraries_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "libraries", "python"))
#     sys.path.append(libraries_dir)
    
#     from kinematics import inverse_kinematics
#     from serial_comm import SerialComm
#     from config import (BAUD_RATE_REC, BAUD_RATE_SEND, DELAY, FOOT_POSITIONS_WALK,
#                        HIP_KNEE_LENGTH, INTERPOLATION_STEPS_X, INTERPOLATION_STEPS_Y,
#                        KNEE_FOOT_LENGTH, NUM_LEGS, SERIAL_PORT_REC, SERIAL_PORT_SEND,
#                        STEP_LENGTH_X, STEP_LENGTH_Y)
    
#     ROBOT_AVAILABLE = True
#     print(" Robot libraries loaded successfully!")
# except ImportError as e:
#     ROBOT_AVAILABLE = False
#     print(f"Robot libraries not available: {e}")

# app = Flask(__name__)

# robot_walking = False
# walking_thread = None

# def get_local_ip():
#     try:
#         with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
#             s.connect(("8.8.8.8", 80))
#             return s.getsockname()[0]
#     except:
#         return "Unable to determine IP"

# if ROBOT_AVAILABLE:
#     try:
#         ser = SerialComm(SERIAL_PORT_SEND, BAUD_RATE_SEND, timeout=0.01)
#         time.sleep(2)
#         print("Serial connection established")
#     except Exception as e:
#         ser = None
#         print(f"Could not connect to motor serial port: {e}")

#     class WalkingMechanism:
#         def __init__(self, arm1, arm2, start, step_lengthX, step_lengthY, offset, inter_stepsX, inter_stepsY):
#             self.arm1 = arm1
#             self.arm2 = arm2
#             self.x1, self.y1 = start[0], start[1]
#             self.step_lengthX = step_lengthX
#             self.step_lengthY = step_lengthY
#             self.swing_steps = inter_stepsX
#             self.stance_steps = inter_stepsY
#             self.gait_cycle_len = self.swing_steps + self.stance_steps
#             self.offset = offset
#             self.trajectory = self.elliptical_path()

#         def elliptical_path(self):
#             a = self.step_lengthX / 2
#             b = self.step_lengthY

#             swing_theta = np.linspace(np.pi, 0, self.swing_steps)
#             swing_x = self.x1 + a * np.cos(swing_theta)
#             swing_y = self.y1 + b * np.sin(swing_theta)

#             stance_theta = np.linspace(0, -np.pi, self.stance_steps)
#             stance_x = self.x1 + a * np.cos(stance_theta)
#             stance_y = self.y1 + b * np.sin(stance_theta)

#             x = np.concatenate([swing_x, stance_x])
#             y = np.concatenate([swing_y, stance_y])

#             return x, y

#         def ik(self, x, y, pX=0, pY=0):
#             d = np.hypot(x, y)
#             if d > (self.arm1 + self.arm2) or d < abs(self.arm1 - self.arm2):
#                 return None

#             cos_theta2 = (x**2 + y**2 - self.arm1**2 - self.arm2**2) / (2 * self.arm1 * self.arm2)
#             cos_theta2 = np.clip(cos_theta2, -1, 1)
#             sin_theta2 = np.sqrt(1 - cos_theta2**2)
#             sin_theta2 = -sin_theta2

#             theta2 = np.arctan2(sin_theta2, cos_theta2)
#             k1 = self.arm1 + self.arm2 * cos_theta2
#             k2 = self.arm2 * sin_theta2
#             theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

#             theta1 = int(np.degrees(theta1))
#             theta2 = int(180 + np.degrees(theta2))

#             return theta1, theta1 + theta2

#     class LegMovement:
#         def __init__(self, legs, fp, steps):
#             self.legs = legs
#             self.foot_positions = fp
#             self.steps = steps * 2
#             self.generate_angles()
#             self.walking = False

#         def generate_angles(self):
#             self.angles = []
#             for steps in range(self.steps):
#                 command = [None] * 4
#                 leg_trajectories = [leg.trajectory for leg in self.legs]
#                 leg_positions = []
                
#                 for i, (x_path, y_path) in enumerate(leg_trajectories):
#                     idx = (steps + self.legs[i].offset * self.legs[i].swing_steps) % self.legs[i].gait_cycle_len
#                     xP, yP = x_path[idx], y_path[idx]
#                     leg_positions.append((xP, yP))

#                 for i in range(4):
#                     x, y = leg_positions[i]
#                     a, b = self.legs[i].ik(x, y)
#                     command[i] = [f"{int(b)}", f"{int(a)}", "0"]

#                 command_flat = [angle for leg_angles in command for angle in leg_angles]
#                 command = f"<{','.join(command_flat)}>"
#                 self.angles.append(command)

#         def start_walking(self, delay=None):
#             """Start the walking sequence"""
#             if delay is None:
#                 delay = DELAY if ROBOT_AVAILABLE else 0.05
                
#             self.walking = True
#             # Initialize robot position
#             command = [None] * 4
#             for i in range(4):
#                 x, y = self.foot_positions[i]
#                 a, b = self.legs[i].ik(x, y)
#                 command[i] = [f"{int(b)}", f"{int(a)}", "0"]

#             command_flat = [angle for leg_angles in command for angle in leg_angles]
#             init_command = f"<{','.join(command_flat)}>"
            
#             if ser and ser.is_open:
#                 try:
#                     ser.write_line(init_command.encode())
#                     now = datetime.now()
#                     minute = now.strftime("%M")
#                     second = now.strftime("%S")
#                     millisecond = int(now.microsecond / 1000)
#                     print(f"Sent: {minute}:{second}:{millisecond}:- {init_command}")
#                 except Exception as e:
#                     print(f"Serial error during init: {e}")
#             else:
#                 print(f"Error: Serial port {SERIAL_PORT_SEND if ROBOT_AVAILABLE else 'N/A'} not available")
            
#             time.sleep(4)  # Initial delay like in your original code
            
#             steps = 0
#             while self.walking:
#                 try:
#                     calcStart = time.time()
#                     command = self.angles[steps % self.steps]
                    
#                     if ser and ser.is_open:
#                         try:
#                             command_encoded = command.encode()
#                             ser.write_line(command_encoded)
#                             now = datetime.now()
#                             minute = now.strftime("%M")
#                             second = now.strftime("%S")
#                             millisecond = int(now.microsecond / 1000)
#                             print(f"Sent: {minute}:{second}:{millisecond}:- {command_encoded}")
#                         except Exception as e:
#                             print(f"Serial error: {e}")
#                     else:
#                         print(f"Error: Serial port {SERIAL_PORT_SEND if ROBOT_AVAILABLE else 'N/A'} not available")
                    
#                     time.sleep(delay)  # Use the delay parameter
#                     print(f"Time taken : {time.time() - calcStart}")
#                     steps += 1
                    
#                 except Exception as e:
#                     print(f"Error in walking loop: {e}")
#                     break

#         def stop_walking(self):
#             """Stop the walking sequence"""
#             self.walking = False
#             print("🛑 Stopping robot walk")

#     # Initialize robot components
#     try:
#         wm = [
#             WalkingMechanism(HIP_KNEE_LENGTH, KNEE_FOOT_LENGTH, FOOT_POSITIONS_WALK[0], 
#                            STEP_LENGTH_X, STEP_LENGTH_Y, 0, INTERPOLATION_STEPS_X, INTERPOLATION_STEPS_Y),
#             WalkingMechanism(HIP_KNEE_LENGTH, KNEE_FOOT_LENGTH, FOOT_POSITIONS_WALK[1], 
#                            STEP_LENGTH_X, STEP_LENGTH_Y, 1, INTERPOLATION_STEPS_X, INTERPOLATION_STEPS_Y),
#             WalkingMechanism(HIP_KNEE_LENGTH, KNEE_FOOT_LENGTH, FOOT_POSITIONS_WALK[2], 
#                            STEP_LENGTH_X, STEP_LENGTH_Y, 1, INTERPOLATION_STEPS_X, INTERPOLATION_STEPS_Y),
#             WalkingMechanism(HIP_KNEE_LENGTH, KNEE_FOOT_LENGTH, FOOT_POSITIONS_WALK[3], 
#                            STEP_LENGTH_X, STEP_LENGTH_Y, 0, INTERPOLATION_STEPS_X, INTERPOLATION_STEPS_Y),
#         ]
        
#         robot_controller = LegMovement(wm, FOOT_POSITIONS_WALK, INTERPOLATION_STEPS_X)
#         print("Robot controller initialized")
#     except Exception as e:
#         robot_controller = None
#         print(f"Failed to initialize robot controller: {e}")

# # Robot control functions
# def robot_forward():
#     global robot_walking, walking_thread
#     print("🔥 EXECUTING: Moving robot forward", flush=True)
    
#     if not ROBOT_AVAILABLE or robot_controller is None:
#         print("Robot not available - running simulation")
    
#     if not robot_walking:
#         robot_walking = True
#         walking_thread = threading.Thread(target=robot_controller.start_walking, args=(DELAY,))
#         walking_thread.daemon = True
#         walking_thread.start()
#         return "Robot started walking forward"
#     else:
#         return "Robot already walking"

# def robot_backward():
#     print("EXECUTING: Moving robot backward", flush=True)
#     # For backward movement, you might need to modify the gait or reverse direction
#     # This would require changes to your WalkingMechanism class
#     return "Backward movement not implemented yet"

# def robot_left():
#     print(" EXECUTING: Turning robot left", flush=True)
#     # Left turn logic would go here
#     return "Left turn not implemented yet"

# def robot_right():
#     print(" EXECUTING: Turning robot right", flush=True)
#     # Right turn logic would go here
#     return "Right turn not implemented yet"

# def robot_stop():
#     global robot_walking, walking_thread
#     print(" EXECUTING: Stopping robot", flush=True)
    
#     if robot_walking and robot_controller:
#         robot_walking = False
#         robot_controller.stop_walking()
#         if walking_thread:
#             walking_thread.join(timeout=1)
#         return "Robot stopped"
#     else:
#         return "Robot was not moving"

# def robot_custom1():
#     print("🎯 EXECUTING: Custom action 1", flush=True)
#     # Add custom behavior here
#     return "Custom action 1 executed"

# def robot_custom2():
#     print("🔧 EXECUTING: Custom action 2", flush=True)
#     # Add custom behavior here
#     return "Custom action 2 executed"

# @app.route('/')
# def index():
#     print("Homepage accessed!", flush=True)
#     return render_template('index.html')

# @app.route('/move', methods=['POST'])
# def move():
#     timestamp = datetime.now().strftime("%H:%M:%S")
#     print("=" * 50, flush=True)
#     print(f"[{timestamp}] MOVE ENDPOINT HIT!", flush=True)
    
#     direction = request.form.get('direction', 'unknown')
#     print(f"📡 COMMAND RECEIVED: {direction.upper()}", flush=True)
    
#     # Execute robot actions based on direction
#     if direction == 'forward':
#         result = robot_forward()
#     elif direction == 'backward':
#         result = robot_backward()
#     elif direction == 'left':
#         result = robot_left()
#     elif direction == 'right':
#         result = robot_right()
#     elif direction == 'stop':
#         result = robot_stop()
#     elif direction == 'custom1':
#         result = robot_custom1()
#     elif direction == 'custom2':
#         result = robot_custom2()
#     else:
#         result = f"Unknown command: {direction}"
#         print(f"⚠️ UNKNOWN COMMAND: {direction}", flush=True)
    
#     print(f"✅ ACTION COMPLETED: {result}", flush=True)
#     print("=" * 50, flush=True)
#     sys.stdout.flush()
    
#     return f"✅ {result} at {timestamp}"

# if __name__ == '__main__':
#     local_ip = get_local_ip()
    
#     print("🚀 Starting Quadruped Robot Flask Controller...", flush=True)
#     print("📡 Server accessible on local network!", flush=True)
#     print("🌐 Access URLs:", flush=True)
#     print(f"   • Computer: http://localhost:5000", flush=True)
#     print(f"   • Phone/Tablet: http://{local_ip}:5000", flush=True)
#     print("👀 Watch this terminal for robot commands!", flush=True)
#     print("-" * 50, flush=True)
    
#     try:
#         app.run(debug=True, host='0.0.0.0', port=5000, threaded=True)
#     except KeyboardInterrupt:
#         print("\n🛑 Shutting down robot controller...")
#         if robot_walking and robot_controller:
#             robot_controller.stop_walking()
#         print("👋 Goodbye!")







#included pi streaming with control buttons



# from flask import Flask, render_template, request, Response
# import sys
# from datetime import datetime
# import socket
# import time
# import threading
# import copy
# import os
# import numpy as np
# from math import comb

# # Camera imports
# from picamera2 import Picamera2
# import cv2

# # Add your robot control imports here
# try:
#     config_dir = os.path.abspath(os.path.join(__file__, "../../../.."))
#     sys.path.append(config_dir)
#     libraries_dir = os.path.abspath(os.path.join(__file__, "../../../../libraries/python"))
#     sys.path.append(libraries_dir)
    
#     from kinematics import inverse_kinematics
#     from serial_comm import SerialComm
#     from config import (BAUD_RATE_REC, BAUD_RATE_SEND, DELAY, FOOT_POSITIONS_WALK,
#                        HIP_KNEE_LENGTH, INTERPOLATION_STEPS_X, INTERPOLATION_STEPS_Y,
#                        KNEE_FOOT_LENGTH, NUM_LEGS, SERIAL_PORT_REC, SERIAL_PORT_SEND,
#                        STEP_LENGTH_X, STEP_LENGTH_Y)
    
#     ROBOT_AVAILABLE = True
#     print("✅ Robot libraries loaded successfully!")
# except ImportError as e:
#     ROBOT_AVAILABLE = False
#     print(f"⚠️ Robot libraries not available: {e}")
#     print("⚠️ Running in simulation mode")

# app = Flask(__name__)

# # Global variables for robot control
# robot_walking = False
# walking_thread = None

# # Initialize camera (with process check to avoid conflicts)
# picam2 = None
# try:
#     picam2 = Picamera2()
#     picam2.start()
#     print("✅ Pi Camera initialized")
# except RuntimeError as e:
#     if "Device or resource busy" in str(e):
#         print("⚠️ Camera busy - kill other camera processes:")
#         print("sudo pkill libcamera")
#         print("sudo pkill -f picamera")
#     else:
#         print(f"⚠️ Camera error: {e}")
#     picam2 = None
# except Exception as e:
#     print(f"⚠️ Camera initialization failed: {e}")
#     picam2 = None

# def get_local_ip():
#     try:
#         with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
#             s.connect(("8.8.8.8", 80))
#             return s.getsockname()[0]
#     except:
#         return "Unable to determine IP"

# # Copy your robot classes here (only if libraries are available)
# if ROBOT_AVAILABLE:
#     try:
#         ser = SerialComm(SERIAL_PORT_SEND, BAUD_RATE_SEND, timeout=0.01)
#         time.sleep(2)
#         print("✅ Serial connection established")
#     except Exception as e:
#         ser = None
#         print(f"⚠️ Could not connect to motor serial port: {e}")

#     class WalkingMechanism:
#         def __init__(self, arm1, arm2, start, step_lengthX, step_lengthY, offset, inter_stepsX, inter_stepsY):
#             self.arm1 = arm1
#             self.arm2 = arm2
#             self.x1, self.y1 = start[0], start[1]
#             self.step_lengthX = step_lengthX
#             self.step_lengthY = step_lengthY
#             self.swing_steps = inter_stepsX
#             self.stance_steps = inter_stepsY
#             self.gait_cycle_len = self.swing_steps + self.stance_steps
#             self.offset = offset
#             self.trajectory = self.elliptical_path()

#         def elliptical_path(self):
#             a = self.step_lengthX / 2
#             b = self.step_lengthY

#             swing_theta = np.linspace(np.pi, 0, self.swing_steps)
#             swing_x = self.x1 + a * np.cos(swing_theta)
#             swing_y = self.y1 + b * np.sin(swing_theta)

#             stance_theta = np.linspace(0, -np.pi, self.stance_steps)
#             stance_x = self.x1 + a * np.cos(stance_theta)
#             stance_y = self.y1 + b * np.sin(stance_theta)

#             x = np.concatenate([swing_x, stance_x])
#             y = np.concatenate([swing_y, stance_y])

#             return x, y

#         def ik(self, x, y, pX=0, pY=0):
#             d = np.hypot(x, y)
#             if d > (self.arm1 + self.arm2) or d < abs(self.arm1 - self.arm2):
#                 return None

#             cos_theta2 = (x**2 + y**2 - self.arm1**2 - self.arm2**2) / (2 * self.arm1 * self.arm2)
#             cos_theta2 = np.clip(cos_theta2, -1, 1)
#             sin_theta2 = np.sqrt(1 - cos_theta2**2)
#             sin_theta2 = -sin_theta2

#             theta2 = np.arctan2(sin_theta2, cos_theta2)
#             k1 = self.arm1 + self.arm2 * cos_theta2
#             k2 = self.arm2 * sin_theta2
#             theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

#             theta1 = int(np.degrees(theta1))
#             theta2 = int(180 + np.degrees(theta2))

#             return theta1, theta1 + theta2

#     class LegMovement:
#         def __init__(self, legs, fp, steps):
#             self.legs = legs
#             self.foot_positions = fp
#             self.steps = steps * 2
#             self.generate_angles()
#             self.walking = False

#         def generate_angles(self):
#             self.angles = []
#             for steps in range(self.steps):
#                 command = [None] * 4
#                 leg_trajectories = [leg.trajectory for leg in self.legs]
#                 leg_positions = []
                
#                 for i, (x_path, y_path) in enumerate(leg_trajectories):
#                     idx = (steps + self.legs[i].offset * self.legs[i].swing_steps) % self.legs[i].gait_cycle_len
#                     xP, yP = x_path[idx], y_path[idx]
#                     leg_positions.append((xP, yP))

#                 for i in range(4):
#                     x, y = leg_positions[i]
#                     a, b = self.legs[i].ik(x, y)
#                     command[i] = [f"{int(b)}", f"{int(a)}", "0"]

#                 command_flat = [angle for leg_angles in command for angle in leg_angles]
#                 command = f"<{','.join(command_flat)}>"
#                 self.angles.append(command)

#         def start_walking(self, delay=None):
#             if delay is None:
#                 delay = DELAY if ROBOT_AVAILABLE else 0.05
                
#             self.walking = True
#             command = [None] * 4
#             for i in range(4):
#                 x, y = self.foot_positions[i]
#                 a, b = self.legs[i].ik(x, y)
#                 command[i] = [f"{int(b)}", f"{int(a)}", "0"]

#             command_flat = [angle for leg_angles in command for angle in leg_angles]
#             init_command = f"<{','.join(command_flat)}>"
            
#             if ser and ser.is_open:
#                 try:
#                     ser.write_line(init_command.encode())
#                     now = datetime.now()
#                     minute = now.strftime("%M")
#                     second = now.strftime("%S")
#                     millisecond = int(now.microsecond / 1000)
#                     print(f"Sent: {minute}:{second}:{millisecond}:- {init_command}")
#                 except Exception as e:
#                     print(f"Serial error during init: {e}")
#             else:
#                 print(f"Error: Serial port {SERIAL_PORT_SEND if ROBOT_AVAILABLE else 'N/A'} not available")
            
#             time.sleep(4)
            
#             steps = 0
#             while self.walking:
#                 try:
#                     calcStart = time.time()
#                     command = self.angles[steps % self.steps]
                    
#                     if ser and ser.is_open:
#                         try:
#                             command_encoded = command.encode()
#                             ser.write_line(command_encoded)
#                             now = datetime.now()
#                             minute = now.strftime("%M")
#                             second = now.strftime("%S")
#                             millisecond = int(now.microsecond / 1000)
#                             print(f"Sent: {minute}:{second}:{millisecond}:- {command_encoded}")
#                         except Exception as e:
#                             print(f"Serial error: {e}")
#                     else:
#                         print(f"Error: Serial port {SERIAL_PORT_SEND if ROBOT_AVAILABLE else 'N/A'} not available")
                    
#                     time.sleep(delay)
#                     print(f"Time taken : {time.time() - calcStart}")
#                     steps += 1
                    
#                 except Exception as e:
#                     print(f"Error in walking loop: {e}")
#                     break

#         def stop_walking(self):
#             self.walking = False
#             print("🛑 Stopping robot walk")

#     # Initialize robot components
#     try:
#         wm = [
#             WalkingMechanism(HIP_KNEE_LENGTH, KNEE_FOOT_LENGTH, FOOT_POSITIONS_WALK[0], 
#                            STEP_LENGTH_X, STEP_LENGTH_Y, 0, INTERPOLATION_STEPS_X, INTERPOLATION_STEPS_Y),
#             WalkingMechanism(HIP_KNEE_LENGTH, KNEE_FOOT_LENGTH, FOOT_POSITIONS_WALK[1], 
#                            STEP_LENGTH_X, STEP_LENGTH_Y, 1, INTERPOLATION_STEPS_X, INTERPOLATION_STEPS_Y),
#             WalkingMechanism(HIP_KNEE_LENGTH, KNEE_FOOT_LENGTH, FOOT_POSITIONS_WALK[2], 
#                            STEP_LENGTH_X, STEP_LENGTH_Y, 1, INTERPOLATION_STEPS_X, INTERPOLATION_STEPS_Y),
#             WalkingMechanism(HIP_KNEE_LENGTH, KNEE_FOOT_LENGTH, FOOT_POSITIONS_WALK[3], 
#                            STEP_LENGTH_X, STEP_LENGTH_Y, 0, INTERPOLATION_STEPS_X, INTERPOLATION_STEPS_Y),
#         ]
        
#         robot_controller = LegMovement(wm, FOOT_POSITIONS_WALK, INTERPOLATION_STEPS_X)
#         print("✅ Robot controller initialized")
#     except Exception as e:
#         robot_controller = None
#         print(f"⚠️ Failed to initialize robot controller: {e}")

# # Camera streaming function
# def generate_frames():
#     if picam2 is None:
#         # Camera not available - return placeholder
#         while True:
#             blank_frame = np.zeros((480, 640, 3), dtype=np.uint8)
#             cv2.putText(blank_frame, "Camera Not Available", (160, 240), 
#                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
#             _, buffer = cv2.imencode('.jpg', blank_frame)
#             yield (b'--frame\r\n'
#                    b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
#             time.sleep(0.5)
#     else:
#         while True:
#             try:
#                 frame = picam2.capture_array()
#                 frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
#                 _, buffer = cv2.imencode('.jpg', frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 85])  # Set quality 85
#                 yield (b'--frame\r\n'
#                        b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
#             except Exception as e:
#                 print(f"Streaming error: {e}")
#                 break

# # Robot control functions
# def robot_forward():
#     global robot_walking, walking_thread
#     print("🔥 EXECUTING: Moving robot forward", flush=True)
    
#     if not ROBOT_AVAILABLE or robot_controller is None:
#         print("⚠️ Robot not available - running simulation")
#         return "Robot moving forward (simulation)"
    
#     if not robot_walking:
#         robot_walking = True
#         walking_thread = threading.Thread(target=robot_controller.start_walking, args=(DELAY,))
#         walking_thread.daemon = True
#         walking_thread.start()
#         return "Robot started walking forward"
#     else:
#         return "Robot already walking"

# def robot_backward():
#     print("⬅️ EXECUTING: Moving robot backward", flush=True)
#     return "Backward movement not implemented yet"

# def robot_left():
#     print("↩️ EXECUTING: Turning robot left", flush=True)
#     return "Left turn not implemented yet"

# def robot_right():
#     print("↪️ EXECUTING: Turning robot right", flush=True)
#     return "Right turn not implemented yet"

# def robot_stop():
#     global robot_walking, walking_thread
#     print("🛑 EXECUTING: Stopping robot", flush=True)
    
#     if robot_walking and robot_controller:
#         robot_walking = False
#         robot_controller.stop_walking()
#         if walking_thread:
#             walking_thread.join(timeout=1)
#         return "Robot stopped"
#     else:
#         return "Robot was not moving"

# def robot_custom1():
#     print("🎯 EXECUTING: Custom action 1", flush=True)
#     return "Custom action 1 executed"

# def robot_custom2():
#     print("🔧 EXECUTING: Custom action 2", flush=True)
#     return "Custom action 2 executed"

# # Routes
# @app.route('/')
# def index():
#     print("Homepage accessed!", flush=True)
#     return render_template('index.html')

# @app.route('/video_feed')
# def video_feed():
#     return Response(generate_frames(),
#                     mimetype='multipart/x-mixed-replace; boundary=frame')

# @app.route('/move', methods=['POST'])
# def move():
#     timestamp = datetime.now().strftime("%H:%M:%S")
#     print("=" * 50, flush=True)
#     print(f"[{timestamp}] MOVE ENDPOINT HIT!", flush=True)
    
#     direction = request.form.get('direction', 'unknown')
#     print(f"📡 COMMAND RECEIVED: {direction.upper()}", flush=True)
    
#     # Execute robot actions based on direction
#     if direction == 'forward':
#         result = robot_forward()
#     elif direction == 'backward':
#         result = robot_backward()
#     elif direction == 'left':
#         result = robot_left()
#     elif direction == 'right':
#         result = robot_right()
#     elif direction == 'stop':
#         result = robot_stop()
#     elif direction == 'custom1':
#         result = robot_custom1()
#     elif direction == 'custom2':
#         result = robot_custom2()
#     else:
#         result = f"Unknown command: {direction}"
#         print(f"⚠️ UNKNOWN COMMAND: {direction}", flush=True)
    
#     print(f"✅ ACTION COMPLETED: {result}", flush=True)
#     print("=" * 50, flush=True)
#     sys.stdout.flush()
    
#     return f"✅ {result} at {timestamp}"

# if __name__ == '__main__':
#     local_ip = get_local_ip()
    
#     print("🚀 Starting Quadruped Robot Flask Controller with Camera Stream...", flush=True)
#     print("📡 Server accessible on local network!", flush=True)
#     print("🌐 Access URLs:", flush=True)
#     print(f"   • Computer: http://localhost:5000", flush=True)
#     print(f"   • Phone/Tablet: http://{local_ip}:5000", flush=True)
#     print(f"   • Camera Stream: http://{local_ip}:5000/video_feed", flush=True)
#     print("👀 Watch this terminal for robot commands!", flush=True)
#     print("-" * 50, flush=True)
    
#     try:
#         app.run(debug=False, host='0.0.0.0', port=5000, threaded=True)  # Turn off debug mode to avoid camera conflicts
#     except KeyboardInterrupt:
#         print("\n🛑 Shutting down robot controller...")
#         if robot_walking and robot_controller:
#             robot_controller.stop_walking()
#         if picam2:
#             picam2.stop()
#         print("👋 Goodbye!")







from flask import Flask, render_template, request, Response
import sys
from datetime import datetime
import socket
import time
import threading
import copy
import os
import numpy as np
from math import comb
import subprocess  # For display control

# Camera imports
from picamera2 import Picamera2
import cv2

# Add your robot control imports here
try:
    config_dir = os.path.abspath(os.path.join(__file__, "../../../.."))
    sys.path.append(config_dir)
    libraries_dir = os.path.abspath(os.path.join(__file__, "../../../../libraries/python"))
    sys.path.append(libraries_dir)
    
    from kinematics import inverse_kinematics
    from serial_comm import SerialComm
    from config import (BAUD_RATE_REC, BAUD_RATE_SEND, DELAY, FOOT_POSITIONS_WALK,
                       HIP_KNEE_LENGTH, INTERPOLATION_STEPS_X, INTERPOLATION_STEPS_Y,
                       KNEE_FOOT_LENGTH, NUM_LEGS, SERIAL_PORT_REC, SERIAL_PORT_SEND,
                       STEP_LENGTH_X, STEP_LENGTH_Y)
    
    ROBOT_AVAILABLE = True
    print("✅ Robot libraries loaded successfully!")
except ImportError as e:
    ROBOT_AVAILABLE = False
    print(f"⚠️ Robot libraries not available: {e}")
    print("⚠️ Running in simulation mode")

app = Flask(__name__)

# Global variables for robot control
robot_walking = False
walking_thread = None
display_active = False  # Display starts dark

# def set_display_black():
#     """Show black image fullscreen on Pi display"""
#     try:
#         # Create a black image and display it fullscreen using fbi
#         subprocess.run(['sudo', 'fbi', '-T', '1', '-noverbose', '--autozoom', '/dev/null'], 
#                       input=b'', check=False, stderr=subprocess.DEVNULL)
#         print("🖥️ Black screen displayed")
#     except:
#         try:
#             # Alternative: use a black wallpaper if available
#             subprocess.run(['pcmanfm', '--set-wallpaper=/dev/null'], check=False)
#         except:
#             print("⚠️ Could not set black display")

# def show_display_image():
#     """Show an image fullscreen on the Pi display"""
#     global display_active
#     if not display_active:
#         try:
#             # Kill any existing fbi processes
#             subprocess.run(['sudo', 'pkill', 'fbi'], check=False)
            
#             # Get image path relative to this app.py file
#             app_dir = os.path.dirname(os.path.abspath(__file__))
#             image_path = os.path.join(app_dir, "robot_image.png")  # Same folder as app.py
            
#             if os.path.exists(image_path):
#                 subprocess.run(['sudo', 'fbi', '-T', '1', '-noverbose', '--autozoom', image_path], 
#                               check=True, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
#                 print(f"🖥️ Image displayed: {image_path}")
#             else:
#                 # If no image file, just clear the black screen
#                 subprocess.run(['sudo', 'pkill', 'fbi'], check=False)
#                 print(f"⚠️ No image file found at: {image_path}")
#                 print("⚠️ Put robot_image.jpg in same folder as app.py")
#         except Exception as e:
#             print(f"⚠️ Could not display image: {e}")
        
#         display_active = True

def get_local_ip():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except:
        return "Unable to determine IP"

# Initialize camera (with process check to avoid conflicts)
picam2 = None
try:
    picam2 = Picamera2()
    picam2.start()
    print("✅ Pi Camera initialized")
except RuntimeError as e:
    if "Device or resource busy" in str(e):
        print("⚠️ Camera busy - kill other camera processes:")
        print("sudo pkill libcamera")
        print("sudo pkill -f picamera")
    else:
        print(f"⚠️ Camera error: {e}")
    picam2 = None
except Exception as e:
    print(f"⚠️ Camera initialization failed: {e}")
    picam2 = None

# Copy your robot classes here (only if libraries are available)
if ROBOT_AVAILABLE:
    try:
        ser = SerialComm(SERIAL_PORT_SEND, BAUD_RATE_SEND, timeout=0.01)
        time.sleep(2)
        print("✅ Serial connection established")
    except Exception as e:
        ser = None
        print(f"⚠️ Could not connect to motor serial port: {e}")

    class WalkingMechanism:
        def __init__(self, arm1, arm2, start, step_lengthX, step_lengthY, offset, inter_stepsX, inter_stepsY):
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

            cos_theta2 = (x**2 + y**2 - self.arm1**2 - self.arm2**2) / (2 * self.arm1 * self.arm2)
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
            self.steps = steps * 2
            self.generate_angles()
            self.walking = False

        def generate_angles(self):
            self.angles = []
            for steps in range(self.steps):
                command = [None] * 4
                leg_trajectories = [leg.trajectory for leg in self.legs]
                leg_positions = []
                
                for i, (x_path, y_path) in enumerate(leg_trajectories):
                    idx = (steps + self.legs[i].offset * self.legs[i].swing_steps) % self.legs[i].gait_cycle_len
                    xP, yP = x_path[idx], y_path[idx]
                    leg_positions.append((xP, yP))

                for i in range(4):
                    x, y = leg_positions[i]
                    a, b = self.legs[i].ik(x, y)
                    command[i] = [f"{int(b)}", f"{int(a)}", "0"]

                command_flat = [angle for leg_angles in command for angle in leg_angles]
                command = f"<{','.join(command_flat)}>"
                self.angles.append(command)

        def start_walking(self, delay=None):
            if delay is None:
                delay = DELAY if ROBOT_AVAILABLE else 0.05
                
            self.walking = True
            command = [None] * 4
            for i in range(4):
                x, y = self.foot_positions[i]
                a, b = self.legs[i].ik(x, y)
                command[i] = [f"{int(b)}", f"{int(a)}", "0"]

            command_flat = [angle for leg_angles in command for angle in leg_angles]
            init_command = f"<{','.join(command_flat)}>"
            
            if ser and ser.is_open:
                try:
                    ser.write_line(init_command.encode())
                    now = datetime.now()
                    minute = now.strftime("%M")
                    second = now.strftime("%S")
                    millisecond = int(now.microsecond / 1000)
                    print(f"Sent: {minute}:{second}:{millisecond}:- {init_command}")
                except Exception as e:
                    print(f"Serial error during init: {e}")
            else:
                print(f"Error: Serial port {SERIAL_PORT_SEND if ROBOT_AVAILABLE else 'N/A'} not available")
            
            time.sleep(4)
            
            steps = 0
            while self.walking:
                try:
                    calcStart = time.time()
                    command = self.angles[steps % self.steps]
                    
                    if ser and ser.is_open:
                        try:
                            command_encoded = command.encode()
                            ser.write_line(command_encoded)
                            now = datetime.now()
                            minute = now.strftime("%M")
                            second = now.strftime("%S")
                            millisecond = int(now.microsecond / 1000)
                            print(f"Sent: {minute}:{second}:{millisecond}:- {command_encoded}")
                        except Exception as e:
                            print(f"Serial error: {e}")
                    else:
                        print(f"Error: Serial port {SERIAL_PORT_SEND if ROBOT_AVAILABLE else 'N/A'} not available")
                    
                    time.sleep(delay)
                    print(f"Time taken : {time.time() - calcStart}")
                    steps += 1
                    
                except Exception as e:
                    print(f"Error in walking loop: {e}")
                    break

        def stop_walking(self):
            self.walking = False
            print("🛑 Stopping robot walk")

    # Initialize robot components
    try:
        wm = [
            WalkingMechanism(HIP_KNEE_LENGTH, KNEE_FOOT_LENGTH, FOOT_POSITIONS_WALK[0], 
                           STEP_LENGTH_X, STEP_LENGTH_Y, 0, INTERPOLATION_STEPS_X, INTERPOLATION_STEPS_Y),
            WalkingMechanism(HIP_KNEE_LENGTH, KNEE_FOOT_LENGTH, FOOT_POSITIONS_WALK[1], 
                           STEP_LENGTH_X, STEP_LENGTH_Y, 1, INTERPOLATION_STEPS_X, INTERPOLATION_STEPS_Y),
            WalkingMechanism(HIP_KNEE_LENGTH, KNEE_FOOT_LENGTH, FOOT_POSITIONS_WALK[2], 
                           STEP_LENGTH_X, STEP_LENGTH_Y, 1, INTERPOLATION_STEPS_X, INTERPOLATION_STEPS_Y),
            WalkingMechanism(HIP_KNEE_LENGTH, KNEE_FOOT_LENGTH, FOOT_POSITIONS_WALK[3], 
                           STEP_LENGTH_X, STEP_LENGTH_Y, 0, INTERPOLATION_STEPS_X, INTERPOLATION_STEPS_Y),
        ]
        
        robot_controller = LegMovement(wm, FOOT_POSITIONS_WALK, INTERPOLATION_STEPS_X)
        print("✅ Robot controller initialized")
    except Exception as e:
        robot_controller = None
        print(f"⚠️ Failed to initialize robot controller: {e}")

# Camera streaming function
def generate_frames():
    if picam2 is None:
        # Camera not available - return placeholder
        while True:
            blank_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(blank_frame, "Camera Not Available", (160, 240), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            _, buffer = cv2.imencode('.jpg', blank_frame)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            time.sleep(0.5)
    else:
        while True:
            try:
                frame = picam2.capture_array()
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                frame_bgr=cv2.rotate(frame_bgr,cv2.ROTATE_180)
                
                _, buffer = cv2.imencode('.jpg', frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 85])  # Set quality 85
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            except Exception as e:
                print(f"Streaming error: {e}")
                break

# Robot control functions
def robot_forward():
    global robot_walking, walking_thread
    print("🔥 EXECUTING: Moving robot forward", flush=True)
    
    # Turn on display when any control is pressed
    # show_display_image()
    
    if not ROBOT_AVAILABLE or robot_controller is None:
        print("⚠️ Robot not available - running simulation")
        return "Robot moving forward (simulation)"
    
    if not robot_walking:
        robot_walking = True
        walking_thread = threading.Thread(target=robot_controller.start_walking, args=(DELAY,))
        walking_thread.daemon = True
        walking_thread.start()
        return "Robot started walking forward"
    else:
        return "Robot already walking"

def robot_backward():
    print("⬅️ EXECUTING: Moving robot backward", flush=True)
    return "Backward movement not implemented yet"

def robot_left():
    print("↩️ EXECUTING: Turning robot left", flush=True)
    return "Left turn not implemented yet"

def robot_right():
    print("↪️ EXECUTING: Turning robot right", flush=True)
    return "Right turn not implemented yet"

def robot_stop():
    global robot_walking, walking_thread
    print("🛑 EXECUTING: Stopping robot", flush=True)
    
    
    if robot_walking and robot_controller:
        robot_walking = False
        robot_controller.stop_walking()
        if walking_thread:
            walking_thread.join(timeout=1)
        return "Robot stopped"
    else:
        return "Robot was not moving"

def robot_custom1():
    print("🎯 EXECUTING: Custom action 1", flush=True)
    return "Custom action 1 executed"

def robot_custom2():
    print("🔧 EXECUTING: Custom action 2", flush=True)
    return "Custom action 2 executed"

# Routes
@app.route('/')
def index():
    print("Homepage accessed!", flush=True)
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/move', methods=['POST'])
def move():
    timestamp = datetime.now().strftime("%H:%M:%S")
    print("=" * 50, flush=True)
    print(f"[{timestamp}] MOVE ENDPOINT HIT!", flush=True)
    
    direction = request.form.get('direction', 'unknown')
    print(f"📡 COMMAND RECEIVED: {direction.upper()}", flush=True)
    
    # Execute robot actions based on direction
    if direction == 'forward':
        result = robot_forward()
    elif direction == 'backward':
        result = robot_backward()
    elif direction == 'left':
        result = robot_left()
    elif direction == 'right':
        result = robot_right()
    elif direction == 'stop':
        result = robot_stop()
    elif direction == 'custom1':
        result = robot_custom1()
    elif direction == 'custom2':
        result = robot_custom2()
    else:
        result = f"Unknown command: {direction}"
        print(f"⚠️ UNKNOWN COMMAND: {direction}", flush=True)
    
    print(f"✅ ACTION COMPLETED: {result}", flush=True)
    print("=" * 50, flush=True)
    sys.stdout.flush()
    
    return f"✅ {result} at {timestamp}"

if __name__ == '__main__':
    local_ip = get_local_ip()
    
    # Show black screen at startup
    
    print("🚀 Starting Quadruped Robot Flask Controller with Camera Stream...", flush=True)
    print("📡 Server accessible on local network!", flush=True)
    print("🌐 Access URLs:", flush=True)
    print(f"   • Computer: http://localhost:5000", flush=True)
    print(f"   • Phone/Tablet: http://{local_ip}:5000", flush=True)
    print(f"   • Camera Stream: http://{local_ip}:5000/video_feed", flush=True)
    print("👀 Watch this terminal for robot commands!", flush=True)
    print("-" * 50, flush=True)
    
    try:
        app.run(debug=False, host='0.0.0.0', port=5000, threaded=True)  # Turn off debug mode to avoid camera conflicts
    except KeyboardInterrupt:
        print("\n🛑 Shutting down robot controller...")
        if robot_walking and robot_controller:
            robot_controller.stop_walking()
        if picam2:
            picam2.stop()
        # Kill any fbi processes
        subprocess.run(['sudo', 'pkill', 'fbi'], check=False)
        print("👋 Goodbye!")