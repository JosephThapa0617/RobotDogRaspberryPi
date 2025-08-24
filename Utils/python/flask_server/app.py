from flask import Flask, request, jsonify
import sys
import os
import time
import threading

# Import your Walk class
from walk import Walk

# Add robot libraries to path
try:
    config_dir = os.path.abspath(os.path.join(__file__, "../../../.."))
    sys.path.append(config_dir)
    libraries_dir = os.path.abspath(os.path.join(__file__, "../libraries/python"))
    sys.path.append(libraries_dir)
    
    from serial_comm import SerialComm
    from config import (BAUD_RATE_SEND, SERIAL_PORT_SEND, STEP_LENGTH_X, STEP_LENGTH_Y)
    
    ROBOT_AVAILABLE = True
    print("✅ Robot libraries loaded!")
except ImportError as e:
    ROBOT_AVAILABLE = False
    print(f"⚠️ Robot libraries not available: {e}")

app = Flask(__name__)

# Global variables
robot_walking = False
walking_thread = None
walk_instance = None

# Initialize robot
if ROBOT_AVAILABLE:
    try:
        ser = SerialComm(SERIAL_PORT_SEND, BAUD_RATE_SEND, timeout=0.01)
        time.sleep(2)
        walk_instance = Walk(ser)
        print("✅ Robot initialized!")
    except Exception as e:
        walk_instance = None
        print(f"⚠️ Robot initialization failed: {e}")

def stop_current_walk():
    """Stop any current walking"""
    global robot_walking
    robot_walking = False

def robot_forward():
    global robot_walking, walking_thread
    print("🔥 Moving robot forward")
    
    if not ROBOT_AVAILABLE or walk_instance is None:
        return "Robot not available"
    
    stop_current_walk()
    robot_walking = True
    
    def walk_forward():
        try:
            walk_instance.reset()
            time.sleep(1)
            walk_instance.set_step_lengthX(-120, -120)
            walk_instance.walk(duration=5)  # Walk for 5 seconds
        except Exception as e:
            print(f"Forward walk error: {e}")
        finally:
            global robot_walking
            robot_walking = False
    
    walking_thread = threading.Thread(target=walk_forward)
    walking_thread.daemon = True
    walking_thread.start()
    return "Moving forward"

def robot_backward():
    global robot_walking, walking_thread
    print("⬅️ Moving robot backward")
    
    if not ROBOT_AVAILABLE or walk_instance is None:
        return "Robot not available"
    
    stop_current_walk()
    robot_walking = True
    
    def walk_backward():
        try:
            walk_instance.reset()
            time.sleep(1)
            walk_instance.set_step_lengthX(120,120)
            walk_instance.set_step_lengthY(0.0, 0.0)
            walk_instance.walk(duration=5)
        except Exception as e:
            print(f"Backward walk error: {e}")
        finally:
            global robot_walking
            robot_walking = False
    
    walking_thread = threading.Thread(target=walk_backward)
    walking_thread.daemon = True
    walking_thread.start()
    return "Moving backward"

def robot_left():
    global robot_walking, walking_thread
    print("↩️ Turning robot left")
    
    if not ROBOT_AVAILABLE or walk_instance is None:
        return "Robot not available"
    
    stop_current_walk()
    robot_walking = True
    
    def walk_left():
        try:
            walk_instance.reset()
            time.sleep(1)
            walk_instance.set_step_lengthX(80, -80)  # Left slower
            # walk_instance.set_step_lengthY(-80, 80)
            walk_instance.walk(duration=3)
        except Exception as e:
            print(f"Left turn error: {e}")
        finally:
            global robot_walking
            robot_walking = False
    
    walking_thread = threading.Thread(target=walk_left)
    walking_thread.daemon = True
    walking_thread.start()
    return "Turning left"

def robot_right():
    global robot_walking, walking_thread
    print("↪️ Turning robot right")
    
    if not ROBOT_AVAILABLE or walk_instance is None:
        return "Robot not available"
    
    stop_current_walk()
    robot_walking = True
    
    def walk_right():
        try:
            walk_instance.reset()
            time.sleep(1)
            walk_instance.set_step_lengthX(-80,80)  # Left slower
            # walk_instance.set_step_lengthY(80,-80)
            walk_instance.walk(duration=3)
            walk_instance.walk(duration=3)
        except Exception as e:
            print(f"Right turn error: {e}")
        finally:
            global robot_walking
            robot_walking = False
    
    walking_thread = threading.Thread(target=walk_right)
    walking_thread.daemon = True
    walking_thread.start()
    return "Turning right"

def robot_stop():
    global robot_walking
    print("🛑 Stopping robot")
    robot_walking=False
    if walk_instance:
        walk_instance.stop()
    return "Stopped"

# Simple web interface
@app.route('/')
def index():
    return '''
    <!DOCTYPE html>
    <html>
    <head>
        <title>Simple Robot Controller</title>
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <style>
            body { font-family: Arial; text-align: center; padding: 20px; }
            .controls { max-width: 300px; margin: 0 auto; }
            button { 
                width: 80px; height: 80px; margin: 10px; 
                font-size: 16px; border-radius: 10px;
                background: #4CAF50; color: white; border: none;
            }
            button:active { background: #45a049; }
            #stop { background: #f44336; }
            .row { display: flex; justify-content: center; }
        </style>
    </head>
    <body>
        <h1>🤖 Simple Robot Controller</h1>
        <div class="controls">
            <div class="row">
                <button onclick="sendCommand('forward')">↑<br>Forward</button>
            </div>
            <div class="row">
                <button onclick="sendCommand('left')">←<br>Left</button>
                <button id="stop" onclick="sendCommand('stop')">⏹<br>Stop</button>
                <button onclick="sendCommand('right')">→<br>Right</button>
            </div>
            <div class="row">
                <button onclick="sendCommand('backward')">↓<br>Backward</button>
            </div>
        </div>
        <div id="status" style="margin-top: 20px; color: #666;"></div>

        <script>
            function sendCommand(direction) {
                document.getElementById('status').innerHTML = 'Sending: ' + direction;
                
                fetch('/move', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                    body: 'direction=' + direction
                })
                .then(response => response.text())
                .then(data => {
                    document.getElementById('status').innerHTML = data;
                })
                .catch(error => {
                    document.getElementById('status').innerHTML = 'Error: ' + error;
                });
            }
        </script>
    </body>
    </html>
    '''

@app.route('/move', methods=['POST'])
def move():
    direction = request.form.get('direction', 'unknown')
    print(f"📡 Command: {direction.upper()}")
    
    # Execute robot actions
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
    else:
        result = f"Unknown command: {direction}"
    
    print(f"✅ Result: {result}")
    return result

if __name__ == '__main__':
    print("🚀 Starting Simple Robot Controller...")
    print("🌐 Access at: http://localhost:5000")
    
    try:
        app.run(debug=False, host='0.0.0.0', port=5000)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
        stop_current_walk()
        print("👋 Goodbye!")