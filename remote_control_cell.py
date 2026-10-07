from flask import Flask, render_template_string, request
import asyncio
import threading
import time
from mks_servo_can import CANInterface, Axis, RotaryKinematics, const
import RPi.GPIO as GPIO # Added GPIO import

app = Flask(__name__)

# Variables globales
can_if = None
motor1 = None
motor2 = None
motor3 = None
motor4 = None # Added motor4
motor5 = None # Added motor5
motor6 = None # Added motor6
motor_loop = None

#sudo ip link set down can0
#sudo ip link set can0 type can bitrate 500000 restart-ms 100
#sudo ip link set up can0

# New variables for GPIO Servo tracking
SERVO_PIN = 2
servo_pwm = None
current_servo_angle = 0  # Start at a neutral 90-degree position

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Pi Motor Control</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { 
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; 
            background: #f2f2f7; 
            color: #1c1c1e;
            padding: 20px 10px;
            display: flex;
            flex-direction: column;
            align-items: center;
        }
        h1 { font-size: 26px; font-weight: 700; margin-bottom: 20px; text-align: center; }
        
        /* Main Container for Mobile Constraints */
        .container {
            width: 100%;
            max-width: 400px; /* Perfectly bounds iPhone screens */
            display: flex;
            flex-direction: column;
            gap: 16px;
        }

        /* Motor Control Card Layout */
        .motor-card {
            background: #ffffff;
            border-radius: 14px;
            padding: 14px;
            box-shadow: 0 2px 8px rgba(0,0,0,0.05);
        }
        .motor-title {
            font-size: 15px;
            font-weight: 600;
            color: #8e8e93;
            margin-bottom: 10px;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }

        /* Button Grid Layout (2 actions per row side-by-side) */
        .btn-group {
            display: flex;
            gap: 10px;
        }
        .btn {
            flex: 1;
            padding: 16px 8px;
            font-size: 16px;
            font-weight: 600;
            color: #ffffff;
            background: #007aff; /* Apple Blue */
            border: none;
            border-radius: 10px;
            cursor: pointer;
            box-shadow: 0 2px 4px rgba(0,122,255,0.15);
            transition: opacity 0.1s ease;
        }
        .btn:active { opacity: 0.7; }
        
        /* Master Stop Button Configuration */
        .btn-stop {
            background: #ff3b30; /* Apple Red */
            padding: 20px;
            font-size: 20px;
            margin-top: 10px;
            box-shadow: 0 4px 10px rgba(255,59,48,0.25);
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Remote Control</h1>
        
        <!-- Motor 1 -->
        <div class="motor-card">
            <div class="motor-title">Motor 1 (Horizontal)</div>
            <div class="btn-group">
                <button class="btn" onclick="fetch('/forward')">FORWARD ⬆️</button>
                <button class="btn" onclick="fetch('/backward')">BACKWARD ⬇️</button>
            </div>
        </div>
        
        <!-- Motor 2 -->
        <div class="motor-card">
            <div class="motor-title">Motor 2 (Vertical)</div>
            <div class="btn-group">
                <button class="btn" onclick="fetch('/upward')">UPWARD ⬆️</button>
                <button class="btn" onclick="fetch('/downward')">DOWNWARD ⬇️</button>
            </div>
        </div>

        <!-- Motor 3 -->
        <div class="motor-card">
            <div class="motor-title">Motor 3 (Auxiliary)</div>
            <div class="btn-group">
                <button class="btn" onclick="fetch('/upward2')">UPWARD 2 ⬆️</button>
                <button class="btn" onclick="fetch('/downward2')">DOWNWARD 2 ⬇️</button>
            </div>
        </div>
        
        <!-- Motor 4 -->
        <div class="motor-card">
            <div class="motor-title">Motor 4 (Rotation 1)</div>
            <div class="btn-group">
                <button class="btn" onclick="fetch('/rot_clockwise')">CW 🔄</button>
                <button class="btn" onclick="fetch('/rot_counter_clockwise')">CCW 🔄</button>
            </div>
        </div>

        <!-- Motor 5 -->
        <div class="motor-card">
            <div class="motor-title">Motor 5 (Rotation 2)</div>
            <div class="btn-group">
                <button class="btn" onclick="fetch('/rot_clockwise2')">CW 2 🔄</button>
                <button class="btn" onclick="fetch('/rot_counter_clockwise2')">CCW 2 🔄</button>
            </div>
        </div>
        
        <!-- Motor 6 -->
        <div class="motor-card">
            <div class="motor-title">Motor 6 (Rotation 3)</div>
            <div class="btn-group">
                <button class="btn" onclick="fetch('/rot_clockwise3')">CW 3 🔄</button>
                <button class="btn" onclick="fetch('/rot_counter_clockwise3')">CCW 3 🔄</button>
            </div>
        </div>

        <!-- Motor 7 -->
        <div class="motor-card">
            <div class="motor-title">Motor 7 (GPIO Servo)</div>
            <div class="btn-group">
                <button class="btn" onclick="fetch('/open')">OPEN 🟢</button>
                <button class="btn" onclick="fetch('/close')">CLOSE 🔴</button>
            </div>
        </div>

        <!-- Emergency Stop -->
        <button class="btn btn-stop" onclick="fetch('/stop')">EMERGENCY STOP 🛑</button>
    </div>
</body>
</html>
"""

def angle_to_duty_cycle(angle):
    """Helper function to convert 0-180 degrees into a 50Hz PWM duty cycle"""
    # 2.5% duty cycle is roughly 0 degrees, 12.5% is roughly 180 degrees
    return 2.5 + (angle / 180.0) * 10.0

def init_gpio_servo():
    """Initializes the GPIO 3 pin for Servo PWM handling"""
    global servo_pwm
    GPIO.setmode(GPIO.BCM)  # Use BCM GPIO numbering layout
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    
    # Standard analog hobby servos run at 50Hz frequency
    servo_pwm = GPIO.PWM(SERVO_PIN, 50)
    servo_pwm.start(angle_to_duty_cycle(current_servo_angle))
    time.sleep(0.3)
    servo_pwm.ChangeDutyCycle(0) # Stop sending signal to prevent motor humming/shaking


async def init_motor_async():
    """Initialise le matériel CAN dans la boucle asynchrone"""
    global can_if, motor1, motor2, motor3, motor4, motor5, motor6
    print("Initializing Linux SocketCAN Interface...")
    can_if = CANInterface(
        use_simulator=False,
        interface_type="socketcan",
        channel="can0",
        bitrate=500000
    )
    try:
        print("Connecting to local socketcan channel 'can0'...")
        await can_if.connect()
        print("✅ Linux can0 linked successfully!")
    except Exception as hardware_error:
        print(f"❌ CONNECTION FAILED: {hardware_error}")
        return False

    kin = RotaryKinematics(steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION)
    motor1 = Axis(can_if, motor_can_id=1, name="MKS_Motor_1", kinematics=kin)
    motor2 = Axis(can_if, motor_can_id=2, name="MKS_Motor_2", kinematics=kin)
    motor3 = Axis(can_if, motor_can_id=3, name="MKS_Motor_3", kinematics=kin)
    motor4 = Axis(can_if, motor_can_id=4, name="MKS_Motor_4", kinematics=kin)
    motor5 = Axis(can_if, motor_can_id=5, name="MKS_Motor_5", kinematics=kin)
    motor6 = Axis(can_if, motor_can_id=6, name="MKS_Motor_6", kinematics=kin)

    try:
        print("Initializing motor tracking...")
        await motor1.initialize(calibrate=False, home=False)
        await motor1.enable_motor()
        
        await motor2.initialize(calibrate=False, home=False)
        await motor2.enable_motor()
        
        await motor3.initialize(calibrate=False, home=False)
        await motor3.enable_motor()

        await motor4.initialize(calibrate=False, home=False)
        await motor4.enable_motor()

        await motor5.initialize(calibrate=False, home=False)
        await motor5.enable_motor()

        await motor6.initialize(calibrate=False, home=False)
        await motor6.enable_motor()
        
        print("✅ All 6 Motors ready!")
        return True
    except Exception as e:
        print(f"❌ Motor initialization failed: {e}")
        return False

def start_asyncio_loop():
    """Démarre une boucle asyncio en arrière-plan dans un thread séparé"""
    global motor_loop
    motor_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(motor_loop)
    success = motor_loop.run_until_complete(init_motor_async())
    if not success:
        print("⚠️ Les moteurs n'ont pas pu être initialisés. Le serveur démarrera quand même.")
    motor_loop.run_forever()

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

# --- MOTOR 1 ROUTES ---
@app.route('/forward')
def forward():
    if motor1 is None or motor_loop is None: return "Motor not initialized", 500
    print("Action: Moving Motor 1 Forward")
    try:
        future_pos = asyncio.run_coroutine_threadsafe(motor1.get_current_position_user(), motor_loop)
        pos_mot1 = future_pos.result()
        future_move = asyncio.run_coroutine_threadsafe(
            motor1.move_to_position_abs_user(pos_mot1 + 200, speed_user=100000.0, wait=True), motor_loop
        )
        future_move.result()
        return "Forward", 200
    except Exception as e: return f"Error: {e}", 500

@app.route('/backward')
def backward():
    if motor1 is None or motor_loop is None: return "Motor not initialized", 500
    print("Action: Moving Motor 1 Backward")
    try:
        future_pos = asyncio.run_coroutine_threadsafe(motor1.get_current_position_user(), motor_loop)
        pos_mot1 = future_pos.result()
        future_move = asyncio.run_coroutine_threadsafe(
            motor1.move_to_position_abs_user(pos_mot1 - 200, speed_user=100000.0, wait=True), motor_loop
        )
        future_move.result()
        return "Backward", 200
    except Exception as e: return f"Error: {e}", 500

# --- MOTOR 2 ROUTES ---
@app.route('/upward')
def upward():
    if motor2 is None or motor_loop is None: return "Motor not initialized", 500
    print("Action: Moving Motor 2 Upward")
    try:
        future_pos = asyncio.run_coroutine_threadsafe(motor2.get_current_position_user(), motor_loop)
        pos_mot2 = future_pos.result()
        future_move = asyncio.run_coroutine_threadsafe(
            motor2.move_to_position_abs_user(pos_mot2 - 500, speed_user=100000.0, wait=True), motor_loop
        )
        future_move.result()
        return "upward", 200
    except Exception as e: return f"Error: {e}", 500

@app.route('/downward')
def downward():
    if motor2 is None or motor_loop is None: return "Motor not initialized", 500
    print("Action: Moving Motor 2 Downward")
    try:
        future_pos = asyncio.run_coroutine_threadsafe(motor2.get_current_position_user(), motor_loop)
        pos_mot2 = future_pos.result()
        future_move = asyncio.run_coroutine_threadsafe(
            motor2.move_to_position_abs_user(pos_mot2 + 500, speed_user=100000.0, wait=True), motor_loop
        )
        future_move.result()
        return "downward", 200
    except Exception as e: return f"Error: {e}", 500

# --- MOTOR 3 ROUTES ---
@app.route('/upward2')
def upward2():
    if motor3 is None or motor_loop is None: return "Motor not initialized", 500
    print("Action: Moving Motor 3 Upward")
    try:
        future_pos = asyncio.run_coroutine_threadsafe(motor3.get_current_position_user(), motor_loop)
        pos_mot3 = future_pos.result()
        future_move = asyncio.run_coroutine_threadsafe(
            motor3.move_to_position_abs_user(pos_mot3 - 400, speed_user=100000.0, wait=True), motor_loop
        )
        future_move.result()
        return "upward2", 200
    except Exception as e: return f"Error: {e}", 500

@app.route('/downward2')
def downward2():
    if motor3 is None or motor_loop is None: return "Motor not initialized", 500
    print("Action: Moving Motor 3 Downward")
    try:
        future_pos = asyncio.run_coroutine_threadsafe(motor3.get_current_position_user(), motor_loop)
        pos_mot3 = future_pos.result()
        future_move = asyncio.run_coroutine_threadsafe(
            motor3.move_to_position_abs_user(pos_mot3 + 400, speed_user=100000.0, wait=True), motor_loop
        )
        future_move.result()
        return "downward2", 200
    except Exception as e: return f"Error: {e}", 500

# --- MOTOR 4 ROUTES ---
@app.route('/rot_clockwise')
def rot_clockwise():
    if motor4 is None or motor_loop is None: return "Motor not initialized", 500
    print("Action: Rotating Motor 4 Clockwise")
    try:
        future_pos = asyncio.run_coroutine_threadsafe(motor4.get_current_position_user(), motor_loop)
        pos_mot4 = future_pos.result()
        future_move = asyncio.run_coroutine_threadsafe(
            motor4.move_to_position_abs_user(pos_mot4 + 200, speed_user=100000.0, wait=True), motor_loop
        )
        future_move.result()
        return "rot_clockwise", 200
    except Exception as e: return f"Error: {e}", 500

@app.route('/rot_counter_clockwise')
def rot_counter_clockwise():
    if motor4 is None or motor_loop is None: return "Motor not initialized", 500
    print("Action: Rotating Motor 4 Counter-Clockwise")
    try:
        future_pos = asyncio.run_coroutine_threadsafe(motor4.get_current_position_user(), motor_loop)
        pos_mot4 = future_pos.result()
        future_move = asyncio.run_coroutine_threadsafe(
            motor4.move_to_position_abs_user(pos_mot4 - 200, speed_user=100000.0, wait=True), motor_loop
        )
        future_move.result()
        return "rot_counter_clockwise", 200
    except Exception as e: return f"Error: {e}", 500

# --- MOTOR 5 ROUTES ---
@app.route('/rot_clockwise2')
def rot_clockwise2():
    if motor5 is None or motor_loop is None: return "Motor not initialized", 500
    print("Action: Rotating Motor 5 Clockwise")
    try:
        future_pos = asyncio.run_coroutine_threadsafe(motor5.get_current_position_user(), motor_loop)
        pos_mot5 = future_pos.result()
        future_move = asyncio.run_coroutine_threadsafe(motor5.move_to_position_abs_user(pos_mot5 + 250, speed_user=100000.0, wait=True), motor_loop)
        future_move.result()
        return "rot_clockwise2", 200
    except Exception as e: 
        return f"Error: {e}", 500

@app.route('/rot_counter_clockwise2')
def rot_counter_clockwise2():
    if motor5 is None or motor_loop is None: 
        return "Motor not initialized", 500
    print("Action: Rotating Motor 5 Counter-Clockwise")
    try:
        future_pos = asyncio.run_coroutine_threadsafe(motor5.get_current_position_user(), motor_loop)
        pos_mot5 = future_pos.result()
        future_move = asyncio.run_coroutine_threadsafe(
            motor5.move_to_position_abs_user(pos_mot5 - 250, speed_user=100000.0, wait=True), motor_loop
        )
        future_move.result()
        return "rot_counter_clockwise2", 200
    except Exception as e: 
        return f"Error: {e}", 500

# --- MOTOR 6 ROUTES ---
@app.route('/rot_clockwise3')
def rot_clockwise3():
    if motor6 is None or motor_loop is None: return "Motor not initialized", 500
    print("Action: Rotating Motor 6 Clockwise")
    try:
        future_pos = asyncio.run_coroutine_threadsafe(motor6.get_current_position_user(), motor_loop)
        pos_mot6 = future_pos.result()
        future_move = asyncio.run_coroutine_threadsafe(motor6.move_to_position_abs_user(pos_mot6 + 250, speed_user=100000.0, wait=True), motor_loop)
        future_move.result()
        return "rot_clockwise6", 200
    except Exception as e: 
        return f"Error: {e}", 500

@app.route('/rot_counter_clockwise3')
def rot_counter_clockwise3():
    if motor6 is None or motor_loop is None: 
        return "Motor not initialized", 500
    print("Action: Rotating Motor 6 Counter-Clockwise")
    try:
        future_pos = asyncio.run_coroutine_threadsafe(motor6.get_current_position_user(), motor_loop)
        pos_mot6 = future_pos.result()
        future_move = asyncio.run_coroutine_threadsafe(
            motor6.move_to_position_abs_user(pos_mot6 - 250, speed_user=100000.0, wait=True), motor_loop
        )
        future_move.result()
        return "rot_counter_clockwise6", 200
    except Exception as e: 
        return f"Error: {e}", 500

# --- MOTOR 7 ROUTES ---

@app.route('/open')
def open_servo():
    global current_servo_angle
    print("Action: Opening GPIO Servo (+10 deg)")
    # Enforce safe physical boundaries (0 to 180 degrees)
    current_servo_angle = min(180, current_servo_angle + 50)
    servo_pwm.ChangeDutyCycle(angle_to_duty_cycle(current_servo_angle))
    time.sleep(0.5)              # Give the servo motor time to spin to position
    servo_pwm.ChangeDutyCycle(0)   # Jitter prevention cut-off
    return f"Opened to {current_servo_angle}", 200

@app.route('/close')
def close_servo():
    global current_servo_angle
    print("Action: Closing GPIO Servo (-10 deg)")
    current_servo_angle = max(0, current_servo_angle - 50)
    servo_pwm.ChangeDutyCycle(angle_to_duty_cycle(current_servo_angle))
    time.sleep(0.5)
    servo_pwm.ChangeDutyCycle(0)
    return f"Closed to {current_servo_angle}", 200

@app.route('/stop')
def stop():
    print("Action: Stopping All Motors")
    return "Stopped", 200

if __name__ == '__main__':
    # Initialize GPIO Pin Configuration
    init_gpio_servo()
    t = threading.Thread(target=start_asyncio_loop, daemon=True)
    t.start()
    time.sleep(1)
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)