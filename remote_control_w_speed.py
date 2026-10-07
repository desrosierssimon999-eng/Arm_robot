from flask import Flask, render_template_string, request
import asyncio
import threading
import time
from mks_servo_can import CANInterface, Axis, RotaryKinematics, const
import RPi.GPIO as GPIO

app = Flask(__name__)

# Global instances
can_if = None
motors = {}  
motor_loop = None

# Track directions (1 = forward/CW, -1 = backward/CCW, 0 = stopped)
move_states = {f"motor{i}": 0 for i in range(1, 7)}
move_start_times = {f"motor{i}": 0.0 for i in range(1, 7)}
move_accumulated_units = {f"motor{i}": 0 for i in range(1, 7)}

SERVO_PIN = 2
servo_pwm = None
current_servo_angle = 0 

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Pi Joystick Control</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; user-select: none; -webkit-user-select: none; }
        body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; background: #f2f2f7; color: #1c1c1e; padding: 20px 10px; display: flex; flex-direction: column; align-items: center; }
        h1 { font-size: 26px; font-weight: 700; margin-bottom: 20px; text-align: center; }
        .container { width: 100%; max-width: 400px; display: flex; flex-direction: column; gap: 16px; }
        .motor-card { background: #ffffff; border-radius: 14px; padding: 14px; box-shadow: 0 2px 8px rgba(0,0,0,0.05); }
        .motor-title { font-size: 15px; font-weight: 600; color: #8e8e93; margin-bottom: 10px; text-transform: uppercase; letter-spacing: 0.5px; }
        .btn-group { display: flex; gap: 10px; }
        .btn { flex: 1; padding: 18px 8px; font-size: 16px; font-weight: 600; color: #ffffff; background: #007aff; border: none; border-radius: 10px; cursor: pointer; box-shadow: 0 2px 4px rgba(0,122,255,0.15); touch-action: none; }
        .btn:active { background: #0051a8; }
        .btn-stop { background: #ff3b30; padding: 20px; font-size: 20px; margin-top: 10px; box-shadow: 0 4px 10px rgba(255,59,48,0.25); }
    </style>
</head>
<body>
    <div class="container">
        <h1>Remote Control</h1>
        
        <div class="motor-card">
            <div class="motor-title">Motor 1 (Horizontal)</div>
            <div class="btn-group">
                <button class="btn" onmousedown="startMove(1, 1)" onmouseup="stopMove(1)" ontouchstart="handleTouchStart(event, 1, 1)" ontouchend="handleTouchEnd(event, 1)">FORWARD ⬆️</button>
                <button class="btn" onmousedown="startMove(1, -1)" onmouseup="stopMove(1)" ontouchstart="handleTouchStart(event, 1, -1)" ontouchend="handleTouchEnd(event, 1)">BACKWARD ⬇️</button>
            </div>
        </div>

        <div class="motor-card">
            <div class="motor-title">Motor 2 (Vertical)</div>
            <div class="btn-group">
                <button class="btn" onmousedown="startMove(2, -1)" onmouseup="stopMove(2)" ontouchstart="handleTouchStart(event, 2, -1)" ontouchend="handleTouchEnd(event, 2)">UPWARD ⬆️</button>
                <button class="btn" onmousedown="startMove(2, 1)" onmouseup="stopMove(2)" ontouchstart="handleTouchStart(event, 2, 1)" ontouchend="handleTouchEnd(event, 2)">DOWNWARD ⬇️</button>
            </div>
        </div>

        <div class="motor-card">
            <div class="motor-title">Motor 3 (Auxiliary)</div>
            <div class="btn-group">
                <button class="btn" onmousedown="startMove(3, -1)" onmouseup="stopMove(3)" ontouchstart="handleTouchStart(event, 3, -1)" ontouchend="handleTouchEnd(event, 3)">UPWARD 2 ⬆️</button>
                <button class="btn" onmousedown="startMove(3, 1)" onmouseup="stopMove(3)" ontouchstart="handleTouchStart(event, 3, 1)" ontouchend="handleTouchEnd(event, 3)">DOWNWARD 2 ⬇️</button>
            </div>
        </div>

        <div class="motor-card">
            <div class="motor-title">Motor 4 (Rotation 1)</div>
            <div class="btn-group">
                <button class="btn" onmousedown="startMove(4, 1)" onmouseup="stopMove(4)" ontouchstart="handleTouchStart(event, 4, 1)" ontouchend="handleTouchEnd(event, 4)">CW 🔄</button>
                <button class="btn" onmousedown="startMove(4, -1)" onmouseup="stopMove(4)" ontouchstart="handleTouchStart(event, 4, -1)" ontouchend="handleTouchEnd(event, 4)">CCW 🔄</button>
            </div>
        </div>

        <div class="motor-card">
            <div class="motor-title">Motor 5 (Rotation 2)</div>
            <div class="btn-group">
                <button class="btn" onmousedown="startMove(5, 1)" onmouseup="stopMove(5)" ontouchstart="handleTouchStart(event, 5, 1)" ontouchend="handleTouchEnd(event, 5)">CW 2 🔄</button>
                <button class="btn" onmousedown="startMove(5, -1)" onmouseup="stopMove(5)" ontouchstart="handleTouchStart(event, 5, -1)" ontouchend="handleTouchEnd(event, 5)">CCW 2 🔄</button>
            </div>
        </div>

        <div class="motor-card">
            <div class="motor-title">Motor 6 (Rotation 3)</div>
            <div class="btn-group">
                <button class="btn" onmousedown="startMove(6, 1)" onmouseup="stopMove(6)" ontouchstart="handleTouchStart(event, 6, 1)" ontouchend="handleTouchEnd(event, 6)">CW 3 🔄</button>
                <button class="btn" onmousedown="startMove(6, -1)" onmouseup="stopMove(6)" ontouchstart="handleTouchStart(event, 6, -1)" ontouchend="handleTouchEnd(event, 6)">CCW 3 🔄</button>
            </div>
        </div>

        <div class="motor-card">
            <div class="motor-title">Motor 7 (GPIO Servo)</div>
            <div class="btn-group">
                <button class="btn" onmousedown="startMove(7, 1)" onmouseup="stopMove(7)" ontouchstart="handleTouchStart(event, 7, 1)" ontouchend="handleTouchEnd(event, 7)">OPEN 🟢</button>
                <button class="btn" onmousedown="startMove(7, -1)" onmouseup="stopMove(7)" ontouchstart="handleTouchStart(event, 7, -1)" ontouchend="handleTouchEnd(event, 7)">CLOSE 🔴</button>
            </div>
        </div>

        <button class="btn btn-stop" onclick="emergencyStop()">EMERGENCY STOP 🛑</button>
    </div>

    <script>
        function startMove(motorId, direction) {
            fetch(`/move?motor=\${motorId}&dir=\${direction}`);
        }
        function stopMove(motorId) {
            fetch(`/move?motor=\${motorId}&dir=0`);
        }
        function emergencyStop() {
            fetch('/stop');
        }
        function handleTouchStart(e, motorId, direction) {
            e.preventDefault(); // Kills ghost mouse events on mobile touchscreens
            startMove(motorId, direction);
        }
        function handleTouchEnd(e, motorId) {
            e.preventDefault();
            stopMove(motorId);
        }
    </script>
</body>
</html>
"""

def angle_to_duty_cycle(angle):
    return 2.5 + (angle / 180.0) * 10.0

def init_gpio_servo():
    global servo_pwm
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    servo_pwm = GPIO.PWM(SERVO_PIN, 50)
    servo_pwm.start(angle_to_duty_cycle(current_servo_angle))
    time.sleep(0.3)
    servo_pwm.ChangeDutyCycle(0)

async def init_motor_async():
    global can_if, motors
    print("Initializing Linux SocketCAN Interface...")
    can_if = CANInterface(use_simulator=False, interface_type="socketcan", channel="can0", bitrate=500000)
    try:
        await can_if.connect()
        print("✅ Linux can0 linked successfully!")
    except Exception as hardware_error:
        print(f"❌ CONNECTION FAILED: {hardware_error}")
        return False

    kin = RotaryKinematics(steps_per_revolution=const.ENCODER_PULSES_PER_REVOLUTION)
    
    for i in range(1, 7):
        motors[f"motor{i}"] = Axis(can_if, motor_can_id=i, name=f"MKS_Motor_{i}", kinematics=kin)
        try:
            await motors[f"motor{i}"].initialize(calibrate=False, home=False)
            await motors[f"motor{i}"].enable_motor()
        except Exception as e:
            print(f"❌ Motor {i} initialization failed: {e}")
            return False
            
    print("✅ All 6 Motors ready!")
    return True

async def motor_processing_loop():
    """Asynchronous background loop running every 50ms to drive aggressive non-linear curves."""
    global current_servo_angle
    
    while True:
        for motor_key, direction in move_states.items():
            if direction != 0:
                motor = motors.get(motor_key)
                if motor:
                    hold_duration = time.time() - move_start_times[motor_key]
                    
                    # 🚀 TRULY EXPONENTIAL / CUBIC JOYSTICK PROFILE:
                    # Base step of 8 units + heavily accelerated polynomial factor proportional to time cubed (t^3)
                    # Gives hyper-precise nudges under 1s, but shifts gears drastically beyond 1.2s.
                    calculated_step = int(direction * (8 + 140.0 * (hold_duration ** 3)))
                    
                    # Absolute safety speed cap per 50ms chunk (e.g., max 400 units per loop iteration)
                    # Keeps the motor driver safe from massive immediate overflows if held for 5+ seconds
                    if abs(calculated_step) > 400:
                        calculated_step = direction * 400
                    
                    move_accumulated_units[motor_key] += abs(calculated_step)
                    
                    print(f"[{motor_key.upper()}] Step Delta: {abs(calculated_step)} | Cumulative: {move_accumulated_units[motor_key]} units (Time: {hold_duration:.2f}s)")
                    
                    try:
                        pos = await motor.get_current_position_user()
                        await motor.move_to_position_abs_user(
                            pos + calculated_step, 
                            speed_user=180000.0, # Increased max tracking speed parameter slightly to clear big steps
                            wait=False 
                        )
                    except Exception as e:
                        print(f"Error stepping {motor_key}: {e}")
                        
        if move_states.get("motor7", 0) != 0:
            dir7 = move_states["motor7"]
            current_servo_angle = max(0, min(180, current_servo_angle + (dir7 * 4)))
            print(f"[MOTOR7/SERVO] Angle: {current_servo_angle}°")
            servo_pwm.ChangeDutyCycle(angle_to_duty_cycle(current_servo_angle))
            
        await asyncio.sleep(0.05) 

def start_asyncio_loop():
    global motor_loop
    motor_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(motor_loop)
    success = motor_loop.run_until_complete(init_motor_async())
    if success:
        motor_loop.create_task(motor_processing_loop())
    motor_loop.run_forever()

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/move')
def move_motor():
    motor_num = request.args.get('motor')
    try:
        direction = int(request.args.get('dir', 0))
    except (ValueError, TypeError):
        direction = 0
        
    if motor_num == "7":
        if direction == 0 and move_states.get("motor7", 0) != 0:
            print("--- MOTOR7/SERVO RELEASED ---\n")
            servo_pwm.ChangeDutyCycle(0) 
        move_states["motor7"] = direction
        return "Servo State Changed", 200

    motor_key = f"motor{motor_num}"
    if motor_key not in move_states:
        return f"Invalid Motor ID: {motor_num}", 400
        
    if direction != 0 and move_states[motor_key] == 0:
        move_start_times[motor_key] = time.time()
        move_accumulated_units[motor_key] = 0
        print(f"\n--- {motor_key.upper()} HOLD STARTED ---")
        
    if direction == 0 and move_states[motor_key] != 0:
        print(f"--- {motor_key.upper()} RELEASED | Final Total: {move_accumulated_units[motor_key]} units ---\n")
        
    move_states[motor_key] = direction
    return f"{motor_key} set to direction {direction}", 200

@app.route('/stop')
def stop():
    print("Action: Stopping All Motors")
    for key in move_states:
        if move_states[key] != 0:
            print(f"--- {key.upper()} HALTED | Stopped at total: {move_accumulated_units[key]} units ---")
        move_states[key] = 0
    servo_pwm.ChangeDutyCycle(0)
    return "All Motions Stopped", 200

if __name__ == '__main__':
    init_gpio_servo()
    t = threading.Thread(target=start_asyncio_loop, daemon=True)
    t.start()
    time.sleep(1)
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
