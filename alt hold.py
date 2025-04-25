import time
import serial
import struct

# === SERIAL SETUP ===
ser = serial.Serial("/dev/ttyAMA0", 115200, timeout=0.01)

# === MSP HELPER ===
def send_msp_override(serial_port, channels):
    payload = b''.join([struct.pack('<H', ch) for ch in channels])
    size = len(payload)
    message_id = 200  # MSP_SET_RAW_RC

    checksum = size ^ message_id
    for b in payload:
        checksum ^= b

    serial_port.write(b'$M<' + bytes([size, message_id]) + payload + bytes([checksum]))

# === PID CLASS ===
class PID:
    def __init__(self, kp, ki, kd, integrator_limit=1000):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integrator = 0
        self.prev_error = 0
        self.integrator_limit = integrator_limit

    def compute(self, error, dt):
        self.integrator += error * dt
        self.integrator = max(min(self.integrator, self.integrator_limit), -self.integrator_limit)
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integrator + self.kd * derivative
        self.prev_error = error
        return output

# === SIMULATED ALTITUDE SENSOR ===
current_alt = 0.0
climb_velocity = 0.0

def simulate_altitude(throttle, hover_throttle, dt):
    global current_alt, climb_velocity
    thrust = (throttle - hover_throttle) / 400.0  # Simple linear model
    climb_velocity += thrust * dt
    climb_velocity *= 0.98  # some air resistance
    current_alt += climb_velocity * dt
    return current_alt, climb_velocity

# === PARAMETERS ===
target_alt = 5.0  # meters
hover_throttle = 1500  # starting guess
learning_rate = 0.05

min_throttle = 1100
max_throttle = 1900

outer_pid = PID(kp=1.5, ki=0.0, kd=0.5)
inner_pid = PID(kp=400, ki=30, kd=70)

last_time = time.time()
last_alt = 0.0

# === FLIGHT LOOP ===
for i in range(300):  # Run for ~6s at 50Hz
    now = time.time()
    dt = now - last_time
    last_time = now

    # Simulate altitude
    current_alt, climb_rate = simulate_altitude(hover_throttle, hover_throttle, dt)

    # Outer PID: Altitude → Desired climb rate
    alt_error = target_alt - current_alt
    desired_climb = outer_pid.compute(alt_error, dt)

    # Inner PID: Climb rate → Throttle output
    climb_error = desired_climb - climb_rate
    throttle_output = hover_throttle + inner_pid.compute(climb_error, dt)
    throttle_output = int(min(max(throttle_output, min_throttle), max_throttle))

    # === Learn hover throttle if stable ===
    if abs(climb_rate) < 0.05:
        hover_throttle = (1 - learning_rate) * hover_throttle + learning_rate * throttle_output

    # === Send MSP override ===
    rc_channels = [1500] * 8
    rc_channels[2] = throttle_output  # Channel 2 = THROTTLE
    send_msp_override(ser, rc_channels)

    # === Log ===
    print(f"[{i:03}] Alt: {current_alt:.2f} m | Climb: {climb_rate:.2f} m/s | "
          f"Throttle: {throttle_output} | HoverThrottle: {int(hover_throttle)}")

    time.sleep(0.02)  # 50Hz loop
