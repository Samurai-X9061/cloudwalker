import time

class PID:
    def __init__(self, kp, ki, kd, integrator_limit=100):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integrator = 0
        self.prev_error = 0
        self.integrator_limit = integrator_limit

    def compute(self, error, dt, output_limit = 200):
        self.integrator += error * dt
        self.integrator = max(min(self.integrator, self.integrator_limit), -self.integrator_limit)
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integrator + self.kd * derivative
        self.prev_error = error
        return max(min(output, output_limit), -output_limit)

class AltController:

    def __init__(self, height):
        self.timelast = time.time()
        self.height = height
        self.hoverthrottle = 1400  # Absolute hoverthrottle
        self.hover = self.hoverthrottle  # Dynamic hoverthrottle
        self.learning_rate = 0.05  # adaptive hover throttle
        self.throttle_output =  self.hover
        self.min_throttle = 1100
        self.max_throttle = 1900
        self.velocity = 0
        self.heightPID = PID(kp=1.5, ki=0.0, kd=0.5)
        self.veloPID = PID(kp=200, ki=0.5, kd=100)

    def __adaptivehover(self):
        if abs(self.velocity) < 0.05:
            self.hover = (1 - self.learning_rate) * self.hover + self.learning_rate * self.throttle_output

    def update(self, height, target_alt):
        now = time.time()    # Time management
        dt = now - self.timelast
        self.timelast = now

        dh = height - self.height # Height mangement
        self.height = height

        self.velocity = dh/dt # current velocity

        # Outer PID: Altitude → Desired climb rate
        alt_error = target_alt - self.height
        desired_velocity = self.heightPID.compute(alt_error, dt)
        
        if abs(alt_error) <= 0.05:
            self.desired_velocity = 0
            self.veloPID.integrator = 0

        velocity_error = desired_velocity - self.velocity

        # Inner PID: Climb rate → Throttle output
        self.throttle_output = self.hover + self.veloPID.compute(velocity_error, dt)
        self.throttle_output = int(min(max(self.throttle_output, self.min_throttle), self.max_throttle))

        #self.__adaptivehover()

        return self.throttle_output