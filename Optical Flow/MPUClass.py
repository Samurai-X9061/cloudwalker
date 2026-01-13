import smbus
import time
import math

MPU6050_ADDR = 0x68

# Registers
PWR_MGMT_1 = 0x6B
GYRO_CONFIG = 0x1B
CONFIG = 0x1A
GYRO_XOUT_H = 0x43

class MPU6050:
    def __init__(self, bus=1):
        self.bus = smbus.SMBus(bus)
        self.bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x00)

        # Gyro ±2000 dps
        self.bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0x18)

        # DLPF ≈ 98 Hz (low delay)
        self.bus.write_byte_data(MPU6050_ADDR, CONFIG, 0x02)

        self.gyro_scale = 2000.0 / 32768.0 * math.pi / 180.0  # rad/s
        self.bias = [0.0, 0.0, 0.0]

        self.last_time = time.monotonic()

    def _read_word(self, reg):
        hi = self.bus.read_byte_data(MPU6050_ADDR, reg)
        lo = self.bus.read_byte_data(MPU6050_ADDR, reg + 1)
        val = (hi << 8) | lo
        return val - 65536 if val & 0x8000 else val

    def read_gyro(self):
        gx = self._read_word(GYRO_XOUT_H)
        gy = self._read_word(GYRO_XOUT_H + 2)
        gz = self._read_word(GYRO_XOUT_H + 4)

        return [
            gx * self.gyro_scale,
            gy * self.gyro_scale,
            gz * self.gyro_scale
        ]

    def update_bias(self, gyro, alpha=0.001):
        for i in range(3):
            self.bias[i] = (1 - alpha) * self.bias[i] + alpha * gyro[i]

    def get_rotation(self):
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now

        gyro = self.read_gyro()
        self.update_bias(gyro)

        wx = gyro[0] - self.bias[0]
        wy = gyro[1] - self.bias[1]
        wz = gyro[2] - self.bias[2]

        return wx * dt, wy * dt, wz * dt, dt
