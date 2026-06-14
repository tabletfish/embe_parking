import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
ROVER_DIR = PROJECT_ROOT / "rover"
sys.path.insert(0, str(ROVER_DIR))

from base_ctrl import BaseController  # noqa: E402


def clip(value, limit):
    return max(-limit, min(limit, value))


def compute_wheel_speeds(steering, speed, max_steer=1.0, max_speed=0.5):
    steer = clip(steering, max_steer)
    spd = clip(speed, max_speed)
    base = abs(spd)

    if steer >= 0:
        left = base * (1.0 - 0.9 * steer)
        right = base
    else:
        left = base
        right = base * (1.0 + 0.9 * steer)

    if spd < 0:
        left, right = -left, -right

    return -clip(left, max_speed), -clip(right, max_speed)


class RoverDrive:
    def __init__(self, config):
        rover = config["rover"]
        self.max_speed = float(rover["max_speed"])
        self.max_steer = float(rover["max_steer"])
        self.base = BaseController(rover["serial_port"], int(rover["baud_rate"]))

    def send(self, steering, speed):
        left, right = compute_wheel_speeds(steering, speed, self.max_steer, self.max_speed)
        self.base.base_json_ctrl({"T": 1, "L": left, "R": right})
        return left, right

    def stop(self):
        self.base.base_json_ctrl({"T": 1, "L": 0.0, "R": 0.0})

