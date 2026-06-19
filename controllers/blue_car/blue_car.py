import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))

from car_base import BaseCarController


class BlueCarController(BaseCarController):
    def __init__(self):
        super().__init__(
            robot_id="blue_car",
            emitter_name="blue_car_emitter",
            receiver_name="blue_car_receiver",
        )


car = BlueCarController()

while car.step() != -1:
    car.update_communication()
    car.update_motion()
