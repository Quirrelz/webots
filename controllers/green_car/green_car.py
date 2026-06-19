import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))

from car_base import BaseCarController


class GreenCarController(BaseCarController):
    def __init__(self):
        super().__init__(
            robot_id="green_car",
            emitter_name="green_car_emitter",
            receiver_name="green_car_receiver",
        )

car = GreenCarController()

while car.step() != -1:
    car.update_communication()
    car.update_motion()
