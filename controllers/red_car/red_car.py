import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))

from car_base import BaseCarController

class RedCarController(BaseCarController):
    def __init__(self):
        super().__init__(
            robot_id="red_car",
            emitter_name="red_car_emitter",
            receiver_name="red_car_receiver",
        )

car = RedCarController()


while car.step() != -1:
    car.update_communication()
    car.update_motion()
