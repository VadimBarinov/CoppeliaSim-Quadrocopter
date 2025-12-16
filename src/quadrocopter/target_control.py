import time

from src.core.config import settings
from src.core.sim_helper import sim


class TargetControl:
    def __init__(self) -> None:
        self.id = sim.getObjectHandle(settings.target_name)
        sim.getObjectPosition(self.id, -1)

        time.sleep(0.5)

    def get_position(self) -> list:
        return sim.getObjectPosition(self.id, -1)

    def set_position(self, x, y, z) -> None:
        sim.setObjectPosition(self.id, -1, [x, y, z])
