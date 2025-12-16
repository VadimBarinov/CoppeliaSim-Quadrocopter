import time

from src.core.config import settings
from src.core.sim_helper import sim


class SonarSensor:
    def __init__(self) -> None:
        self.id = sim.getObjectHandle(settings.sonar_sensor_name)
        sim.readProximitySensor(self.id)

        time.sleep(0.5)

    def get_state_collision(self) -> bool:
        return sim.readProximitySensor(self.id)[0]