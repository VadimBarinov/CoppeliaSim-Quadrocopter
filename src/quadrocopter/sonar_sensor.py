import time

from src.core.config import settings
from src.util import (
    sim,
    simConst,
)


class SonarSensor:
    def __init__(
            self,
            client_id: int,
    ) -> None:
        self.client_id = client_id
        self.id = sim.simxGetObjectHandle(
            clientID=client_id,
            objectName=settings.sonar_sensor_name,
            operationMode=simConst.simx_opmode_blocking,
        )[1]
        sim.simxReadProximitySensor(
            clientID=self.client_id,
            sensorHandle=self.id,
            operationMode=simConst.simx_opmode_streaming,
        )

        time.sleep(0.5)

    def get_state_collision(self):
        return sim.simxReadProximitySensor(
            clientID=self.client_id,
            sensorHandle=self.id,
            operationMode=simConst.simx_opmode_buffer,
        )[1]