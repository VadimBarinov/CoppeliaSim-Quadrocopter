import time

from src.core.config import settings
from src.util import (
    sim,
    simConst,
)


class TargetControl:
    def __init__(
            self,
            client_id: int,
    ) -> None:
        self.client_id = client_id
        self.id = sim.simxGetObjectHandle(
            clientID=client_id,
            objectName=settings.target_name,
            operationMode=simConst.simx_opmode_blocking,
        )[1]
        sim.simxGetObjectPosition(
            clientID=self.client_id,
            objectHandle=self.id,
            relativeToObjectHandle=-1,
            operationMode=simConst.simx_opmode_streaming,
        )

        time.sleep(0.5)

    def get_position(self):
        return sim.simxGetObjectPosition(
            clientID = self.client_id,
            objectHandle = self.id,
            relativeToObjectHandle = -1,
            operationMode = simConst.simx_opmode_buffer,
        )[1]

    def set_position(self, x, y, z):
        sim.simxSetObjectPosition(
            clientID=self.client_id,
            objectHandle=self.id,
            relativeToObjectHandle=-1,
            position=[x, y, z],
            operationMode=simConst.simx_opmode_oneshot,
        )
