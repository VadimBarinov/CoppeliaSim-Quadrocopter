import time
from typing import Any

from src.core.config import settings
from src.util import (
    sim,
    simConst,
)


class VisionSensor:
    resolution: int | None = None
    line: int | None = None
    half: int | None = None

    def __init__(
            self,
            client_id: int,
    ) -> None:
        self.client_id = client_id
        self.id = sim.simxGetObjectHandle(
            clientID=client_id,
            objectName=settings.vision_sensor_name,
            operationMode=simConst.simx_opmode_blocking
        )[1]
        sim.simxGetVisionSensorImage(
            clientID=client_id,
            sensorHandle=self.id,
            options=0,
            operationMode=simConst.simx_opmode_streaming,
        )

        time.sleep(0.5)

    def get_image(self) -> Any:
        if not self.resolution:
            error, res, image = sim.simxGetVisionSensorImage(
                clientID=self.client_id,
                sensorHandle=self.id,
                options=0,
                operationMode=simConst.simx_opmode_buffer,
            )
            self.resolution = res[0]
            self.line = self.resolution * 3
            self.half = self.line * self.resolution // 2

            return image

        return sim.simxGetVisionSensorImage(
            clientID=self.client_id,
            sensorHandle=self.id,
            options=0,
            operationMode=simConst.simx_opmode_buffer
        )[2]

    def get_position_object(
            self,
            image: Any,
            ref_object: int,
            v: float = settings.quadrocopter.v_vision_sensor,
    ) -> list:
        front = image[self.half:]
        back = image[:self.half]
        control = 0
        orientation = None
        searched = 0
        if ref_object in front:
            control = 0
            orientation = v
            while True:
                control += 1
                valor = front.pop()
                if ref_object == valor:
                    searched += 1
                    if searched >= 3:
                        break
                else:
                    searched = 0

                if control == self.line:
                    control = 0
                    searched = 0

                if len(front) == 0:
                    control = 0
                    searched = 0
                    break

        if ref_object in back:
            control = 0
            orientation = -v if not orientation else 0
            while True:
                control += 1
                valor = back.pop()
                if ref_object == valor:
                    searched += 1
                    if searched >= 3:
                        break
                else:
                    searched = 0

                if control == self.line:
                    control = 0
                    searched = 0

                if len(back) == 0:
                    control = 0
                    searched = 0
                    break

        if control == 0:
            return [-1, -1]
        elif ((self.line / 2) - 4) <= control <= self.line / 2:
            return [orientation, 0]
        elif control <= self.line / 2:
            return [orientation, -v]
        else:
            return [orientation, v]