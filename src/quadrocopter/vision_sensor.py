import time
import numpy as np

from src.core.config import settings
from src.util import (
    sim,
    simConst,
)


class VisionSensor:
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

    def get_image(self) -> np.ndarray:
        error, res, image = sim.simxGetVisionSensorImage(
            clientID=self.client_id,
            sensorHandle=self.id,
            options=0,
            operationMode=simConst.simx_opmode_buffer,
        )

        if not self.line:
            self.line = res[0]
            self.half = res[0] * res[1] // 2

        image = np.array(image).astype(np.uint8).reshape(-1, 3)
        return image

    def get_position_object(
            self,
            image: np.ndarray,
            ref_object: np.ndarray,
            v: float = settings.quadrocopter.v_vision_sensor,
    ) -> list:
        front = image[self.half:].copy()
        back = image[:self.half].copy()
        control = 0
        orientation = None
        if np.size(front) > 0:
            if np.any(np.all(front == ref_object, axis=1)):
                control = 0
                orientation = v
                while True:
                    control += 1
                    valor = front[-1, :].copy()
                    front = np.delete(front, -1, axis=0)
                    if np.array_equal(ref_object, valor):
                        break

                    if control == self.line:
                        control = 0

                    if np.size(front) == 0:
                        control = 0
                        break

        if np.size(back) > 0:
            if np.any(np.all(back == ref_object, axis=1)):
                control = 0
                orientation = -v if not orientation else 0
                while True:
                    control += 1
                    valor = back[-1, :].copy()
                    back = np.delete(back, -1, axis=0)
                    if np.array_equal(ref_object, valor):
                        break

                    if control == self.line:
                        control = 0

                    if np.size(back) == 0:
                        control = 0
                        break


        if control < 1:
            return [-1, -1]
        elif ((self.line / 2) - 1) <= control <= self.line / 2:
            return [orientation, 0]
        elif control <= self.line / 2:
            return [orientation, -v]
        else:
            return [orientation, v]