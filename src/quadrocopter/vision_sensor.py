import time
import numpy as np

from src.core.config import settings
from src.core.sim_helper import sim


class VisionSensor:
    line: int | None = None
    half: int | None = None

    def __init__(self) -> None:
        self.id = sim.getObjectHandle(settings.vision_sensor_name)
        sim.getVisionSensorImg(self.id)

        time.sleep(0.5)

    def get_image(self) -> np.ndarray:
        image, res = sim.getVisionSensorImg(self.id)

        if not self.line:
            self.line = res[0]
            self.half = res[0] * res[1] // 2

        image = np.frombuffer(image, dtype=np.uint8).reshape(-1, 3)
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