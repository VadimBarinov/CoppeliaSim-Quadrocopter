import time

from src.quadrocopter.sonar_sensor import SonarSensor
from src.quadrocopter.target_control import TargetControl
from src.quadrocopter.vision_sensor import VisionSensor

from src.util import sim
from src.core.config import settings


class Quadrocopter:
    obj_found: bool | None = None
    msg: str = ""

    def __init__(
            self,
            ref_object: int = settings.quadrocopter.ref_object,
            server_host: str = settings.app.host,
            server_port: int = settings.app.port,
            v_min: float = settings.quadrocopter.v_min,
    ) -> None:
        self.server_host = server_host
        self.server_port = server_port
        self.ref_object = ref_object
        self.v_min = v_min

        self.client_id = self._start_server()
        self.target = self._create_target_control()
        self.vision = self._create_vision_sensor()
        self.sonar = self._create_sonar_sensor()

    def _start_server(self) -> int:
        return sim.simxStart(
            connectionAddress=self.server_host,
            connectionPort=self.server_port,
            waitUntilConnected=True,
            doNotReconnectOnceDisconnected=True,
            timeOutInMs=2000,
            commThreadCycleInMs=5,
        )

    def _create_target_control(self) -> TargetControl:
        return TargetControl(
            client_id=self.client_id,
        )

    def _create_vision_sensor(self) -> VisionSensor:
        return VisionSensor(
            client_id=self.client_id,
        )

    def _create_sonar_sensor(self) -> SonarSensor:
        return SonarSensor(
            client_id=self.client_id,
        )

    def _finish_server(self) -> None:
        sim.simxFinish(
            client_id=self.client_id,
        )

    def start_position(
            self,
            v_start: float,
            scene_map: settings.scene_map.__class__,
    ) -> None:
        pos = self.target.get_position()
        x = scene_map.x_min
        y = scene_map.y_min
        z = scene_map.z_max
        while not (pos[0] <= x and pos[1] <= y and pos[2] >= z):
            if pos[2] < z:
                x_new = pos[0]
                y_new = pos[1]
                z_new = pos[2] + v_start
            else:
                x_new = pos[0] - v_start if pos[0] > x else pos[0]
                y_new = pos[1] - v_start if pos[1] > y else pos[1]
                z_new = pos[2]
            
            self.target.set_position(
                x=x_new,
                y=y_new,
                z=z_new,
            )
            time.sleep(0.05)
            
            image = self.vision.get_image()
            if self.ref_object in image:
                self.obj_found = True
                break
            
            pos = self.target.get_position()
            
    def search_object(
            self,
            scene_map: settings.scene_map.__class__,
    ) -> None:
        y_control = 0
        x_enable = True
        y_enable = False
        boss = False
        while sim.simxGetConnectionId(self.client_id) != -1:
            x, y, z = self.target.get_position()
            if x >= scene_map.x_min and y >= scene_map.y_max:
                break 
            
            if x_enable:
                if (x < scene_map.x_min and self.v_min < 0) or (x > scene_map.x_max and self.v_min > 0):
                    self.v_min = self.v_min * (-1)
                    x_enable = False
                    y_enable = True
                
                if scene_map.x_min + 1 < x < scene_map.x_max - 1 and not boss:
                    boss = True
                else:
                    if x <= scene_map.x_min + 1 or x >= scene_map.x_max - 1 and boss:
                        boss = False

                v = self.v_min * 4 if boss else self.v_min
                x = x + v

            if y_enable:
                y = y + 0.1
                y_control = y_control + 1
                if y_control >= 40:
                    y_control = 0
                    y_enable = False
                    x_enable = True

            self.target.set_position(
                x=x,
                y=y,
                z=z,
            )
            time.sleep(0.1)

            image = self.vision.get_image()
            if self.ref_object in image:
                self.obj_found = True
                self.v_min = self.v_min/10
                return
        self.obj_found = False
        self.msg = "\nОбъект не найден."

    def land(
            self,
            scene_map: settings.scene_map.__class__,
    ) -> bool:
        height = 0
        not_found = 0
        while True:
            x, y, z = self.target.get_position()
            if z <= scene_map.z_min:
                self.msg = "\nОбъект найден!\nОн внизу!"
                return True
            
            image = self.vision.get_image()
            orientation, direction = self.vision.get_position_object(
                image=image,
                ref_object=self.ref_object,
            )
            if orientation != -1 and direction != -1:
                if orientation == 0 and direction == 0 and z <= scene_map.z_min:
                    self.msg = "\nОбъект найден!\nОн внизу!"
                    return True 
                
                if orientation == 0 or direction == 0:
                    height = -0.01

                if z <= scene_map.z_max / 2:
                    height = height * 10
                    self.v_min = self.v_min / 10
                
                if not self.sonar.get_state_collision():
                    x_new = x + orientation
                    y_new = y + direction
                    z_new = z + height
                    self.target.set_position(
                        x=x_new,
                        y=y_new,
                        z=z_new,
                    )
                else:
                    self.msg = "\nОбъект найден!\nОн внизу!\nНе могу спуститься, слишком опасно..."
                    return True
                    
                height = 0
                not_found = 0
            else:
                x_new = x - self.v_min
                self.target.set_position(
                    x=x_new,
                    y=y,
                    z=z,
                )
                not_found += 1
            
            if not_found > 500:
                self.msg = "\nОбъект потерян."
                self.v_min = self.v_min * 10
                return False

            time.sleep(0.05)
