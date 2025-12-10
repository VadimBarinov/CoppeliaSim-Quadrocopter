from src.quadrocopter.quadrocopter import Quadrocopter
from src.core.config import settings


def main():
    scene_map = settings.scene_map
    quadrocopter = Quadrocopter()

    if quadrocopter.client_id != -1:
        print("\nСоединение с сервером установлено!")
        quadrocopter.start_position(
            v_start=settings.quadrocopter.v_start,
            scene_map=scene_map,
        )
        while not isinstance(quadrocopter.obj_found, bool):
            quadrocopter.search_object(scene_map)
        while quadrocopter.obj_found:
            if quadrocopter.land(scene_map):
                print(quadrocopter.msg)
                break
            else:
                quadrocopter.search_object(scene_map)
                print(quadrocopter.msg)
        else:
            print(quadrocopter.msg)
    else:
        print("\nНе удалось подключиться к серверу. Сервер отключен.")


if __name__ == "__main__":
    main()