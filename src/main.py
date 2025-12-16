from src.quadrocopter.quadrocopter import Quadrocopter
from src.core.config import settings
from src.core.sim_helper import sim


def main():
    scene_map = settings.scene_map
    quadrocopter = Quadrocopter()

    try:
        sim.startSimulation()
        print("\nСоединение с сервером установлено, симуляция запущена!")
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
    except:
        print("\nПроизошла ошибка.")


if __name__ == "__main__":
    main()