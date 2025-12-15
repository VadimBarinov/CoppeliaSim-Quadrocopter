from pathlib import Path
from pydantic import BaseModel
from pydantic_settings import (
    BaseSettings,
    SettingsConfigDict,
)


BASE_URL = Path(__file__).parent.parent


class SceneMapConfig(BaseModel):
    x_min: float = -10.5
    x_max: float = 10

    y_min: float = -9
    y_max: float = 10

    z_min: float = 0.5
    z_max: float = 3


class QuadrocopterConfig(BaseModel):
    # красный цвет
    ref_object: list = [255, 0, 0]

    v_start: float = 0.1
    v_min: float = 0.1
    v_vision_sensor: float = 0.02


class AppConfig(BaseModel):
    host: str
    port: int


class Settings(BaseSettings):
    model_config = SettingsConfigDict(
        env_file=(BASE_URL / ".env.template", BASE_URL / ".env"),
        case_sensitive=False,
        env_nested_delimiter="__",
    )
    app: AppConfig

    scene_map: SceneMapConfig = SceneMapConfig()
    quadrocopter: QuadrocopterConfig = QuadrocopterConfig()

    target_name: str = "Quadrocopter_target"
    sonar_sensor_name: str = "Proximity_sensor"
    vision_sensor_name: str = "Vision_sensor"


settings = Settings()