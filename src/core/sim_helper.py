from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from core.config import settings

client = RemoteAPIClient(
    host=settings.app.host,
    port=settings.app.port,
)

sim = client.require("sim")