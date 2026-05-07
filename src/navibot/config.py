from dataclasses import dataclass
import os


@dataclass(frozen=True)
class Settings:
    host: str = "0.0.0.0"
    port: int = 8080
    env: str = "development"
    pairing_required: bool = True


def load_settings() -> Settings:
    return Settings(
        host=os.getenv("NAVIBOT_HOST", "0.0.0.0"),
        port=int(os.getenv("NAVIBOT_PORT", "8080")),
        env=os.getenv("NAVIBOT_ENV", "development"),
        pairing_required=os.getenv("NAVIBOT_PAIRING_REQUIRED", "true").lower() == "true",
    )

