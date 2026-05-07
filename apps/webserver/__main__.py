from navibot.config import load_settings
from navibot.server.app import create_app


def main() -> None:
    try:
        import uvicorn
    except ImportError as exc:
        msg = "Install server dependencies with: python -m pip install -e '.[server]'"
        raise RuntimeError(msg) from exc

    settings = load_settings()
    uvicorn.run(create_app(settings), host=settings.host, port=settings.port)


if __name__ == "__main__":
    main()

