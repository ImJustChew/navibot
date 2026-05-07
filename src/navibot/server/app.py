from navibot.config import Settings, load_settings


def create_app(settings: Settings | None = None) -> object:
    settings = settings or load_settings()

    try:
        from fastapi import FastAPI
        from fastapi.responses import HTMLResponse
        from fastapi.staticfiles import StaticFiles
    except ImportError as exc:
        msg = "Install server dependencies with: python -m pip install -e '.[server]'"
        raise RuntimeError(msg) from exc

    app = FastAPI(title="Navibot", version="0.1.0")
    app.mount("/static", StaticFiles(directory="web/static"), name="static")

    @app.get("/health")
    def health() -> dict[str, str]:
        return {"status": "ok", "env": settings.env}

    @app.get("/", response_class=HTMLResponse)
    def portal() -> str:
        with open("web/templates/index.html", encoding="utf-8") as template:
            return template.read()

    return app

