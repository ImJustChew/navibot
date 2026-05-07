from dataclasses import dataclass
from secrets import token_urlsafe


@dataclass(frozen=True)
class PairingCode:
    value: str


@dataclass(frozen=True)
class SecureSession:
    session_id: str
    peer_public_key: str


def generate_pairing_code() -> PairingCode:
    return PairingCode(value=token_urlsafe(16))

