from navibot.crypto.session import generate_pairing_code


def test_pairing_code_generation_returns_unique_values() -> None:
    first = generate_pairing_code()
    second = generate_pairing_code()

    assert first.value
    assert second.value
    assert first != second

