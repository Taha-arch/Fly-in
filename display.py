"""ANSI colour helpers for the terminal simulation output."""

from __future__ import annotations

_RESET = "\033[0m"

_NAMED_COLORS: dict[str, str] = {
    "black": "30",
    "red": "31",
    "green": "32",
    "yellow": "33",
    "blue": "34",
    "magenta": "35",
    "purple": "35",
    "cyan": "36",
    "white": "37",
    "orange": "33",
    "gray": "90",
    "grey": "90",
    "pink": "95",
    "brown": "33",
}

_RAINBOW = ["31", "33", "32", "36", "34", "35"]
_FALLBACK_PALETTE = [f"38;5;{n}" for n in (208, 92, 51, 165, 172, 27, 202, 105)]


def colorize(text: str, color: str | None) -> str:
    """Wrap `text` in an ANSI escape for `color`, or return it unchanged."""
    if color is None:
        return text
    if color.lower() == "rainbow":
        return _rainbow(text)
    code = _NAMED_COLORS.get(color.lower(), _fallback_code(color))
    return f"\033[{code}m{text}{_RESET}"


def _rainbow(text: str) -> str:
    return "".join(
        f"\033[{_RAINBOW[i % len(_RAINBOW)]}m{ch}{_RESET}"
        for i, ch in enumerate(text)
    )


def _fallback_code(name: str) -> str:
    """Deterministic 256-colour code for a name outside the named palette."""
    index = sum(ord(ch) for ch in name) % len(_FALLBACK_PALETTE)
    return _FALLBACK_PALETTE[index]
