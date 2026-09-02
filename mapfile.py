"""Reader for the Fly-in map text format described in the subject.

The format is line oriented: one directive per line, ``#`` starts a
comment, and blank lines are ignored. Every error raised while reading a
line is wrapped into a `MapError` carrying that line's number, so callers
get a precise, human-readable diagnostic.
"""

from __future__ import annotations

from network import Network
from zones import Link, Zone, ZoneKind

_ZONE_KEYS = {"zone", "color", "max_drones"}
_LINK_KEYS = {"max_link_capacity"}
_ZONE_PREFIXES = ("start_hub:", "end_hub:", "hub:")


class MapError(Exception):
    """Raised when a map file does not respect the expected format."""

    def __init__(self, line_number: int, reason: str) -> None:
        super().__init__(f"line {line_number}: {reason}")


def read_map(path: str) -> tuple[Network, int]:
    """Parse a map file into a `Network` and its declared drone count."""
    zones: dict[str, Zone] = {}
    links: list[Link] = []
    seen_links: set[tuple[str, str]] = set()
    start: str | None = None
    end: str | None = None
    drone_count: int | None = None
    line_number = 0

    with open(path, encoding="utf-8") as handle:
        for line_number, raw_line in enumerate(handle, start=1):
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue
            try:
                if drone_count is None:
                    drone_count = _parse_drone_count(line)
                elif line.startswith(_ZONE_PREFIXES):
                    start, end = _ingest_zone(line, zones, start, end)
                elif line.startswith("connection:"):
                    links.append(_ingest_link(line, zones, seen_links))
                else:
                    raise ValueError(f"unrecognised line '{line}'")
            except ValueError as exc:
                raise MapError(line_number, str(exc)) from exc

    if drone_count is None:
        raise MapError(line_number or 1, "missing 'nb_drones' declaration")
    if start is None:
        raise MapError(line_number, "no start_hub was declared")
    if end is None:
        raise MapError(line_number, "no end_hub was declared")

    zones[start].capacity = None
    zones[end].capacity = None
    return Network(zones=zones, links=links, start=start, end=end), drone_count


def _ingest_zone(
    line: str,
    zones: dict[str, Zone],
    start: str | None,
    end: str | None,
) -> tuple[str | None, str | None]:
    zone = _parse_zone(line)
    if zone.name in zones:
        raise ValueError(f"zone '{zone.name}' redefined")
    zones[zone.name] = zone
    if line.startswith("start_hub:"):
        if start is not None:
            raise ValueError("a second start_hub was found")
        start = zone.name
    elif line.startswith("end_hub:"):
        if end is not None:
            raise ValueError("a second end_hub was found")
        end = zone.name
    return start, end


def _ingest_link(
    line: str, zones: dict[str, Zone], seen_links: set[tuple[str, str]]
) -> Link:
    link = _parse_link(line, zones)
    if link.key() in seen_links:
        raise ValueError(
            f"connection '{link.zone_a}-{link.zone_b}' is a duplicate"
        )
    seen_links.add(link.key())
    return link


def _parse_drone_count(line: str) -> int:
    if not line.startswith("nb_drones:"):
        raise ValueError("expected 'nb_drones: <count>' as the first entry")
    return _parse_positive_int(line.split(":", 1)[1].strip(), "nb_drones")


def _parse_zone(line: str) -> Zone:
    prefix, rest = line.split(":", 1)
    body, meta = _split_metadata(rest)
    fields = body.split()
    if len(fields) != 3:
        raise ValueError(f"expected '{prefix}: <name> <x> <y> [metadata]'")
    name, x_text, y_text = fields
    if "-" in name:
        raise ValueError(f"zone name '{name}' must not contain '-'")

    values = _parse_key_values(meta, _ZONE_KEYS)
    kind = ZoneKind.NORMAL
    if "zone" in values:
        try:
            kind = ZoneKind(values["zone"])
        except ValueError:
            raise ValueError(f"unknown zone type '{values['zone']}'")
    color = values.get("color")
    if color is not None and not color.isalnum():
        raise ValueError(f"invalid color '{color}'")
    capacity = 1
    if "max_drones" in values:
        capacity = _parse_positive_int(values["max_drones"], "max_drones")

    return Zone(
        name=name,
        x=_parse_int(x_text, "x"),
        y=_parse_int(y_text, "y"),
        kind=kind,
        color=color,
        capacity=capacity,
    )


def _parse_link(line: str, zones: dict[str, Zone]) -> Link:
    _, rest = line.split(":", 1)
    body, meta = _split_metadata(rest)
    endpoints = [part.strip() for part in body.split("-")]
    if len(endpoints) != 2 or not all(endpoints):
        raise ValueError("expected 'connection: <nameA>-<nameB>'")
    name_a, name_b = endpoints
    for name in (name_a, name_b):
        if name not in zones:
            raise ValueError(f"connection refers to unknown zone '{name}'")
    if name_a == name_b:
        raise ValueError("a zone cannot connect to itself")

    values = _parse_key_values(meta, _LINK_KEYS)
    capacity = 1
    if "max_link_capacity" in values:
        capacity = _parse_positive_int(
            values["max_link_capacity"], "max_link_capacity"
        )
    return Link(zone_a=name_a, zone_b=name_b, capacity=capacity)


def _split_metadata(rest: str) -> tuple[str, str | None]:
    rest = rest.strip()
    if "[" not in rest:
        return rest, None
    if not rest.endswith("]"):
        raise ValueError("metadata block must end with ']'")
    body, _, meta = rest.partition("[")
    return body.strip(), meta[:-1].strip()


def _parse_key_values(meta: str | None, allowed: set[str]) -> dict[str, str]:
    if not meta:
        return {}
    values: dict[str, str] = {}
    for token in meta.split():
        if "=" not in token:
            raise ValueError(f"invalid metadata token '{token}'")
        key, _, value = token.partition("=")
        if key not in allowed:
            raise ValueError(f"unknown metadata key '{key}'")
        if key in values:
            raise ValueError(f"metadata key '{key}' repeated")
        values[key] = value
    return values


def _parse_int(text: str, label: str) -> int:
    try:
        return int(text)
    except ValueError:
        raise ValueError(f"{label} must be an integer, got '{text}'")


def _parse_positive_int(text: str, label: str) -> int:
    value = _parse_int(text, label)
    if value <= 0:
        raise ValueError(f"{label} must be a positive integer")
    return value
