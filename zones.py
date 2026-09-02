"""Domain model for a single Fly-in zone and the link between two zones."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ZoneKind(Enum):
    """The four zone categories defined by the map format."""

    NORMAL = "normal"
    RESTRICTED = "restricted"
    PRIORITY = "priority"
    BLOCKED = "blocked"


#: Turns spent flying into a zone of a given kind. Blocked zones can never
#: be entered, so they intentionally have no entry here.
MOVE_TURNS: dict[ZoneKind, int] = {
    ZoneKind.NORMAL: 1,
    ZoneKind.RESTRICTED: 2,
    ZoneKind.PRIORITY: 1,
}

#: Fractional weight used only to bias pathfinding. Priority zones still
#: cost a single real turn (see MOVE_TURNS), but a slightly lower search
#: weight makes routes through them win ties over plain normal zones.
SEARCH_WEIGHT: dict[ZoneKind, float] = {
    ZoneKind.NORMAL: 1.0,
    ZoneKind.RESTRICTED: 2.0,
    ZoneKind.PRIORITY: 0.9,
}


@dataclass
class Zone:
    """A single hub in the drone network."""

    name: str
    x: int
    y: int
    kind: ZoneKind = ZoneKind.NORMAL
    color: str | None = None
    capacity: int | None = 1

    def has_room(self, occupied: int) -> bool:
        """Whether one more drone can currently enter this zone."""
        return self.capacity is None or occupied < self.capacity


@dataclass
class Link:
    """A bidirectional connection between two zones."""

    zone_a: str
    zone_b: str
    capacity: int = 1

    def other_end(self, zone_name: str) -> str:
        """The zone at the far end of this link from `zone_name`."""
        if zone_name == self.zone_a:
            return self.zone_b
        if zone_name == self.zone_b:
            return self.zone_a
        raise ValueError(f"{zone_name} is not part of this link")

    def key(self) -> tuple[str, str]:
        """A direction-independent identifier for this link."""
        a, b = sorted((self.zone_a, self.zone_b))
        return a, b
