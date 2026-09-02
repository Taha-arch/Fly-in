"""Static topology of a Fly-in map: its zones and the links between them."""

from __future__ import annotations

from dataclasses import dataclass, field

from zones import Link, Zone


@dataclass
class Network:
    """An immutable graph of zones and links, plus the start/end zone.

    A `Network` only describes topology and per-zone/per-link limits; it
    holds no information about where drones currently are. That live state
    belongs to the simulation that runs on top of it, so the same network
    can be reused across independent simulation runs.
    """

    zones: dict[str, Zone]
    links: list[Link]
    start: str
    end: str
    _adjacency: dict[str, list[Link]] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._adjacency = {name: [] for name in self.zones}
        for link in self.links:
            self._adjacency[link.zone_a].append(link)
            self._adjacency[link.zone_b].append(link)

    def zone(self, name: str) -> Zone:
        """Look up a zone by name."""
        return self.zones[name]

    def links_from(self, zone_name: str) -> list[Link]:
        """All links touching the given zone."""
        return self._adjacency[zone_name]

    def link_between(self, a: str, b: str) -> Link:
        """The link connecting two adjacent zones."""
        for link in self._adjacency[a]:
            if link.other_end(a) == b:
                return link
        raise ValueError(f"no link between {a} and {b}")
