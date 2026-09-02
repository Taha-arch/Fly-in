"""Turn-based execution of a fleet of drones across a `Network`.

Each turn is resolved in three passes: drones finishing a restricted-zone
crossing land first, then remaining in-flight drones tick down, then every
drone still waiting attempts to depart, processed furthest-along-its-route
first. Handling the furthest drones before the ones behind them lets a
zone freed up by a departure be claimed the very same turn by whoever is
queued behind it, instead of falsely reporting it as blocked.
"""

from __future__ import annotations

from dataclasses import dataclass

from network import Network
from zones import MOVE_TURNS

_STALL_LIMIT = 20


class DeadlockError(Exception):
    """Raised when no drone can move for too many consecutive turns."""


@dataclass
class Drone:
    """A single drone following a pre-assigned route."""

    drone_id: int
    route: list[str]
    step: int = 0
    in_transit: bool = False
    turns_remaining: int = 0
    transit_link: tuple[str, str] | None = None

    @property
    def delivered(self) -> bool:
        """Whether this drone has reached the last zone of its route."""
        return not self.in_transit and self.step == len(self.route) - 1


class Simulation:
    """Advances a fleet of drones turn by turn over a single `Network`."""

    def __init__(self, net: Network, drones: list[Drone]) -> None:
        self.net = net
        self.drones = drones
        self._zone_load: dict[str, int] = {
            name: 0 for name in net.zones if name not in (net.start, net.end)
        }
        self._link_load: dict[tuple[str, str], int] = {
            link.key(): 0 for link in net.links
        }

    def run(self) -> list[list[str]]:
        """Simulate turns until every drone is delivered.

        Returns one list of move descriptions per turn, in the format
        ``D<id>-<zone>`` for an arrival and ``D<id>-<from>-<to>`` for a
        drone entering a restricted-zone crossing.
        """
        turns: list[list[str]] = []
        stalled_for = 0
        while not all(drone.delivered for drone in self.drones):
            moves = self._advance_turn()
            turns.append(moves)
            stalled_for = 0 if moves else stalled_for + 1
            if stalled_for >= _STALL_LIMIT:
                raise DeadlockError(
                    f"no drone moved for {_STALL_LIMIT} consecutive turns"
                )
        return turns

    def _advance_turn(self) -> list[str]:
        moves: list[str] = []

        landing = [
            d for d in self.drones if d.in_transit and d.turns_remaining == 1
        ]
        for drone in sorted(landing, key=lambda d: -d.step):
            moves.append(self._land(drone))

        for drone in self.drones:
            if drone.in_transit and drone.turns_remaining > 1:
                drone.turns_remaining -= 1

        waiting = [
            d for d in self.drones if not d.in_transit and not d.delivered
        ]
        for drone in sorted(waiting, key=lambda d: -d.step):
            move = self._try_depart(drone)
            if move is not None:
                moves.append(move)

        return moves

    def _try_depart(self, drone: Drone) -> str | None:
        origin = drone.route[drone.step]
        destination = drone.route[drone.step + 1]
        dest_zone = self.net.zone(destination)
        link = self.net.link_between(origin, destination)
        link_key = link.key()

        if not dest_zone.has_room(self._zone_load.get(destination, 0)):
            return None
        if self._link_load[link_key] >= link.capacity:
            return None

        if destination not in (self.net.start, self.net.end):
            self._zone_load[destination] += 1
        if origin not in (self.net.start, self.net.end):
            self._zone_load[origin] -= 1
        self._link_load[link_key] += 1

        turns = MOVE_TURNS[dest_zone.kind]
        drone.step += 1

        if turns == 1:
            self._link_load[link_key] -= 1
            return f"D{drone.drone_id}-{destination}"

        drone.in_transit = True
        drone.turns_remaining = turns - 1
        drone.transit_link = link_key
        return f"D{drone.drone_id}-{origin}-{destination}"

    def _land(self, drone: Drone) -> str:
        assert drone.transit_link is not None
        self._link_load[drone.transit_link] -= 1
        drone.in_transit = False
        drone.turns_remaining = 0
        drone.transit_link = None
        return f"D{drone.drone_id}-{drone.route[drone.step]}"
