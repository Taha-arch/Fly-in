"""Route discovery and fleet assignment for a parsed `Network`.

Pathfinding is plain Dijkstra over fractional zone weights (see
`zones.SEARCH_WEIGHT`), which naturally prefers priority zones without
pretending they are faster than the one real turn they cost. To spread
the fleet across a congested map, several diverse routes are discovered
by lightly penalising zones already used by an earlier route, and drones
are then assigned greedily to whichever route currently offers the best
estimated completion turn given its narrowest capacity.
"""

from __future__ import annotations

import heapq
import math

from network import Network
from zones import MOVE_TURNS, SEARCH_WEIGHT, ZoneKind


class RouteError(Exception):
    """Raised when no path exists between the start and end zones."""


def shortest_path(
    net: Network,
    start: str,
    end: str,
    penalty: dict[str, float] | None = None,
) -> list[str] | None:
    """Cheapest start-to-end route, or None if the zones are disconnected.

    `penalty` adds an extra weight to specific zones, which is how
    `discover_routes` steers successive searches away from routes it has
    already found.
    """
    penalty = penalty or {}
    best: dict[str, float] = {start: 0.0}
    previous: dict[str, str] = {}
    frontier: list[tuple[float, str]] = [(0.0, start)]
    settled: set[str] = set()

    while frontier:
        cost, zone_name = heapq.heappop(frontier)
        if zone_name in settled:
            continue
        settled.add(zone_name)
        if zone_name == end:
            break
        for link in net.links_from(zone_name):
            neighbour = link.other_end(zone_name)
            zone = net.zone(neighbour)
            if zone.kind is ZoneKind.BLOCKED:
                continue
            weight = SEARCH_WEIGHT[zone.kind] + penalty.get(neighbour, 0.0)
            new_cost = cost + weight
            if new_cost < best.get(neighbour, math.inf):
                best[neighbour] = new_cost
                previous[neighbour] = zone_name
                heapq.heappush(frontier, (new_cost, neighbour))

    if end not in best:
        return None
    path = [end]
    while path[-1] != start:
        path.append(previous[path[-1]])
    path.reverse()
    return path


def discover_routes(
    net: Network, start: str, end: str, limit: int = 6
) -> list[list[str]]:
    """Find up to `limit` diverse start-to-end routes."""
    penalty: dict[str, float] = {}
    routes: list[list[str]] = []
    for _ in range(limit):
        route = shortest_path(net, start, end, penalty)
        if route is None or route in routes:
            break
        routes.append(route)
        for zone_name in route[1:-1]:
            penalty[zone_name] = penalty.get(zone_name, 0.0) + 3.0
    if not routes:
        raise RouteError(f"no path found between '{start}' and '{end}'")
    return routes


def route_turn_cost(net: Network, route: list[str]) -> int:
    """Turns a lone, unobstructed drone would need to fly this route."""
    return sum(MOVE_TURNS[net.zone(name).kind] for name in route[1:])


def route_capacity(net: Network, route: list[str]) -> int | float:
    """The narrowest zone or link capacity along a route."""
    capacity: int | float = math.inf
    for zone_name in route[1:-1]:
        zone_capacity = net.zone(zone_name).capacity
        if zone_capacity is not None:
            capacity = min(capacity, zone_capacity)
    for here, there in zip(route, route[1:]):
        capacity = min(capacity, net.link_between(here, there).capacity)
    return capacity


def plan_fleet(
    net: Network, start: str, end: str, drone_count: int
) -> list[list[str]]:
    """Assign every drone a route, balancing load across discovered ones."""
    routes = discover_routes(net, start, end)
    turn_cost = [route_turn_cost(net, route) for route in routes]
    capacity = [route_capacity(net, route) for route in routes]
    load = [0] * len(routes)

    assignments: list[list[str]] = []
    for _ in range(drone_count):
        best = min(
            range(len(routes)),
            key=lambda i: turn_cost[i] + load[i] // capacity[i],
        )
        assignments.append(routes[best])
        load[best] += 1
    return assignments
