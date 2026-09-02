"""Command-line entry point for the Fly-in drone simulation."""

from __future__ import annotations

import argparse
import sys

from display import colorize
from mapfile import MapError, read_map
from network import Network
from router import RouteError, plan_fleet
from simulate import DeadlockError, Drone, Simulation


def _build_drones(net: Network, drone_count: int) -> list[Drone]:
    routes = plan_fleet(net, net.start, net.end, drone_count)
    return [
        Drone(drone_id=i + 1, route=route) for i, route in enumerate(routes)
    ]


def _format_turn(net: Network, moves: list[str], use_color: bool) -> str:
    if not use_color:
        return " ".join(moves)
    formatted = []
    for move in moves:
        drone_tag, _, zone_names = move.partition("-")
        painted = "-".join(
            colorize(name, net.zone(name).color)
            for name in zone_names.split("-")
        )
        formatted.append(f"{drone_tag}-{painted}")
    return " ".join(formatted)


def main(argv: list[str] | None = None) -> int:
    """Parse a map, plan routes and run the turn-based simulation."""
    parser = argparse.ArgumentParser(description="Fly-in drone simulation")
    parser.add_argument("map_file", help="path to a Fly-in map file")
    parser.add_argument(
        "--no-color", action="store_true", help="disable coloured output"
    )
    args = parser.parse_args(argv)

    try:
        net, drone_count = read_map(args.map_file)
        drones = _build_drones(net, drone_count)
        turns = Simulation(net, drones).run()
    except FileNotFoundError:
        print(f"Error: map file '{args.map_file}' not found", file=sys.stderr)
        return 1
    except (MapError, RouteError, DeadlockError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    for moves in turns:
        print(_format_turn(net, moves, use_color=not args.no_color))

    print(
        f"\nDelivered {drone_count} drone(s) in {len(turns)} turn(s).",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
