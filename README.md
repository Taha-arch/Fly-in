<i>This project has been created as part of the 42 curriculum by tel-atou.</i>

# Fly-in

## Description

Fly-in routes a fleet of drones from a shared start zone to a shared end
zone across a network of connected zones, in as few simultaneous
simulation turns as possible. Zones and connections carry their own
rules: a zone may only hold so many drones at once, a connection may only
carry so many drones at once, and a zone's type changes how long it takes
to fly into it (`normal`/`priority` cost one turn, `restricted` costs two,
`blocked` cannot be entered at all).

The project is pure Python: a hand-written parser reads the map file, a
small object graph represents the network, Dijkstra's algorithm with
zone-aware weights plans one route per drone, and a turn-based engine
plays every drone's route forward at once while enforcing capacity and
movement rules.

## Instructions

```sh
make install   # install flake8 and mypy
make run       # simulate maps/easy_linear.txt
make debug     # same, under pdb
make lint      # flake8 + mypy
make clean     # remove __pycache__ / .mypy_cache
```

Run any other map directly:

```sh
python3 main.py maps/hard_maze.txt
python3 main.py maps/hard_maze.txt --no-color   # plain text output
```

Sample maps of increasing difficulty live in `maps/`: `easy_linear.txt`,
`easy_fork.txt`, `medium_priority.txt`, `hard_maze.txt`. The map file
format itself is described inline through the examples; it follows the
grammar handed out with the subject (`nb_drones:`, `start_hub:`,
`end_hub:`, `hub:`, `connection:`, `#` comments, `[key=value ...]`
metadata).

## Resources

- Dijkstra's algorithm — used, with zone-type weights, for route search.
- Python `dataclasses`, `enum`, `heapq`, `argparse` standard library docs.
- Colleagues' independent takes on the same subject were read for ideas
  (not copied) before writing this version from scratch.

**AI usage:** AI was used as a research aid while preparing this
project — reading and summarizing the ~20-page subject and comparing
approaches taken by prior student implementations of the same
assignment, so the design below could be made independently and
deliberately rather than by trial and error. All parsing rules, the
routing/weighting scheme, the turn-resolution order, and the simulation
code itself were written and reasoned through explicitly, and every
design choice below can be explained and defended individually.

## Algorithm choices

**Parsing** (`mapfile.py`) is a hand-written, line-by-line reader: strip
comments, dispatch on the line's prefix, and raise a plain `ValueError`
from small per-field helpers. Those errors are only wrapped with their
line number once, at the top-level loop — so every error message reads
as `line N: <reason>` without repeating that formatting at every call
site.

**Routing** (`router.py`) is Dijkstra's algorithm, but the weight of
entering a zone is *not* simply its turn cost: `normal` and `restricted`
zones use their real cost (1 and 2), while `priority` zones are weighted
`0.9` even though they still take a single real turn. This makes the
search prefer priority zones as a tie-breaker without ever pretending
they are faster than they really are. `blocked` zones are excluded from
the search entirely.

To spread the fleet instead of sending every drone down the single
cheapest route, `discover_routes` runs Dijkstra repeatedly, adding a
small penalty to every zone used by a route already found — which
nudges each following search toward a different one. `plan_fleet` then
assigns drones one at a time to whichever discovered route currently has
the best `cost + queued // bottleneck_capacity` estimate, where the
bottleneck capacity is the narrowest zone or link capacity anywhere
along that route. This adapts to the map's shape: a linear map yields
one route and every drone takes it, while a map with several disjoint
paths spreads drones across them in rough proportion to how much traffic
each can actually absorb.

**Complexity:** each Dijkstra search is `O((V + E) log V)`; discovering
up to 6 routes is `O(6 (V + E) log V)`. All routes are computed once,
before the simulation starts, and simply replayed turn by turn — nothing
is recomputed mid-simulation. Memory is `O(V + E + drones)`.

**Simulation** (`simulate.py`) resolves one turn at a time in three
passes: drones finishing a restricted-zone crossing land first, other
in-flight drones tick down, then every waiting drone attempts to depart,
processed **furthest along its route first**. That ordering matters: a
drone about to vacate a zone is handled before a drone waiting to enter
that same zone, so a zone freed up this turn can be claimed the very
same turn instead of being reported as blocked for one turn too long.
Zone and link occupancy are tracked separately from the static network
(in the `Simulation` object, not on `Zone`/`Link` themselves), so the
same parsed `Network` could drive several independent simulation runs.
If no drone manages to move for 20 consecutive turns, a `DeadlockError`
is raised rather than looping forever.

## Visual representation

Move lines are printed with each zone name coloured according to its
`color=` metadata (ANSI true colour where a code exists, a deterministic
fallback colour otherwise, and a per-character cycling rainbow for
`color=rainbow`), so a terminal shows at a glance which zones a drone is
passing through turn by turn. Pass `--no-color` for plain text.

## Example

Input (`maps/easy_linear.txt`):

```
nb_drones: 2

start_hub: base 0 0 [color=green]
hub: gate 2 0 [color=blue]
hub: relay 4 0 [zone=priority color=cyan]
end_hub: dock 6 0 [color=yellow]

connection: base-gate
connection: gate-relay
connection: relay-dock
```

Output (`python3 main.py maps/easy_linear.txt --no-color`):

```
D1-gate
D1-relay D2-gate
D1-dock D2-relay
D2-dock
```
