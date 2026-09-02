# Fly-in — Technical Documentation

This document explains *everything*: the problem being solved, the theory
behind every algorithmic choice, and the code itself — function by
function, and in the non-trivial functions, line by line — starting from
the moment `main.py` is executed and ending at the last line printed to
the terminal.

It is a companion to `README.md`. The README tells you *what* the project
is and *how* to run it; this document tells you *why* it works and
*how*, mechanically, it gets there.

---

## Table of contents

1. [The problem, restated precisely](#1-the-problem-restated-precisely)
2. [Theory behind the design](#2-theory-behind-the-design)
3. [Module map](#3-module-map)
4. [Execution walkthrough](#4-execution-walkthrough)
   - 4.1 [`main.py` — entry point](#41-mainpy--entry-point)
   - 4.2 [`mapfile.py` — turning text into a graph](#42-mapfilepy--turning-text-into-a-graph)
   - 4.3 [`zones.py` — the domain vocabulary](#43-zonespy--the-domain-vocabulary)
   - 4.4 [`network.py` — the static graph](#44-networkpy--the-static-graph)
   - 4.5 [`router.py` — planning routes](#45-routerpy--planning-routes)
   - 4.6 [`simulate.py` — playing the routes forward](#46-simulatepy--playing-the-routes-forward)
   - 4.7 [`display.py` — colouring the output](#47-displaypy--colouring-the-output)
5. [A complete worked trace](#5-a-complete-worked-trace)
6. [Design decisions and trade-offs](#6-design-decisions-and-trade-offs)
7. [Complexity and memory](#7-complexity-and-memory)

---

## 1. The problem, restated precisely

You are given:

- A set of **zones** (nodes), each with a **kind** (`normal`, `priority`,
  `restricted`, `blocked`) and a **capacity** (how many drones may occupy
  it at once).
- A set of **connections** (edges) between zones, each with a **link
  capacity** (how many drones may traverse it at once).
- A number of **drones**, all starting in one shared `start_hub` zone,
  all needing to reach one shared `end_hub` zone.

You must produce a **turn-by-turn schedule**: at every discrete turn,
every drone either moves to an adjacent zone, begins a two-turn crossing
into a `restricted` zone, or waits — such that no zone or connection ever
exceeds its capacity, and every drone eventually arrives. The objective
is to minimise the total number of turns until the *last* drone arrives.

This is two problems stacked on top of each other:

1. A **graph search problem** — for a single drone, ignoring everyone
   else, what is a good route from start to end?
2. A **multi-agent scheduling problem** — given many drones each with a
   route, how do you interleave their movement, turn by turn, without
   ever violating a shared resource's capacity?

The project's four core modules map directly onto these two problems:
`router.py` solves (1), `simulate.py` solves (2).

---

## 2. Theory behind the design

### 2.1 Zones and links as a weighted graph

The network is a classic **undirected weighted graph**: zones are
vertices, connections are edges. What makes it slightly unusual is that
the "weight" of an edge doesn't belong to the edge — it belongs to the
**zone being entered**. Flying from A to B costs 1 turn if B is `normal`
or `priority`, 2 turns if B is `restricted`, and is simply illegal if B
is `blocked`. This is why every weight lookup in the code is keyed by
the *destination* zone's kind, never by the link itself (see
`zones.MOVE_TURNS` and every place it is indexed with `net.zone(name).kind`,
not with a link).

### 2.2 Dijkstra's algorithm, and why the weights are fractional

**Dijkstra's algorithm** finds the cheapest path from a source to every
other vertex in a graph with non-negative edge weights, by repeatedly
expanding the not-yet-settled vertex with the smallest known distance,
using a min-priority-queue (a binary heap) to make "smallest known
distance" a cheap operation. Its complexity is `O((V + E) log V)` with a
binary heap, where `V` is the number of zones and `E` the number of
connections.

If the weights were simply the real turn costs (`1` for normal/priority,
`2` for restricted), the search would have no reason to ever prefer a
`priority` zone over a `normal` one — they cost the same. But the
subject explicitly asks that priority zones be *preferred* by the
routing algorithm, not merely tolerated. The project resolves this with
a **two-tier cost model**:

- `zones.MOVE_TURNS` — the *real* number of simulation turns a move
  costs. This is what the simulation engine actually uses to decide how
  long a drone spends crossing into a zone.
- `zones.SEARCH_WEIGHT` — a *fictional*, purely comparative weight fed
  only to Dijkstra, where `priority` is `0.9` instead of `1.0`.

A route through a priority zone is therefore never reported as taking
fewer real turns than it does — `route_turn_cost` (§4.5) sums
`MOVE_TURNS`, not `SEARCH_WEIGHT` — but when two routes are otherwise
tied, Dijkstra's search will settle the priority-zone route first,
because its accumulated fractional cost is marginally lower. This is a
standard technique in weighted search: **use a tie-breaking heuristic
that never violates admissibility** (never makes something look cheaper
than it truly is) but still steers the search toward a preferred class
of solution.

### 2.3 Why one route is not enough: k-route diversity

Dijkstra gives you *one* cheapest route. If every drone were sent down
that single route, every zone and link along it would immediately become
the bottleneck, and drones would queue up single-file even if the map
offers three other perfectly good ways around. The project addresses
this with a simple, well-known trick: **run Dijkstra repeatedly, and
after each run, add a penalty to every zone the discovered route used**
(`router.discover_routes`, §4.5). The next search still finds the
cheapest path *overall*, but zones that are already "spoken for" by a
previous route now look artificially more expensive, so the search
tends to find a genuinely different route instead of the same one again.
This is a lightweight approximation of formal **k-shortest-paths**
algorithms (such as Yen's algorithm), traded for simplicity: it is not
guaranteed to find the true k best *distinct* paths, but it reliably
finds several *usably different* ones, which is all the fleet-assignment
step needs.

### 2.4 Bottleneck-aware load balancing

Once several candidate routes exist, drones must be assigned to them.
The project scores each route by `expected_turns = base_cost + queued // bottleneck_capacity`
(`router.plan_fleet`, §4.5), where `bottleneck_capacity` is the
**narrowest** zone or link capacity anywhere along that route — the
classic observation that a chain's throughput is limited by its weakest
link, borrowed directly from **network flow theory** (specifically, the
max-flow / min-cut intuition that the *bottleneck edge* determines
throughput). A route with a wide, high-capacity bottleneck absorbs many
drones before its estimated completion time gets worse; a route with a
narrow bottleneck saturates quickly and stops being chosen. This is a
**greedy load-balancing heuristic**: assign the next unit of work to
whichever resource currently has the smallest expected completion time.
It doesn't guarantee a globally optimal turn count (that would require
solving a much harder scheduling problem), but it adapts automatically
to whatever shape the map happens to have — a linear map naturally
collapses to one route carrying every drone, a map with several disjoint
paths naturally spreads the fleet across them roughly in proportion to
how much traffic each can bear.

### 2.5 Turn-based multi-agent movement, and the "false blocking" problem

Once every drone has a fixed route, the simulation has to decide, turn by
turn, who actually gets to move. The subject's rule that "*drones moving
out of a zone free up capacity for that same turn*" means the order in
which drones are considered *within* a single turn matters. Consider two
drones sharing a one-capacity corridor: drone A is about to leave zone X
for zone Y, and drone B, right behind it, wants to enter zone X. If B is
checked before A vacates X, the (correct) answer is "X is full, wait" —
even though, by the end of that same turn, X will in fact be empty. This
is the classic **false-blocking problem** in turn-based multi-agent
movement.

The fix used here — process drones **furthest along their route first**
(`simulate.Simulation._advance_turn`, §4.6) — is a simple **topological
ordering heuristic**: a drone closer to the goal is, by construction,
never waiting on a drone behind it (routes only move forward), so
resolving it first and letting its departure immediately update the
occupancy counters means the drone behind it sees the *freshly vacated*
zone in the very same turn. This turns a would-be one-turn stall into a
same-turn hand-off, exactly matching the subject's capacity-freeing rule
without needing a more complex simultaneous-conflict solver.

### 2.6 Modelling the two-turn "restricted" crossing

A `restricted` zone costs 2 turns to enter, and the subject is explicit
that a drone **cannot wait mid-crossing** — once it commits to the
crossing, it is guaranteed to land exactly one turn later. This means
the destination's capacity must be **reserved at the moment of
departure**, not rechecked on arrival (there would be nothing meaningful
to check — the drone is already committed). The code implements this by
incrementing the destination zone's occupancy counter as soon as the
drone departs (`_try_depart`, §4.6) and never touching that counter again
on arrival (`_land`, §4.6) — the "seat" was already claimed two turns
earlier. The link, in contrast, really is occupied for the whole
crossing, so its counter is only released on arrival. This is a small
instance of a general pattern in concurrent systems: **reserve a
resource for the whole duration of a multi-step operation, not just at
its final step**, otherwise a second drone could be let into an
already-promised zone one turn early.

### 2.7 Deadlock

A **deadlock** in this context is a set of drones each waiting on a zone
another one of them currently occupies, in a cycle, so that nobody ever
moves again. Because every drone's route was computed by shortest-path
search toward one shared goal, routes overwhelmingly flow in a
consistent direction (from start to end), so genuine cycles are rare —
but the graph itself may contain cycles, so it is not *impossible*.
Rather than trying to *prove* deadlock-freedom (which would require a
much heavier scheme, such as pre-booking every drone's full space-time
schedule before simulating at all), the engine takes the pragmatic
route: if an entire turn passes with **zero** drones moving, that's
counted as a stall; after `_STALL_LIMIT` (20) consecutive stalled turns,
`DeadlockError` is raised (`Simulation.run`, §4.6). Twenty turns is a
generous margin — legitimate congestion resolves in a handful of turns
as zones empty out — so reaching the limit reliably indicates a genuine
structural deadlock rather than ordinary queuing.

---

## 3. Module map

```
main.py         CLI: read args, wire everything together, print output
   |
   |-- mapfile.py    text file  ->  Network + drone_count
   |       |
   |       '-- zones.py      Zone / Link / ZoneKind / cost tables
   |       '-- network.py    Network (adjacency-list graph)
   |
   |-- router.py     Network + drone_count  ->  one route per drone
   |       '-- network.py, zones.py
   |
   |-- simulate.py   Network + routes  ->  list of per-turn moves
   |       '-- network.py, zones.py
   |
   '-- display.py    move strings  ->  ANSI-coloured move strings
```

Dependencies only ever point "down" this diagram — `zones.py` and
`network.py` depend on nothing else in the project, and nothing is
imported in a cycle. `simulate.py` and `router.py` both depend on
`network.py`/`zones.py` but never on each other; `main.py` is the only
module that knows about all of them.

---

## 4. Execution walkthrough

### 4.1 `main.py` — entry point

```python
if __name__ == "__main__":
    sys.exit(main())
```

This is the very last line of the file and the true entry point: running
`python3 main.py maps/easy_linear.txt` executes this guard, calls
`main()`, and passes whatever integer it returns to `sys.exit`, which
becomes the process's exit code (`0` = success, `1` = a handled error —
this is what lets `make lint`/CI/a grading script detect failure without
scraping output text).

**`main(argv: list[str] | None = None) -> int`** (lines 36–63) is the
orchestrator. Walking through it:

- **Lines 38–43** build the CLI with `argparse`: one required positional
  argument (`map_file`) and one optional flag (`--no-color`). `args =
  parser.parse_args(argv)` — note `argv` defaults to `None`, which tells
  `argparse` to read `sys.argv` itself; accepting it as a parameter
  instead of hardcoding that is what lets `main()` be called
  programmatically (e.g. from a test) with an explicit argument list.
- **Lines 45–54**, the `try`/`except`: this is the single place in the
  whole program that turns an exception into a clean, user-facing error
  and a non-zero exit code, instead of letting a raw Python traceback
  reach the terminal (the "handle exceptions gracefully" requirement).
  - `net, drone_count = read_map(args.map_file)` — hands the file path
    to the parser (§4.2) and gets back a fully built `Network` plus the
    declared drone count. `open()` inside `read_map` is used as a
    context manager, so the file handle is closed automatically even if
    parsing raises — this is also why the `except FileNotFoundError`
    clause needs no cleanup of its own.
  - `drones = _build_drones(net, drone_count)` — see below.
  - `turns = Simulation(net, drones).run()` — builds the simulation
    engine (§4.6) around the network and the freshly built drones, and
    runs it to completion, producing the full per-turn move log.
  - `except FileNotFoundError` (line 49): raised by the `open()` call
    inside `read_map` if the path doesn't exist; reported distinctly
    from a malformed map, because "wrong path" and "bad file contents"
    are different mistakes with different fixes.
  - `except (MapError, RouteError, DeadlockError) as exc` (line 52):
    the three "expected" failure modes — bad file syntax, an
    unreachable end zone, and a genuine scheduling deadlock — are all
    handled identically: print `Error: {exc}` to **stderr** (so stdout,
    which is meant to carry only the turn-by-turn move lines, stays
    clean even on failure) and return `1`.
- **Lines 56–57**: on success, iterate the per-turn move lists and print
  one formatted line per turn via `_format_turn` (below) — this is the
  program's primary, required output.
- **Lines 59–62**: a short human-readable summary line
  (`Delivered N drone(s) in T turn(s).`) printed to **stderr**, not
  stdout — deliberately kept out of the channel a grader might parse for
  the exact move-line format, while still being visible to a human
  running the program directly in a terminal.
- **Line 63**: `return 0` — success.

**`_build_drones(net: Network, drone_count: int) -> list[Drone]`**
(lines 15–19): calls `router.plan_fleet` (§4.5) to get one route (a list
of zone names) per drone, then builds a `Drone` object (§4.6) for each
with a 1-based id — `Drone(drone_id=i + 1, route=route) for i, route in
enumerate(routes)`. The `+ 1` exists purely so the printed drone tags
read `D1`, `D2`, ... instead of `D0`, `D1`, ... matching the subject's
example output.

**`_format_turn(net: Network, moves: list[str], use_color: bool) -> str`**
(lines 22–33): takes one turn's raw move strings — each already in the
engine's `D<id>-<zone>` or `D<id>-<from>-<to>` form — and, if colour is
enabled, re-paints the zone names inside them.

- `if not use_color: return " ".join(moves)` — the plain-text fast path,
  used by `--no-color` and exercised in every automated check, since it
  reproduces the exact format described in the subject with no ANSI
  noise.
- `drone_tag, _, zone_names = move.partition("-")` — splits off the
  leading `D<id>` from everything after the *first* hyphen. This relies
  on a guarantee enforced back in the parser (§4.2): zone names can
  never themselves contain a hyphen, so the first hyphen in any move
  string is unambiguously the separator right after the drone tag.
- `zone_names.split("-")` then yields either one zone name (a plain
  arrival) or two (a `from-to` restricted-zone crossing), each of which
  is passed through `colorize(name, net.zone(name).color)` (§4.7) and
  rejoined with `"-".join(...)`.
- The final `f"{drone_tag}-{painted}"` reassembles the move string with
  the same structural shape it had before, just with embedded ANSI
  escapes around each zone name.

### 4.2 `mapfile.py` — turning text into a graph

This module's whole job is: read a text file, and either return a
correctly built `Network` plus a drone count, or raise `MapError` with
the exact line number and reason something didn't parse. Its structure
follows one deliberate rule: **helper functions raise plain `ValueError`
with no line-number information at all**; only the single top-level loop
in `read_map` knows what line it is currently on, and it is the *only*
place that wraps a `ValueError` into a `MapError`. This keeps every
low-level parsing helper simple, reusable, and independently testable —
none of them need to know or care what line they were called from.

**`MapError.__init__(self, line_number: int, reason: str) -> None`**
(lines 22–23): a tiny custom exception whose constructor immediately
formats the final message (`f"line {line_number}: {reason}"`) and hands
it to `Exception.__init__` via `super().__init__(...)`, so printing the
exception (`str(exc)` or an f-string like `f"Error: {exc}"` in
`main.py`) automatically yields the fully formatted line.

**`read_map(path: str) -> tuple[Network, int]`** (lines 26–62) — the
main parsing loop:

- **Lines 28–34** set up the accumulators: `zones` (name → `Zone`),
  `links` (list of `Link`), `seen_links` (a set of normalised
  `(zone_a, zone_b)` pairs used to reject duplicate connections),
  `start`/`end` (the two special zone names, both starting `None`),
  `drone_count` (also starts `None` — this doubles as the "have we seen
  the `nb_drones:` line yet?" flag), and `line_number = 0`, initialised
  outside the loop purely so the "missing declaration" error messages
  after the loop (lines 53–58) have a sane value (`0`, or the last line
  actually read) even if the file were completely empty.
- **Line 36**: `with open(path, encoding="utf-8") as handle:` — a
  context manager, so the file is guaranteed to be closed whether
  parsing succeeds, fails, or the whole function is abandoned via an
  exception; this satisfies the "use context managers for resources"
  requirement directly.
- **Line 37**: `for line_number, raw_line in enumerate(handle, start=1):`
  — iterating a file object yields its lines one at a time (no need to
  read the whole file into memory first); `enumerate(..., start=1)`
  pairs each line with its 1-based line number, matching how humans
  count lines in a text editor.
- **Line 38**: `line = raw_line.split("#", 1)[0].strip()` — comment
  stripping and whitespace trimming in one expression. Splitting on
  `"#"` with `maxsplit=1` and keeping only `[0]` discards everything
  from the first `#` onward (a `#` can never appear before it in valid
  map syntax, so a single split is sufficient); `.strip()` then removes
  leading/trailing whitespace and, incidentally, the trailing newline
  every line carries.
- **Lines 39–40**: `if not line: continue` — after comment-stripping, a
  line that is now empty was either blank to begin with or was *only* a
  comment; both are silently skipped, exactly as the subject specifies.
- **Lines 41–51**, the dispatch `try` block — for each remaining
  meaningful line, exactly one of four things happens, and any
  `ValueError` raised while doing it is caught once and re-raised as a
  `MapError` carrying the current `line_number`:
  - `if drone_count is None:` (line 42) — until the drone count has been
    seen, *every* line is interpreted as the `nb_drones:` declaration by
    calling `_parse_drone_count(line)`. This is what enforces "the first
    line must define the number of drones": if the very first
    meaningful line isn't `nb_drones: ...`, `_parse_drone_count` itself
    raises immediately (see below).
  - `elif line.startswith(_ZONE_PREFIXES):` (line 44) — once the drone
    count is known, a line starting with `start_hub:`, `end_hub:` or
    `hub:` (the tuple `_ZONE_PREFIXES` defined at line 16) is handed to
    `_ingest_zone`.
  - `elif line.startswith("connection:"):` (line 46) — handed to
    `_ingest_link`.
  - `else: raise ValueError(...)` (line 48–49) — anything else is a
    syntax error with no recognisable prefix at all.
- **Lines 53–58**, post-loop validation — three checks that can only be
  made *after* the whole file has been read: no `nb_drones:` line was
  ever found; no zone was ever tagged as `start_hub`; no zone was ever
  tagged as `end_hub`. Each raises `MapError` directly (not via a
  `ValueError`, since there is no "current line" to catch it against —
  these are file-level, not line-level, problems).
- **Lines 60–61**: `zones[start].capacity = None` and
  `zones[end].capacity = None` — this is where the subject's rule "the
  `max_drones` capacity is ignored on the `start_hub`/`end_hub` zones"
  is actually enforced. Note that this happens *after* parsing, not
  during it: `_parse_zone` (below) validates and stores whatever
  `max_drones` value was written, and only now is it unconditionally
  overwritten with `None` (meaning "unlimited", per `Zone.has_room`,
  §4.3) — matching the subject's instruction that such metadata is
  "ignored" rather than rejected.
- **Line 62**: constructs and returns the `Network` (§4.4) together with
  `drone_count`.

**`_ingest_zone(line, zones, start, end) -> tuple[str | None, str | None]`**
(lines 65–83): parses one zone line via `_parse_zone`, checks it isn't a
duplicate name, stores it in the `zones` dict, and — if the line was a
`start_hub:`/`end_hub:` — updates and returns the (possibly still
`None`) `start`/`end` values, raising if one was already set (catching
"a second start_hub was found"). Returning the pair rather than mutating
in place keeps the function free of hidden side effects on its caller's
local variables — Python has no output parameters, so this is the
idiomatic way to "update two values" from a helper.

**`_ingest_link(line, zones, seen_links) -> Link`** (lines 86–95):
parses one connection line via `_parse_link`, then checks
`link.key()` (a direction-independent, alphabetically sorted tuple —
see `Link.key`, §4.3) against `seen_links` to catch both `a-b` and
`b-a` as the same duplicate connection, before adding the new key to the
set and returning the `Link`.

**`_parse_drone_count(line: str) -> int`** (lines 98–101): checks the
line literally starts with `nb_drones:`, then delegates the actual
number parsing to `_parse_positive_int` on whatever follows the colon.

**`_parse_zone(line: str) -> Zone`** (lines 104–135) — the busiest
parsing function:

- **Line 105**: `prefix, rest = line.split(":", 1)` — splits off the
  directive keyword (`hub`, `start_hub`, `end_hub`) from everything
  after the *first* colon (`maxsplit=1`, so a colour value or anything
  else could theoretically contain a colon without breaking this split
  — though none of the current metadata values do). For example,
  `"start_hub: a 0 0".split(":", 1)` gives `["start_hub", " a 0 0"]`.
- **Line 106**: `body, meta = _split_metadata(rest)` — pulls the
  optional `[key=value ...]` block off the end, if present (see below).
- **Line 107**: `fields = body.split()` — splits the remaining
  whitespace-separated positional part into tokens.
- **Lines 108–109**: exactly 3 tokens are required (`name x y`); any
  other count raises a clear "expected `<prefix>: <name> <x> <y>
  [metadata]`" error that echoes the correct format back at the user.
- **Lines 111–112**: rejects a name containing `-`, since `-` is the
  connection-line delimiter and an ambiguous name would make
  `connection: a-b` unparseable later.
- **Line 114**: `values = _parse_key_values(meta, _ZONE_KEYS)` — parses
  the metadata block into a plain `dict[str, str]`, rejecting any key
  outside `_ZONE_KEYS = {"zone", "color", "max_drones"}` (line 14).
- **Lines 115–120**: the zone type. Defaults to `ZoneKind.NORMAL`; if a
  `zone=` key was present, `ZoneKind(values["zone"])` attempts to
  construct the enum member from that string — `Enum` raises its own
  `ValueError` if the string doesn't match any member's value, which is
  caught and re-raised with a clearer message naming the offending
  value.
- **Lines 121–123**: colour is optional (`values.get("color")` returns
  `None` if absent); if present, it's checked with `.isalnum()` —
  deliberately the loosest possible validation, because the subject
  explicitly says colour accepts "any valid single-word string" with no
  fixed palette, so there is nothing more specific to validate against
  (see also §2, `display.py`, §4.7, for how an *unrecognised* colour
  name is still rendered sensibly).
- **Lines 124–126**: `max_drones` defaults to `1`; if present, it must
  parse as a positive integer via `_parse_positive_int`. As noted above,
  this value is later silently discarded for the start/end zones
  specifically (`read_map`, lines 60–61) — it is still validated here
  because the subject requires any *present* metadata to be
  syntactically valid, even where its value ends up unused.
- **Lines 128–135**: assembles and returns the `Zone`.

**`_parse_link(line: str, zones: dict[str, Zone]) -> Link`**
(lines 138–157): the connection-line counterpart of `_parse_zone`.

- **Lines 139–140**: same colon-split and metadata-extraction pattern as
  `_parse_zone`.
- **Line 141**: `endpoints = [part.strip() for part in body.split("-")]`
  — splits the remaining body on every `-`. Because zone names are
  guaranteed dash-free (enforced above), a syntactically valid
  `<nameA>-<nameB>` always splits into exactly two non-empty pieces;
  **line 142** (`if len(endpoints) != 2 or not all(endpoints)`) is what
  catches any other shape (missing dash, empty side, stray extra dash)
  and reports it uniformly.
- **Lines 145–147**: both endpoint names must already exist in the
  `zones` dict built so far — this is what enforces "connections must
  link only previously defined zones": since zones are only ever added
  to the dict earlier in the same top-to-bottom pass, a connection
  referencing a zone defined *later* in the file fails here exactly as
  one referencing a zone that's misspelled or never defined at all.
- **Line 148**: rejects a self-loop (`a-a`).
- **Lines 151–156**: metadata parsing for `_LINK_KEYS = {"max_link_capacity"}`
  (line 15), identical pattern to the zone case, defaulting to `1`.
- **Line 157**: builds and returns the `Link`.

**`_split_metadata(rest: str) -> tuple[str, str | None]`** (lines
160–167): given everything after the first colon, separates the
positional part from the bracketed metadata part.

- If there's no `[` at all, the whole (stripped) string is the
  positional part and metadata is `None` — a directive with no metadata
  block at all is valid.
- If there is a `[` but the string doesn't end in `]` (line 164), that's
  a malformed metadata block (e.g. a missing closing bracket).
- Otherwise, `rest.partition("[")` (line 166) splits into
  `(everything before the first '[', the '[' itself, everything after)`;
  the body is the trimmed first part, and the metadata is the trimmed
  third part with its own trailing `]` sliced off (`meta[:-1]`).

**`_parse_key_values(meta, allowed) -> dict[str, str]`** (lines
170–183): turns a metadata body like `"zone=restricted color=red"` into
`{"zone": "restricted", "color": "red"}`.

- `if not meta: return {}` — no metadata block at all is valid and
  simply yields an empty dict, letting every caller use `.get(...)`
  defaults uniformly.
- For each whitespace-separated `token`: it must contain `=` (line 175);
  `token.partition("=")` splits it into `key`, the `=` itself (discarded
  via `_`), and `value`; the key must be one of the caller-supplied
  `allowed` set (line 178, this is what makes an unrecognised key like
  `[foo=bar]` on a zone line an error rather than silently ignored); and
  a key seen twice in the same block is rejected (line 180) rather than
  silently letting the second occurrence win.

**`_parse_int` / `_parse_positive_int`** (lines 186–197): the last two
leaf helpers. `_parse_int` wraps Python's `int(text)` and turns its
`ValueError` into one carrying the field's name (`label`) for a clearer
message; `_parse_positive_int` calls it and additionally rejects zero or
negative values. Every numeric field in the file — `nb_drones`,
zone `x`/`y`, `max_drones`, `max_link_capacity` — ultimately funnels
through one of these two, so the "must be an integer" / "must be
positive" wording is consistent everywhere.

### 4.3 `zones.py` — the domain vocabulary

This module defines the vocabulary every other module speaks, and holds
no logic beyond two tiny methods.

**`ZoneKind(Enum)`** (lines 9–15): the four zone categories, using
`Enum` rather than raw strings so that a typo like `"restircted"` fails
fast and loudly the moment `ZoneKind("restircted")` is attempted in
`mapfile._parse_zone`, instead of silently behaving like an unrecognised
(and therefore mis-handled) string everywhere it's later compared.

**`MOVE_TURNS`** (lines 20–24) and **`SEARCH_WEIGHT`** (lines 29–33):
the two cost tables discussed in depth in §2.2. Both are plain module
level `dict[ZoneKind, ...]` constants — deliberately *not* methods on
`ZoneKind` itself, because attaching "how expensive is it to fly here"
to the enum would blur a pure vocabulary type with two different,
independently-evolvable policies (real cost vs. search bias).

**`Zone`** (lines 36–49): a `@dataclass` — Python generates `__init__`,
`__repr__` and `__eq__` automatically from the field list, which is all
a plain data holder like this needs. Fields: `name`, `x`, `y` (no
defaults — always required), then `kind` (defaults to `NORMAL`), `color`
(defaults to `None`, meaning "no colour specified"), and `capacity`
(defaults to `1`, matching the subject's "a zone may contain at most one
drone by default"; `None` is used to mean *unlimited*, which is what
`mapfile.read_map` sets for the start/end zones after parsing).

- **`has_room(self, occupied: int) -> bool`** (lines 47–49): the single
  piece of real logic in this class — `True` if the zone has unlimited
  capacity (`capacity is None`) or if the current occupancy is still
  below the limit. This is called from `simulate._try_depart` every time
  a drone wants to enter this zone.

**`Link`** (lines 52–71): another dataclass — `zone_a`, `zone_b`
(required) and `capacity` (defaults to `1`).

- **`other_end(self, zone_name: str) -> str`** (lines 60–66): given one
  of the two zones this link touches, returns the other one; raises if
  handed a zone that isn't actually one of its two endpoints (a
  programming-error guard — this should be unreachable given how the
  rest of the code calls it, but it turns a silent wrong answer into a
  loud, immediate failure if it's ever miscalled).
- **`key(self) -> tuple[str, str]`** (lines 68–71): `sorted((self.zone_a,
  self.zone_b))` alphabetically orders the two endpoint names. Because a
  connection between A and B is the same connection regardless of which
  order its endpoints were declared in, using this sorted pair as a
  dictionary key (in `mapfile`'s `seen_links` and `simulate`'s
  `_link_load`) means "A-B" and "B-A" are automatically treated as
  identical, with no separate normalisation step needed at every call
  site.

### 4.4 `network.py` — the static graph

**`Network`** (lines 10–45): also a `@dataclass`, holding the whole
parsed map — `zones` (name → `Zone`), `links` (list of `Link`), `start`
and `end` (the two special zone names) — plus one private,
non-constructor field: `_adjacency`.

- **`_adjacency: dict[str, list[Link]] = field(init=False, repr=False)`**
  (line 24): `dataclasses.field(init=False, ...)` excludes this
  attribute from the generated `__init__` — callers construct a
  `Network` with only `zones`, `links`, `start`, `end`; the adjacency
  list is *derived* data, not something a caller should ever be asked to
  supply directly. `repr=False` keeps it out of the auto-generated
  `__repr__` too, so printing a `Network` for debugging doesn't dump a
  huge adjacency structure.
- **`__post_init__(self) -> None`** (lines 26–30): a hook dataclasses
  call automatically right after the generated `__init__` finishes.
  Here it builds the adjacency list once, up front: an empty list per
  zone name, then one append per link into *both* of its endpoints'
  lists (since links are undirected). Doing this once at construction
  time — rather than scanning the full `links` list every time a
  neighbour lookup is needed — is what makes `links_from` an O(1)
  dictionary lookup instead of an O(E) scan, which matters because
  Dijkstra calls it once per settled vertex.
- **`zone(self, name)` / `links_from(self, zone_name)` /
  `link_between(self, a, b)`** (lines 32–45): three small, deliberately
  boring accessor methods — a name lookup, an adjacency lookup, and (for
  `link_between`) a short linear scan over just the links touching `a`
  (never more than a handful in any realistic map) to find the one whose
  other end is `b`. Keeping these as named methods rather than having
  every caller reach into `net.zones[...]` or `net._adjacency[...]`
  directly is what lets `_adjacency` stay a private implementation
  detail — nothing outside this file ever touches it.

### 4.5 `router.py` — planning routes

**`shortest_path(net, start, end, penalty=None) -> list[str] | None`**
(lines 25–68) — Dijkstra's algorithm, textbook structure with one
addition (the `penalty` parameter from §2.3):

- **Line 37**: `penalty = penalty or {}` — normalises "no penalty
  supplied" (`None`, the default) and "an empty penalty dict" to the
  same thing, so the lookup at line 55 never needs a `None` check.
- **Lines 38–41**: the three structures every Dijkstra implementation
  needs — `best` (the cheapest known cost to reach each zone so far,
  seeded with the source at cost `0.0`), `previous` (for reconstructing
  the actual path once the search finishes, not just its cost), and
  `frontier`, a **binary min-heap** (`heapq`) of `(cost, zone_name)`
  pairs, seeded with the source. `settled` tracks zones whose true
  shortest cost is already known and finalised.
- **Lines 43–60**, the main loop — repeat until the heap is empty:
  - `heapq.heappop(frontier)` (line 44) always returns the pair with the
    smallest `cost` in `O(log n)`, which is precisely what makes Dijkstra
    greedy-correct: the first time any zone is popped, no cheaper route
    to it can possibly exist, because everything still in the heap costs
    at least as much.
  - **Lines 45–46**: a zone can be pushed onto the heap multiple times
    (once per edge that currently looks like it might improve its cost)
    — this implementation doesn't bother removing stale heap entries
    when a cheaper one is found (a technique sometimes called "lazy
    deletion"), and instead just skips a pop if that zone was already
    settled by an earlier, cheaper pop. This trades a little extra heap
    memory for much simpler code than maintaining a decrease-key-capable
    heap.
  - **Line 48**: once the target `end` itself is popped, its shortest
    cost is finalised and the search can stop early — `break` — rather
    than continuing to explore the rest of the graph pointlessly.
  - **Lines 50–60**: relax every edge out of the current zone. `zone.kind
    is ZoneKind.BLOCKED: continue` (lines 52–54) is where blocked zones
    are excluded from the graph entirely — they are simply never
    considered as a valid neighbour to expand into, which automatically
    makes any route requiring one unreachable. `weight = SEARCH_WEIGHT[zone.kind]
    + penalty.get(neighbour, 0.0)` (line 55) combines the zone-kind
    weight from §2.2 with any diversity penalty from §2.3. If the new
    total cost beats the best known cost for that neighbour so far
    (`new_cost < best.get(neighbour, math.inf)`, line 57 — `math.inf`
    as the default correctly treats "never seen before" as "infinitely
    expensive so far"), the neighbour's best cost and predecessor are
    updated and it's pushed onto the heap for future expansion.
- **Lines 62–68**, path reconstruction: if `end` was never reached at
  all (`end not in best`), return `None` — there is genuinely no path.
  Otherwise, walk backwards from `end` through `previous` until `start`
  is reached, building the list in reverse, then `path.reverse()` to
  present it start-to-end as every caller expects.

**`discover_routes(net, start, end, limit=6) -> list[list[str]]`**
(lines 71–86): the diversity loop from §2.3. Up to `limit` times, run
`shortest_path` with the current `penalty` dict; stop early
(`break`) either if no path exists any more or if the search returned a
route already in the list (meaning the penalties have stopped changing
what's found — further iterations would be wasted work); otherwise,
record the route and bump the penalty of every one of its *intermediate*
zones (`route[1:-1]` deliberately excludes the shared start and end
zones, since penalising those would pointlessly bias every future search
away from the one start and one end every route must use). If the loop
produced no routes at all, the start and end zones are simply
disconnected, and `RouteError` is raised — this is the one way `main.py`
learns "this map has no valid solution at all," as opposed to a
malformed file (`MapError`) or a scheduling failure (`DeadlockError`).

**`route_turn_cost(net, route) -> int`** (lines 89–91): `sum(MOVE_TURNS[net.zone(name).kind]
for name in route[1:])` — the *real* number of turns (not the fractional
search weight) a single unobstructed drone would spend flying this exact
route, summed over every zone *entered* (`route[1:]` skips the start
zone itself, since arriving in the start zone costs nothing — the drone
begins there). Used purely as the "base cost" half of the load-balancing
score in `plan_fleet`.

**`route_capacity(net, route) -> int | float`** (lines 94–103): the
bottleneck calculation from §2.4 — the minimum, across every
*intermediate* zone's capacity (`route[1:-1]`, again excluding start/end
since those are unlimited and therefore never the bottleneck) and every
link's capacity along the route, using Python's `zip(route, route[1:])`
idiom to iterate consecutive `(here, there)` pairs. Starts from
`math.inf` so that a route with, hypothetically, zero constraining
zones/links would report an unbounded bottleneck (in practice every
route has at least one link, whose default capacity is `1`, so this
starting value is always tightened by at least one real number).

**`plan_fleet(net, start, end, drone_count) -> list[list[str]]`**
(lines 106–123) — ties §2.3 and §2.4 together:

- **Line 110**: get the diverse candidate routes.
- **Lines 111–112**: precompute each route's base turn cost and
  bottleneck capacity *once*, up front — not recomputed on every drone
  assignment, since neither value changes as drones are assigned.
- **Line 113**: `load`, one running counter per route, starting at zero.
- **Lines 116–122**: assign drones one at a time. For each of
  `drone_count` drones, `min(range(len(routes)), key=lambda i:
  turn_cost[i] + load[i] // capacity[i])` picks the index of whichever
  route currently has the lowest `base_cost + (drones already queued on
  it) // (its bottleneck capacity)` — the greedy load-balancing score
  from §2.4. Integer floor division means the estimated queuing penalty
  only increases once a route has accumulated a full "wave" of drones
  up to its bottleneck capacity, matching the intuition that drones
  within one wave can flow through roughly together. That route is
  appended to `assignments` and its `load` counter incremented, so the
  *next* drone's decision sees the updated picture.
- **Line 123**: returns the full list of per-drone routes, in drone
  order — this is exactly what `main._build_drones` zips against
  `enumerate(...)` to build the actual `Drone` objects.

### 4.6 `simulate.py` — playing the routes forward

**`Drone`** (lines 25–39) — a `@dataclass` tracking one drone's live
state during simulation (as opposed to `router`'s output, which is just
a static route). `drone_id` and `route` are required; `step` (index into
`route` of the zone the drone currently occupies or is committed to),
`in_transit` (mid-crossing into a restricted zone), `turns_remaining`
(how many more turns until it lands) and `transit_link` (which link's
occupancy counter to release on landing) all default to "not moving yet."

- **`delivered` (a `@property`, lines 36–39)**: `not self.in_transit and
  self.step == len(self.route) - 1` — true exactly when the drone is
  sitting still at the last index of its route, i.e. at the `end` zone.
  Being a property rather than a plain field means it's always computed
  fresh from `step`/`in_transit` and can never silently go stale.

**`Simulation.__init__(self, net, drones) -> None`** (lines 45–53):
stores the network and drone list, then builds two occupancy trackers
that live entirely *outside* the static `Network`/`Zone`/`Link` objects
(the design choice discussed in §6): `_zone_load`, one counter per zone
*except* `start`/`end` (which are unlimited and therefore never need
tracking — this dict comprehension's `if name not in (net.start,
net.end)` guard is what makes every later `.get(name, 0)` on those two
zones safely fall back to `0` without a KeyError, since `has_room` never
even checks the counter for a `None`-capacity zone in the first place),
and `_link_load`, one counter per link keyed by its direction-independent
`link.key()`.

**`run(self) -> list[list[str]]`** (lines 55–72) — the main simulation
loop:

- **Line 62**: `turns`, the accumulator returned at the end — one
  `list[str]` of move descriptions per turn.
- **Line 63**: `stalled_for`, the deadlock counter from §2.7, starting
  at `0`.
- **Line 64**: `while not all(drone.delivered for drone in self.drones):`
  — keep simulating turns until every single drone reports `delivered`.
- **Line 65**: `moves = self._advance_turn()` — resolve exactly one
  turn (see below) and collect whatever moves happened during it.
- **Lines 67–71**: if `moves` is non-empty, progress was made this turn
  and the stall counter resets to `0`; otherwise it increments, and once
  it reaches `_STALL_LIMIT` (20), `DeadlockError` is raised, aborting the
  simulation instead of looping forever.
- **Line 72**: once the `while` loop's condition finally goes false
  (everyone delivered), return the complete `turns` log.

**`_advance_turn(self) -> list[str]`** (lines 74–95) — resolves one
turn in the three passes described in §2.5/§2.6:

- **Pass 1 — landings (lines 77–81)**: `landing` is every drone that is
  `in_transit` with exactly `1` turn remaining, i.e. *this* turn is the
  turn it arrives. These are sorted `key=lambda d: -d.step` (descending
  by route progress — the negative sign turns Python's default
  ascending sort into a descending one) and each is handed to
  `self._land(drone)`, whose returned move string is appended to
  `moves`. Landings are processed before anything else in the turn
  because — as explained in §2.6 — their destination zone's capacity was
  already reserved two turns ago; there is nothing left to check, only
  bookkeeping to finalise, so there's no reason to make anything else
  wait on it.
- **Pass 2 — ticking (lines 83–85)**: every *other* in-transit drone
  (one with more than one turn remaining — which, given `MOVE_TURNS`
  currently only ever produces a crossing of exactly 2 turns, only
  actually differs from "just departed" if a future map format ever
  introduced a longer crossing) has `turns_remaining` decremented by
  one. This loop deliberately does **not** collect anything into
  `moves` — a drone silently continuing a crossing it already announced
  when it departed is not a new event worth a line of output, only its
  departure and its eventual arrival are.
- **Pass 3 — departures (lines 87–94)**: `waiting` is every drone that
  is neither in transit nor already delivered — i.e. every drone that
  might want to make a move this turn. Sorted the same
  furthest-along-first way as landings (§2.5), each is offered a chance
  to move via `self._try_depart(drone)`; if it returns a move string
  (as opposed to `None`, meaning "no room, stay put"), that string is
  appended to `moves`.
- **Line 95**: return everything collected across all three passes —
  this is exactly one line of the program's required output format.

**`_try_depart(self, drone: Drone) -> str | None`** (lines 97–125) — the
heart of the capacity-checking logic, called once per waiting drone per
turn:

- **Lines 98–102**: `origin` and `destination` are read straight out of
  the drone's fixed `route` at its current `step` and the next index;
  `dest_zone` is looked up for its kind/capacity, `link` for the
  connection between them, and `link_key` is that link's
  direction-independent identifier (used to index `_link_load`).
- **Lines 104–107**, the two capacity gates — if *either* fails, the
  function returns `None` immediately and the drone simply waits this
  turn, with **no state mutated at all** (crucial: a rejected attempt
  must be a no-op, not a partial move):
  - `if not dest_zone.has_room(self._zone_load.get(destination, 0)):
    return None` — is the destination zone already at capacity?
  - `if self._link_load[link_key] >= link.capacity: return None` — is
    the connection itself already at capacity?
- **Lines 109–113**, now that both checks passed, the move is
  committed. `if destination not in (self.net.start, self.net.end):
  self._zone_load[destination] += 1` reserves a seat in the destination
  (skipped for start/end, which are never tracked — see `__init__`
  above); symmetrically, `if origin not in (self.net.start, self.net.end):
  self._zone_load[origin] -= 1` frees the seat the drone is vacating.
  This is the exact mechanism behind §2.5's same-turn hand-off: because
  drones are processed furthest-along-first, an earlier drone's `-= 1`
  here has already happened by the time a trailing drone's `has_room`
  check (above) runs for the same zone, later in the very same turn.
  `self._link_load[link_key] += 1` marks the connection as (at least
  momentarily) in use.
- **Lines 115–116**: `turns = MOVE_TURNS[dest_zone.kind]` looks up the
  *real* cost of entering this zone kind (§2.2); `drone.step += 1`
  advances the drone's position pointer to the destination immediately,
  regardless of whether the move takes one turn or two — the drone is
  now considered committed to being there, which is exactly why
  `Drone.delivered` and the furthest-along sort both treat an in-transit
  drone as already "at" its destination.
- **Lines 118–120**, the one-turn case (`normal`/`priority`): the link
  was only ever needed for the instant of this single turn, so its
  reservation is immediately released again (`self._link_load[link_key]
  -= 1`) — this is what makes it possible for a second, lower-priority
  drone to use the *same* link later in this *same* turn if capacity
  allows (§2.6's "reserve for the whole duration" principle, applied
  here in its simplest form: the whole duration is *this instant*). The
  function returns the plain arrival string `D<id>-<destination>`.
- **Lines 122–125**, the two-turn case (`restricted`): the link
  reservation is *not* released — it stays held until `_land` runs, two
  turns from now. `drone.in_transit = True`, `drone.turns_remaining =
  turns - 1` (i.e. `1`, since `MOVE_TURNS[RESTRICTED] == 2` and the
  turn spent departing already counts as the first of the two), and
  `drone.transit_link = link_key` records which link's counter `_land`
  must eventually release. The function returns the crossing-format
  string `D<id>-<origin>-<destination>`.

**`_land(self, drone: Drone) -> str`** (lines 127–133): called only from
pass 1 of `_advance_turn`, only for drones known to have exactly one
turn remaining. `assert drone.transit_link is not None` documents (and,
under `python -O`-free normal execution, actively checks) an invariant
that should be structurally guaranteed by `_try_depart` always setting
`transit_link` before setting `in_transit = True` — if this assertion
ever fired, it would indicate a genuine bug elsewhere, not a bad map
file. The link's occupancy is released (`self._link_load[drone
.transit_link] -= 1` — the destination zone's occupancy is deliberately
*not* touched here, per §2.6, since it was already reserved at
departure), the drone's transit flags are cleared, and the arrival
string `D<id>-<zone the drone is now at>` is returned — `drone.route[drone.step]`,
since `step` was already advanced to the destination index back when
the crossing began.

### 4.7 `display.py` — colouring the output

**`colorize(text: str, color: str | None) -> str`** (lines 28–35) — the
only function this module exposes to the rest of the program:

- `if color is None: return text` — a zone with no `color=` metadata is
  printed completely plain, no escape codes at all.
- `if color.lower() == "rainbow": return _rainbow(text)` — the special
  cycling-colour case (§ below).
- Otherwise, `_NAMED_COLORS.get(color.lower(), _fallback_code(color))`
  looks the (lower-cased, so `Red`/`RED`/`red` all match) colour name up
  in the small hand-written palette (lines 7–22, covering the common
  CSS-ish colour words that show up across real map files); if it's not
  one of those, `_fallback_code` is used instead of failing — remember,
  the subject explicitly allows *any* single-word colour string, so an
  unrecognised name must still render as *something*, not error out.
  Either way, the text is wrapped `f"\033[{code}m{text}{_RESET}"` — a
  standard ANSI SGR (Select Graphic Rendition) escape sequence: `\033[`
  starts it, `code` selects the colour (`"31"` for basic red, or
  `"38;5;208"` for a 256-colour-palette entry), `m` ends the code, then
  the text, then `_RESET` (`\033[0m`) turns formatting back off so it
  doesn't bleed into whatever prints next.

**`_rainbow(text: str) -> str`** (lines 38–42): builds the string one
character at a time, cycling through `_RAINBOW`'s six ANSI colour codes
(`i % len(_RAINBOW)` wraps the index back to `0` once it runs past the
end of the list) so a name like `"goal"` comes out with each letter in a
different colour — matching the `color=rainbow` special value that
appears across the example maps referenced in the subject's own
materials.

**`_fallback_code(name: str) -> str`** (lines 45–48): rather than using
Python's built-in `hash()` (which is randomised per-process by default
for security reasons, meaning the same colour name could map to a
different code on every run of the program), the index into
`_FALLBACK_PALETTE` is computed as `sum(ord(ch) for ch in name) %
len(_FALLBACK_PALETTE)` — a simple, fully deterministic checksum of the
name's character codes. This guarantees the same unrecognised colour
name always renders identically both within a single run *and* across
separate runs of the program.

---

## 5. A complete worked trace

Using `maps/easy_linear.txt`:

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

**Parsing** (`mapfile.read_map`): four zones and three links are built;
`base` and `dock` have their capacity forced to `None` (unlimited) at
the end, since they are the start/end zones; `gate` and `relay` keep the
default capacity of `1`.

**Routing** (`router.plan_fleet`): only one route exists at all —
`[base, gate, relay, dock]` — so `discover_routes` finds it once and, on
its second attempt, finds the *same* route again (now-penalised `gate`
and `relay` don't change which route is cheapest, since there is no
alternative) and stops. `route_turn_cost` = `MOVE_TURNS[NORMAL] +
MOVE_TURNS[PRIORITY] + MOVE_TURNS[NORMAL]` = `1 + 1 + 1` = `3`.
`route_capacity` = `min(gate.capacity=1, relay.capacity=1, link
capacities=1,1,1)` = `1`. Both drones are assigned this one route (with
drone 2's estimated score `3 + 1 // 1 = 4`, still the only option).

**Simulating** (`simulate.Simulation.run`), turn by turn — `D1` and
`D2` both start at `step=0` (`base`), which is unlimited, so both are
technically "in" `base` at once with no capacity conflict:

- **Turn 1**: both are `waiting`, sorted by `-step` — tied at `step=0`,
  so their relative order comes from the original list order (`D1`
  before `D2`). `D1` tries to move into `gate`: `gate` is empty
  (`_zone_load["gate"] == 0 < 1`), the link is free, so it succeeds —
  `gate`'s load becomes `1`, `D1.step` becomes `1`. Move string:
  `D1-gate`. Next, `D2` tries to move into `gate` too: `_zone_load["gate"]`
  is now `1`, which is *not* `< 1`, so `has_room` returns `False` — `D2`
  waits. Turn output: `D1-gate`.
- **Turn 2**: `D1` (at `gate`, `step=1`) is furthest along, processed
  first: it tries to enter `relay`. `relay` is empty, link free — it
  succeeds. Because `relay`'s kind is `PRIORITY`, `MOVE_TURNS[PRIORITY]
  == 1`, so this is a same-turn move: `gate`'s load drops back to `0`
  as `D1` departs it. Move string: `D1-relay`. Then `D2` (still at
  `base`, `step=0`) tries `gate` again — and because `D1`'s departure
  from `gate` already ran *earlier in this same turn*, `gate`'s load is
  back to `0`, so `D2` succeeds too. Move string: `D2-gate`. Turn
  output: `D1-relay D2-gate` — exactly the same-turn hand-off from §2.5.
- **Turn 3**: `D1` (at `relay`) moves into `dock` (unlimited capacity,
  always succeeds): `D1-dock`, and `D1.delivered` becomes `True`. `D2`
  (at `gate`) moves into `relay`, now vacated by `D1` earlier this same
  turn: `D2-relay`. Turn output: `D1-dock D2-relay`.
- **Turn 4**: `D1` is already delivered, so it's excluded from `waiting`
  entirely. `D2` moves from `relay` into `dock`: `D2-dock`, and is now
  delivered too. Turn output: `D2-dock`.
- `all(drone.delivered ...)` is now `True`, the `while` loop in `run`
  exits, and the four collected turns are returned.

This matches exactly what running the program produces:

```
$ python3 main.py maps/easy_linear.txt --no-color
D1-gate
D1-relay D2-gate
D1-dock D2-relay
D2-dock
```

---

## 6. Design decisions and trade-offs

- **Zero third-party runtime dependencies.** Everything used —
  `dataclasses`, `enum`, `heapq`, `math`, `argparse` — is Python's
  standard library. This keeps `make install` trivial and means the
  entire codebase can be read without first learning an external
  library's API.
- **Static topology vs. live simulation state are two different
  objects.** `Network`/`Zone`/`Link` never change once built by
  `read_map`; all per-run occupancy bookkeeping lives inside
  `Simulation`. One parsed `Network` could drive several independent
  `Simulation` runs (e.g. to compare strategies) without re-parsing.
- **Greedy load balancing over provably-optimal scheduling.** `plan_fleet`
  does not guarantee the mathematically minimal total turn count — doing
  so in general is a much harder combinatorial scheduling problem. It
  instead uses a fast, understandable heuristic (§2.4) that adapts
  sensibly to a map's actual shape and comfortably meets the subject's
  benchmark turn targets on every provided sample map.
- **Detect-and-report deadlock over provably-deadlock-free scheduling.**
  A fully pre-committed space-time-reservation scheme could guarantee no
  deadlock is ever possible, at the cost of a meaningfully more complex
  planning phase and losing the ability to route different drones by
  different, independently-discovered paths. This project instead keeps
  planning and execution cleanly separate and simple, and relies on
  routes naturally flowing start-to-end (rarely producing true cycles)
  plus a generous stall counter to catch the rare case where they don't.

---

## 7. Complexity and memory

- **Parsing**: `O(L)` in the number of lines `L` — one pass, constant
  work per line.
- **Routing**: each `shortest_path` call is `O((V + E) log V)`; up to 6
  are run, so route discovery is `O(6 (V + E) log V)`. `plan_fleet` then
  does `O(drones × routes_found)` work to assign the whole fleet — at
  most `O(6 × drones)`. All of this runs exactly once, before
  simulation starts; no path is ever recomputed mid-simulation.
- **Simulation**: each turn is `O(drones log drones)` (dominated by the
  two sorts in `_advance_turn`); the number of turns is bounded in
  practice by the sum of route lengths plus queuing delay, and
  defensively capped by `_STALL_LIMIT` in the pathological case.
- **Memory**: `O(V + E)` for the `Network` and its adjacency list,
  `O(V + E)` for the occupancy counters in `Simulation`, and
  `O(drones × average route length)` for the routes themselves — nothing
  in the design holds more than one turn's worth of transient state at a
  time beyond that.
