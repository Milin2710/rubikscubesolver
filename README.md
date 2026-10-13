# Rubik's Cube Optimal Solver

An optimal 3×3 Rubik's Cube solver in C++17, based on **Korf's IDA\* algorithm with pattern database heuristics** (Korf, 1997). It finds a shortest possible solution (in the half-turn metric) for any scramble it can search within a reasonable time.

**Highlights**

- Cubie-level cube model with all 18 moves (6 faces × clockwise, counter-clockwise, half-turn)
- Three pattern databases covering **173,335,680 precomputed states**, stored in about 87 MB using 4-bit packing
- IDA\* search with admissible heuristics and move pruning, using memory proportional only to the solution depth
- Built-in self-tests, including independent verification of every solution the solver returns
- Zero dependencies: just `g++` and `make`

> For a deeper walkthrough of the design, see [explainer.md](explainer.md).

---

## How it works

### 1. Cube representation

The cube is stored as 8 corners and 12 edges, each with a **position** and an **orientation** (corner twist 0–2, edge flip 0–1). Only the six clockwise face turns are defined by hand; the counter-clockwise and half-turn moves are derived by composing them.

### 2. Pattern databases (PDBs)

A pattern database stores the exact minimum number of moves needed to solve a *sub-problem* (only some pieces tracked). The solver uses three, each built by a backward breadth-first search from the solved state:

| Database | Pieces tracked | States |
|---|---|---|
| Corners | all 8 corners (position + orientation) | 88,179,840 |
| Edges A | 6 of the 12 edges | 42,577,920 |
| Edges B | the other 6 edges | 42,577,920 |
| **Total** | | **173,335,680** |

Sub-states are mapped to unique integers with Lehmer codes and partial-permutation ranking, so a lookup is a single array access. Two 4-bit entries are packed into each byte, which keeps all three tables at about 87 MB.

### 3. IDA\* search

IDA\* runs a depth-first search with an increasing cost threshold. The heuristic for a state is the **maximum** of the three PDB values, which stays admissible, so the first solution found is optimal. Two pruning rules cut the branching factor:

- never turn the same face twice in a row
- for opposite faces (U/D, R/L, F/B), only allow one ordering, since the two orders reach the same state

### 4. Fallback heuristic

If no PDB files are found, the solver uses a simple admissible heuristic based on misplaced and twisted pieces. It needs no setup but is weak, so it only handles shallow scrambles (it stops after a 50-million-node limit).

---

## Getting started

**Requirements:** `g++` with C++17 support and `make`.

```bash
# Build
make
# Windows without make:
# g++ -std=c++17 -O2 -o rubiks_solver cube.cpp pdb.cpp solver.cpp main.cpp

# Run the self-tests (11 tests)
make test            # or: ./rubiks_solver test

# Generate the pattern databases (one-time, takes several minutes, ~170 MB of RAM)
./rubiks_solver genpdb

# Run the built-in demo scrambles (1 to 13 moves)
./rubiks_solver solve

# Enter your own scramble
./rubiks_solver interactive

# On Windows, use .\rubiks_solver.exe instead of ./rubiks_solver
```

In `solve` mode the program first runs demo scrambles of 1 to 13 moves (60-second time limit each). For every scramble it prints the initial heuristic value, the nodes expanded at each IDA\* threshold, the optimal solution, and a verification result. Example output for a 13-move scramble:

```
Pattern databases loaded successfully.
Time limit: 60s per scramble

--- 13-move ---
Scramble: F R' U2 B L D' R2 U' F2 L' D B' R (13 moves)
  Initial h = 7
  Threshold 7: 18 nodes, 0.000981s
  Threshold 8: 81 nodes, 0s
  Threshold 9: 579 nodes, 0s
  Threshold 10: 5646 nodes, 0.002002s
  Threshold 11: 70647 nodes, 0.035003s
  Threshold 12: 881163 nodes, 0.349162s
  Threshold 13: 5352701 nodes, 2.0263s
SOLVED! Depth: 13  Nodes: 5352701  Time: 2.41934s
Solution: R' B D' L F2 U R2 D L' B' U2 R F'
Verification: CORRECT
```

To solve your own scramble, run `./rubiks_solver interactive` and enter it in standard move notation: `U D L R F B`, with `'` for counter-clockwise and `2` for a half-turn, for example `R U2 F' D`. The built-in demo scrambles are defined in `main.cpp`.

To remove build files and the generated databases: `make clean`.

---

## Performance

Results from the built-in demo scrambles (`./rubiks_solver solve`, with all three pattern databases loaded). Every solution was verified as correct.

| Scramble length | Optimal depth | Nodes expanded | Time |
|---|---|---|---|
| 1 move | 1 | 9 | 0.0009 s |
| 2 moves | 2 | 6 | 0.002 s |
| 3 moves | 3 | 21 | 0.001 s |
| 4 moves | 4 | 24 | 0.001 s |
| 5 moves | 5 | 36 | 0.001 s |
| 6 moves | 6 | 54 | 0.001 s |
| 8 moves | 8 | 88 | 0.003 s |
| 10 moves | 10 | 2,497 | 0.005 s |
| 12 moves | 12 | 688,655 | 0.33 s |
| 13 moves | 13 | 5,352,701 | 2.42 s |

**What the numbers show**

- For scrambles of up to 6 moves, the initial heuristic equals the optimal solution length, so the search expands only a handful of nodes.
- Beyond that, each extra level of depth multiplies the node count by roughly 6-13× (for example 5,646 → 70,647 → 881,163 → 5,352,701 nodes across thresholds 10 to 13 in the 13-move run above).
- These are fixed demo scrambles, not averages over random positions. Random scrambles of the same length can take longer.

---

## Testing

`./rubiks_solver test` runs 11 self-tests covering:

- solved-state identity and move inverses (for example `R R' = identity`)
- the order of each face turn and half-turn
- corner and edge index round-trips (rank → unrank → original state)
- move-string parsing
- independent verification that every solution returned by the solver really solves the scramble

---

## Project structure

| File | Purpose |
|---|---|
| `cube.h` / `cube.cpp` | Cube state, the 18 moves, move tables, ranking and unranking functions |
| `pdb.h` / `pdb.cpp` | Pattern databases: generation by backward BFS, lookup, save and load; the simple fallback heuristic |
| `solver.h` / `solver.cpp` | IDA\* solver and move pruning |
| `main.cpp` | Command-line entry point (`test`, `genpdb`, `solve`, `interactive`) and self-tests |
| `Makefile` | Build, test and clean targets |
| `explainer.md` | Detailed explanation of the design |

---

## Limitations

- Optimality is in the **half-turn metric** (any face turn counts as one move), not the quarter-turn metric.
- Optimal search is exponential in scramble depth. The demo covers scrambles of up to 13 moves, and very deep scrambles (near the 20-move "God's number") are out of reach for this implementation.
- The pattern databases must be generated once before the solver can handle anything beyond shallow scrambles.

## Reference

R. E. Korf, *Finding Optimal Solutions to Rubik's Cube Using Pattern Databases*, AAAI-97, 1997.