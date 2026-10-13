# Rubik's Cube Optimal Solver — Korf's IDA* Algorithm

## Project Overview

This project implements an **optimal solver** for the 3×3 Rubik's Cube using Richard Korf's 1997 algorithm — a combination of **Iterative Deepening A\* (IDA\*)** search and **Pattern Databases (PDBs)** as heuristics. The solver guarantees finding the shortest possible solution (in terms of half-turn metric) for any scrambled cube configuration.

The Rubik's Cube has approximately **4.3 × 10¹⁹** (43 quintillion) possible states, making brute-force search impossible. Korf's algorithm solves this by:

1. Breaking the cube into smaller **sub-problems** (corners, edge groups)
2. **Precomputing** the exact solution depth for every configuration of each sub-problem
3. Using these precomputed values as a **lower-bound heuristic** to guide an intelligent search

This was the first algorithm to find provably optimal solutions to random Rubik's Cube instances, and it remains one of the most elegant applications of heuristic search in computer science.

---

## Core Data Structures

### Cube State Representation

The cube is represented at the **cubie level**, not the facelet (sticker) level. A Rubik's Cube has:

- **8 corner cubies**, each with a **position** (0–7) and **orientation** (0, 1, or 2)
- **12 edge cubies**, each with a **position** (0–11) and **orientation** (0 or 1)

```
Corner Positions:              Edge Positions:
  0: URF (Up-Right-Front)        0: UR    4: DR    8: FR
  1: UFL (Up-Front-Left)         1: UF    5: DF    9: FL
  2: ULB (Up-Left-Back)          2: UL    6: DL   10: BL
  3: UBR (Up-Back-Right)         3: UB    7: DB   11: BR
  4: DFR (Down-Front-Right)
  5: DLF (Down-Left-Front)
  6: DBL (Down-Back-Left)
  7: DRB (Down-Right-Back)
```

The state is stored as four arrays:

```cpp
struct CubeState {
    uint8_t cp[8];   // corner permutation — cp[i] = which cubie is at position i
    uint8_t co[8];   // corner orientation — 0: correct, 1: CW twist, 2: CCW twist
    uint8_t ep[12];  // edge permutation
    uint8_t eo[12];  // edge orientation — 0: correct, 1: flipped
};
```

**Why cubie-level?** Pattern databases index directly into corner/edge configurations. A facelet-level representation (54 stickers) would require expensive conversion before every PDB lookup, which happens millions of times during search.

**Orientation rules:**
- Corner orientation is defined by where the U/D-colored sticker faces. If it's on a U/D face → 0; rotated clockwise (viewing from outside) → 1; counter-clockwise → 2.
- Edge orientation flips only on **F and B quarter turns**. U, D, R, L moves preserve edge orientation.

### Move Representation

The standard 3×3 cube has **6 faces** (U, D, R, L, F, B), each with **3 possible turns**:
- Clockwise 90° (e.g., `R`)
- Half turn 180° (e.g., `R2`)
- Counter-clockwise 90° (e.g., `R'`)

This gives **18 total moves**, encoded as integers 0–17.

Each move is defined by a **MoveTable** — a permutation + orientation delta:

```cpp
struct MoveTable {
    uint8_t cp[8];   // corner permutation (position i receives cubie from cp[i])
    int8_t  co[8];   // corner orientation change
    uint8_t ep[12];  // edge permutation
    int8_t  eo[12];  // edge orientation change
};
```

Only the 6 clockwise (CW) moves are manually defined using **4-cycles**. The half-turns and counter-clockwise moves are derived by **composing** the CW move with itself:

```
Half turn  = CW ∘ CW         (apply CW twice)
CCW turn   = CW ∘ CW ∘ CW   (apply CW three times = same as one reverse)
```

This composition approach eliminates manual errors — if the CW definition is correct, the derived moves are guaranteed correct.

Applying a move to a state:

```cpp
// For each position i in the new state:
new_cp[i] = old_cp[ move.cp[i] ];                    // follow the permutation
new_co[i] = (old_co[ move.cp[i] ] + move.co[i]) % 3; // add orientation change mod 3
```

---

## Pattern Databases (PDBs)

### What is a Pattern Database?

A Pattern Database is a **precomputed lookup table** that stores the minimum number of moves needed to solve a **sub-problem** of the cube. It's built by running a **reverse BFS** (Breadth-First Search) from the solved state.

The key insight: if it takes at least *k* moves to solve just the corners, then it takes at least *k* moves to solve the entire cube. This gives us an **admissible heuristic** — it never overestimates.

### The Three PDBs

Korf's algorithm uses three pattern databases:

#### 1. Corner PDB — All 8 Corners

Tracks the position and orientation of all 8 corner cubies, ignoring edges entirely.

- **Positions:** 8 corners can be in 8! = 40,320 arrangements
- **Orientations:** Each corner has 3 possible orientations, but the 8th is determined by the other 7 (constraint of the cube group), so 3⁷ = 2,187 possibilities
- **Total states:** 40,320 × 2,187 = **88,179,840**
- **Max depth:** ~11 moves
- **Storage:** ~44 MB (nibble-packed, 4 bits per entry)

#### 2. Edge PDB Group 0 — Edges 0–5 (UR, UF, UL, UB, DR, DF)

Tracks only 6 of the 12 edges, ignoring the other 6 and all corners.

- **Positions:** 6 edges placed in 12 slots = P(12,6) = 665,280
- **Orientations:** 2⁶ = 64
- **Total states:** 665,280 × 64 = **42,577,920**
- **Max depth:** ~10 moves
- **Storage:** ~21 MB (nibble-packed)

#### 3. Edge PDB Group 1 — Edges 6–11 (DL, DB, FR, FL, BL, BR)

Same structure as Group 0 but tracking the other 6 edges.

- **Total states:** **42,577,920** (same size)
- **Storage:** ~21 MB

**Total memory:** ~86.7 MB (nibble-packed) or ~170 MB (byte-packed)

### Combined Heuristic

For any cube state, the heuristic value is:

```
h(state) = max( corner_PDB[state], edge0_PDB[state], edge1_PDB[state] )
```

Since each PDB provides an admissible (never-overestimating) lower bound, and the maximum of admissible heuristics is also admissible, this combined heuristic is **safe to use with A\*/IDA\*** and still guarantees optimality.

### PDB Generation via BFS

Generation works by running BFS **backward from the solved state**:

```
1. Mark solved state with depth 0
2. For each state at depth d:
     Apply all 18 moves to get neighbor states
     If a neighbor hasn't been visited:
       Mark it with depth d+1
       Add to queue for next iteration
3. Continue until all reachable states are visited
```

For corners, this explores all 88 million states. Every possible corner configuration gets assigned its exact solution depth. This process takes a few minutes and only needs to happen once — the results are saved to disk as `.pdb` files.

---

## Ranking Functions (State → Index Mapping)

To use a PDB, we need a **perfect hash function** that maps a cube sub-state to a compact integer index. This is where combinatorial mathematics comes in.

### Corner Index — Lehmer Code

The corner permutation (which cubie is at each position) is ranked using the **Lehmer code** (factorial number system):

```
For permutation [2, 0, 3, 1]:
  Position 0: value 2, count elements smaller to its right: 0,1 → 2
  Position 1: value 0, count elements smaller to its right: none → 0
  Position 2: value 3, count elements smaller to its right: 1 → 1
  Position 3: value 1, nothing right → 0
  
Lehmer code: [2, 0, 1, 0]
Rank = 2×3! + 0×2! + 1×1! + 0×0! = 12 + 0 + 1 + 0 = 13
```

For 8 corners, this maps any permutation to a unique integer in [0, 8!) = [0, 40320).

**Corner orientation** is ranked as a base-3 number using the first 7 values (the 8th is determined by the constraint that total orientation ≡ 0 mod 3):

```
Orientations [1, 2, 0, 0, 1, 2, 0, ?]
? = (3 - (1+2+0+0+1+2+0) % 3) % 3 = 0
Rank = 1×3⁶ + 2×3⁵ + 0×3⁴ + 0×3³ + 1×3² + 2×3¹ + 0×3⁰ = 729 + 486 + 9 + 6 = 1230
```

**Combined corner index:**

```
corner_index = perm_rank × 2187 + ori_rank
```

This maps any corner state to a unique integer in [0, 88,179,840).

### Edge Group Index — Partial Permutation Ranking

For 6-of-12 edges, we need to rank a **partial permutation**: 6 distinguishable items placed in 12 slots.

The ranking algorithm processes each edge in order, counting how many unused slots fall before its actual slot:

```
6 edges at positions [1, 4, 7, 0, 9, 11] in 12 slots:
  Edge 0 at slot 1: 1 free slot below (slot 0) → digit 1, rank = 1
  Edge 1 at slot 4: 3 free slots below (0,2,3) → digit 3, rank = 1×11 + 3 = 14
  Edge 2 at slot 7: 4 free slots below (0,2,3,5) → digit 4, rank = 14×10 + 4 = 144
  ... and so on
```

This produces a unique index in [0, P(12,6)) = [0, 665,280).

Combined with a 6-bit orientation rank:

```
edge_index = partial_perm_rank × 64 + ori_rank
```

Total range: [0, 42,577,920).

**Why this matters:** These ranking functions are called **millions of times** during search. They must be O(n²) at worst (n=8 for corners, n=6 for edges) with no allocations. Our implementation uses simple nested loops and small fixed-size arrays.

---

## IDA* Search Algorithm

### What is IDA*?

IDA* (Iterative Deepening A\*) combines the **optimality of A\*** with the **memory efficiency of DFS**. Unlike A\*, which maintains an open list of all frontier nodes (potentially billions for the Rubik's Cube), IDA* uses only **O(d) stack space** where d is the solution depth.

### How it Works

```
function IDA*(start):
    threshold = h(start)           // initial bound = heuristic of start
    
    loop:
        result = DFS(start, g=0, threshold)
        if result == FOUND: return solution
        if result == ∞: return no solution
        threshold = result         // increase to next promising depth
        
function DFS(state, g, threshold):
    f = g + h(state)               // f = actual cost + estimated remaining
    if f > threshold: return f     // exceeded bound, report minimum
    if h(state) == 0 and is_solved(state): return FOUND
    
    min_exceeded = ∞
    for each move m (with pruning):
        child = apply(state, m)
        result = DFS(child, g+1, threshold)
        if result == FOUND: return FOUND
        min_exceeded = min(min_exceeded, result)
    
    return min_exceeded            // smallest f that exceeded threshold
```

**Key properties:**
- Each iteration is a complete DFS up to the f-cost threshold
- The threshold increases to the smallest f-value that exceeded the previous threshold
- This guarantees optimality — the first solution found is the shortest
- Memory usage: only the current path (max ~20 moves deep)

### Move Pruning

Not all move sequences need to be explored. Two pruning rules dramatically reduce the search tree:

**Rule 1: No same-face repetition.** Applying `R` after `R2` is equivalent to `R'` — a single move. Any two moves of the same face can always be combined into one or zero moves, so we never apply two moves of the same face consecutively.

**Rule 2: Opposite-face ordering.** Opposite faces commute: `U D` produces the same state as `D U`. To avoid exploring both, we enforce that the lower-indexed face always comes first. If the last move was `D` (index 1), we don't allow `U` (index 0) next — because `U D` would have been explored elsewhere.

```cpp
bool is_redundant(int current_face, int last_face) {
    if (current_face == last_face) return true;                    // Rule 1
    if (current_face/2 == last_face/2 && current_face > last_face) // Rule 2
        return true;
    return false;
}
```

These two rules reduce the branching factor from 18 to approximately **15** on average (and lower in practice), which has a massive compound effect over 15-20 levels of search.

---

## Memory Optimization — Nibble Packing

PDB depth values range from 0 to ~11 (corners) or ~10 (edges). These fit in **4 bits** (a nibble, range 0–15), but the smallest addressable unit in C++ is a byte (8 bits). We pack **two entries per byte**:

```cpp
void set_val(uint32_t idx, uint8_t val) {
    uint32_t byte_idx = idx / 2;
    if (idx % 2 == 0)
        data[byte_idx] = (data[byte_idx] & 0xF0) | (val & 0x0F);   // low nibble
    else
        data[byte_idx] = (data[byte_idx] & 0x0F) | ((val << 4));    // high nibble
}

uint8_t get_val(uint32_t idx) const {
    uint32_t byte_idx = idx / 2;
    if (idx % 2 == 0) return data[byte_idx] & 0x0F;
    else              return (data[byte_idx] >> 4) & 0x0F;
}
```

This halves the storage requirement:

| PDB | States | Byte-packed | Nibble-packed |
|-----|--------|-------------|---------------|
| Corners | 88,179,840 | 88 MB | 44 MB |
| Edge Group 0 | 42,577,920 | 42 MB | 21 MB |
| Edge Group 1 | 42,577,920 | 42 MB | 21 MB |
| **Total** | **173 million** | **172 MB** | **86.7 MB** |

---

## Testing and Verification

The solver includes **11 automated tests** that verify correctness at multiple levels:

### Group-Theoretic Tests

These exploit mathematical properties of the Rubik's Cube group:

- **Identity:** The solved state equals itself
- **Inverse:** Applying a move then its inverse returns to solved (`R R' = I`)
- **Order:** Every face turn has order 4 (`R⁴ = I`), every half-turn has order 2 (`R2² = I`)
- These are tested for **all 6 faces**, catching any error in the 6 move definitions

### Index Round-Trip Tests

- Encode a state to its integer index, decode back, re-encode — the index must match
- Solved state maps to index 0 for corners and edge group 0

### Solution Verification

After the solver finds a solution, we **independently verify** it:

```cpp
CubeState verify = scrambled_cube;
verify.apply_moves(solution);
assert(verify.is_solved());  // must return to solved state
```

This catches any bug in the search logic, heuristic, or move application.

---

## Fallback: Simple Heuristic

For testing and demonstration without the multi-minute PDB generation, the project includes a **simple admissible heuristic** that requires no precomputation:

```
h(state) = max(
    ceil(misplaced_corners / 4),   // each move displaces at most 4 corners
    ceil(misplaced_edges / 4),     // each move displaces at most 4 edges
    ceil(twisted_corners / 4),     // each move twists at most 4 corners
    ceil(flipped_edges / 4)        // each move flips at most 4 edges
)
```

This is **admissible** (never overestimates) because each quarter/half turn affects at most 4 corners and 4 edges. However, it's very weak (typically returns 1–4 even for deeply scrambled cubes), so IDA* explores many more nodes. It works for scrambles up to ~7–8 moves deep within reasonable time.

---

## Complexity Analysis

| Aspect | Value |
|--------|-------|
| Total cube states | 4.3 × 10¹⁹ |
| Corner PDB states | 88,179,840 |
| Edge PDB states (each) | 42,577,920 |
| Branching factor (raw) | 18 |
| Branching factor (pruned) | ~15 |
| Maximum solution depth (God's Number) | 20 (half-turn metric) |
| PDB generation time | ~5–10 minutes (one-time) |
| Typical solve time (with PDBs) | Seconds to minutes |
| Memory for PDBs | ~86.7 MB (nibble-packed) |
| Search memory | O(d) where d = solution depth |

---

## Project Structure

```
rubiks-solver/
├── cube.h          — State struct, move enum, ranking function signatures
├── cube.cpp        — Move tables, apply logic, Lehmer/partial-perm ranking
├── pdb.h           — PatternDB and Heuristic class declarations
├── pdb.cpp         — BFS generation, nibble storage, simple heuristic
├── solver.h        — IDASolver declaration with pruning
├── solver.cpp      — IDA* search loop implementation
├── main.cpp        — Tests, demo solves, interactive CLI
└── Makefile        — Build with: make
```

---

## How to Run

```bash
# Compile
make

# Run self-tests
./rubiks_solver test

# Generate pattern databases (run on your machine, takes a few minutes)
./rubiks_solver genpdb

# Solve interactively (auto-loads PDBs if present, else uses simple heuristic)
./rubiks_solver solve
```

**Move notation:** `R` (clockwise), `R'` (counter-clockwise), `R2` (half-turn), for faces U, D, R, L, F, B.

---

## References

1. Korf, R.E. (1997). *Finding Optimal Solutions to Rubik's Cube Using Pattern Databases.* Proceedings of AAAI-97.
2. Korf, R.E. & Felner, A. (2002). *Disjoint Pattern Database Heuristics.* Artificial Intelligence, 134(1-2).
3. Kociemba, H. *Cube Explorer.* http://kociemba.org/cube.htm