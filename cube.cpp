#include "cube.h"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <sstream>

// ============================================================
//  Solved State
// ============================================================
CubeState CubeState::solved() {
    CubeState s;
    for (int i = 0; i < N_CORNERS; i++) { s.cp[i] = i; s.co[i] = 0; }
    for (int i = 0; i < N_EDGES;   i++) { s.ep[i] = i; s.eo[i] = 0; }
    return s;
}

bool CubeState::is_solved() const {
    return *this == CubeState::solved();
}

// ============================================================
//  Base Move Tables (6 clockwise face turns)
//
//  Convention: cp[i] = j means "position i receives the cubie
//  that was at position j". co[i] = twist applied to that cubie.
//  Same for edges.
// ============================================================

// Helper: build a move table from a 4-cycle
static MoveTable make_identity() {
    MoveTable t;
    for (int i = 0; i < N_CORNERS; i++) { t.cp[i] = i; t.co[i] = 0; }
    for (int i = 0; i < N_EDGES;   i++) { t.ep[i] = i; t.eo[i] = 0; }
    return t;
}

// The 6 base CW move tables
static MoveTable base_moves[6];
// All 18 move tables (CW, half, CCW for each face)
static MoveTable all_moves[18];
static bool tables_initialized = false;

static void init_base_moves() {
    // Each base move defined by corner 4-cycle, corner ori deltas,
    // edge 4-cycle, and edge ori deltas.
    //
    // cycle[4] = {a,b,c,d} means a←b←c←d←a (position a receives from b, etc.)

    struct FaceDef {
        int c_cycle[4];
        int c_twist[4]; // orientation delta for each position in cycle
        int e_cycle[4];
        int e_flip[4];  // flip delta for each position in cycle
    };

    //                    Corner cycle      Corner twist       Edge cycle        Edge flip
    FaceDef defs[6] = {
      /* U */ { {0,1,2,3},  {0,0,0,0},     {0,1,2,3},       {0,0,0,0} },
      /* D */ { {4,7,6,5},  {0,0,0,0},     {4,7,6,5},       {0,0,0,0} },
      /* R */ { {0,3,7,4},  {2,1,2,1},     {0,11,4,8},      {0,0,0,0} },
      /* L */ { {1,5,6,2},  {1,2,1,2},     {2,9,6,10},      {0,0,0,0} },
      /* F */ { {0,4,5,1},  {1,2,1,2},     {1,8,5,9},       {1,1,1,1} },
      /* B */ { {3,2,6,7},  {1,2,1,2},     {3,10,7,11},     {1,1,1,1} },
    };

    for (int f = 0; f < 6; f++) {
        base_moves[f] = make_identity();
        auto& t = base_moves[f];
        auto& d = defs[f];

        // Apply corner 4-cycle: pos a ← cubie from pos b
        // cycle = {a,b,c,d}: a←b, b←c, c←d, d←a
        t.cp[d.c_cycle[0]] = d.c_cycle[1];
        t.cp[d.c_cycle[1]] = d.c_cycle[2];
        t.cp[d.c_cycle[2]] = d.c_cycle[3];
        t.cp[d.c_cycle[3]] = d.c_cycle[0];

        t.co[d.c_cycle[0]] = d.c_twist[0];
        t.co[d.c_cycle[1]] = d.c_twist[1];
        t.co[d.c_cycle[2]] = d.c_twist[2];
        t.co[d.c_cycle[3]] = d.c_twist[3];

        // Apply edge 4-cycle
        t.ep[d.e_cycle[0]] = d.e_cycle[1];
        t.ep[d.e_cycle[1]] = d.e_cycle[2];
        t.ep[d.e_cycle[2]] = d.e_cycle[3];
        t.ep[d.e_cycle[3]] = d.e_cycle[0];

        t.eo[d.e_cycle[0]] = d.e_flip[0];
        t.eo[d.e_cycle[1]] = d.e_flip[1];
        t.eo[d.e_cycle[2]] = d.e_flip[2];
        t.eo[d.e_cycle[3]] = d.e_flip[3];
    }
}

// Compose two move tables: result = applying A then B
static MoveTable compose(const MoveTable& A, const MoveTable& B) {
    MoveTable R;
    for (int i = 0; i < N_CORNERS; i++) {
        // Position i in result gets cubie from B.cp[i] in intermediate,
        // which got cubie from A.cp[B.cp[i]] in original.
        R.cp[i] = A.cp[B.cp[i]];
        R.co[i] = (A.co[B.cp[i]] + B.co[i]) % 3;
    }
    for (int i = 0; i < N_EDGES; i++) {
        R.ep[i] = A.ep[B.ep[i]];
        R.eo[i] = (A.eo[B.ep[i]] + B.eo[i]) % 2;
    }
    return R;
}

static void init_all_moves() {
    init_base_moves();
    for (int f = 0; f < 6; f++) {
        all_moves[f * 3 + 0] = base_moves[f];                           // CW  (90°)
        all_moves[f * 3 + 1] = compose(base_moves[f], base_moves[f]);   // Half (180°)
        all_moves[f * 3 + 2] = compose(all_moves[f*3+1], base_moves[f]);// CCW (270°)
    }
    tables_initialized = true;
}

const MoveTable& get_move_table(Move m) {
    if (!tables_initialized) init_all_moves();
    return all_moves[m];
}

// ============================================================
//  Apply Move to State
// ============================================================
void CubeState::apply_move(Move m) {
    const MoveTable& t = get_move_table(m);
    CubeState old = *this;

    for (int i = 0; i < N_CORNERS; i++) {
        cp[i] = old.cp[t.cp[i]];
        co[i] = (old.co[t.cp[i]] + t.co[i]) % 3;
    }
    for (int i = 0; i < N_EDGES; i++) {
        ep[i] = old.ep[t.ep[i]];
        eo[i] = (old.eo[t.ep[i]] + t.eo[i]) % 2;
    }
}

void CubeState::apply_moves(const std::vector<Move>& moves) {
    for (Move m : moves) apply_move(m);
}

// ============================================================
//  Ranking Functions
// ============================================================

// Factorial lookup
static const uint32_t fact[] = {
    1, 1, 2, 6, 24, 120, 720, 5040, 40320, 362880, 3628800, 39916800, 479001600
};

// Rank a full permutation of n elements using Lehmer code
static uint32_t perm_rank(const uint8_t* p, int n) {
    uint32_t rank = 0;
    for (int i = 0; i < n; i++) {
        int count = 0;
        for (int j = i + 1; j < n; j++) {
            if (p[j] < p[i]) count++;
        }
        rank += count * fact[n - 1 - i];
    }
    return rank;
}

// Unrank a full permutation
static void perm_unrank(uint32_t rank, uint8_t* p, int n) {
    std::vector<int> available(n);
    for (int i = 0; i < n; i++) available[i] = i;

    for (int i = 0; i < n; i++) {
        uint32_t f = fact[n - 1 - i];
        int idx = rank / f;
        rank %= f;
        p[i] = available[idx];
        available.erase(available.begin() + idx);
    }
}

// Corner orientation rank: first 7 values as base-3 number
static uint32_t corner_ori_rank(const uint8_t* co) {
    uint32_t r = 0;
    for (int i = 0; i < 7; i++) r = r * 3 + co[i];
    return r;
}

// Corner orientation unrank
static void corner_ori_unrank(uint32_t rank, uint8_t* co) {
    int sum = 0;
    for (int i = 6; i >= 0; i--) {
        co[i] = rank % 3;
        sum += co[i];
        rank /= 3;
    }
    co[7] = (3 - (sum % 3)) % 3; // 8th orientation determined
}

uint32_t corner_index(const CubeState& s) {
    return perm_rank(s.cp.data(), 8) * 2187 + corner_ori_rank(s.co.data());
}

void corner_unrank(uint32_t idx, CubeState& s) {
    corner_ori_unrank(idx % 2187, s.co.data());
    perm_unrank(idx / 2187, s.cp.data(), 8);
    // Leave edges as identity
    for (int i = 0; i < N_EDGES; i++) { s.ep[i] = i; s.eo[i] = 0; }
}

// ============================================================
//  Partial Permutation Ranking for Edge Groups
//
//  We track 6 specific edges (group 0: cubies 0-5, group 1: cubies 6-11).
//  We rank their positions among the 12 edge slots.
//
//  Index = partial_perm_rank * 64 + ori_rank
//  partial_perm_rank ∈ [0, P(12,6)) = [0, 665280)
//  ori_rank ∈ [0, 64) = 6-bit orientation
// ============================================================

// Rank a partial permutation: given that k specific items are placed
// in n slots, rank by their positions (in order of the items).
// positions[i] = slot where item i sits, for i in 0..k-1
static uint32_t partial_perm_rank(const uint8_t* positions, int n, int k) {
    // Use the standard ranking for k items in n positions:
    // For each item i (in order), count how many remaining slots
    // are below its position.
    uint32_t rank = 0;
    bool used[12] = {};

    for (int i = 0; i < k; i++) {
        int pos = positions[i];
        // Count how many slots below 'pos' are still free
        int count = 0;
        for (int j = 0; j < pos; j++) {
            if (!used[j]) count++;
        }
        rank = rank * (n - i) + count;
        used[pos] = true;
    }
    return rank;
}

// Unrank partial permutation
static void partial_perm_unrank(uint32_t rank, uint8_t* positions, int n, int k) {
    // Unrank in reverse
    std::vector<int> digits(k);
    for (int i = k - 1; i >= 0; i--) {
        digits[i] = rank % (n - i);
        rank /= (n - i);
    }

    bool used[12] = {};
    for (int i = 0; i < k; i++) {
        int count = 0;
        for (int j = 0; j < n; j++) {
            if (!used[j]) {
                if (count == digits[i]) {
                    positions[i] = j;
                    used[j] = true;
                    break;
                }
                count++;
            }
        }
    }
}

uint32_t edge_index(const CubeState& s, int group) {
    // Find positions and orientations of the 6 edges in this group
    int base = group * 6; // cubies base..base+5

    uint8_t positions[6];
    uint8_t oris[6];

    // For each cubie in the group, find which slot it's in
    for (int c = 0; c < 6; c++) {
        int cubie = base + c;
        for (int slot = 0; slot < N_EDGES; slot++) {
            if (s.ep[slot] == cubie) {
                positions[c] = slot;
                oris[c] = s.eo[slot];
                break;
            }
        }
    }

    // Orientation rank: 6-bit number
    uint32_t ori_rank = 0;
    for (int i = 0; i < 6; i++) ori_rank = ori_rank * 2 + oris[i];

    return partial_perm_rank(positions, 12, 6) * 64 + ori_rank;
}

void edge_unrank(uint32_t idx, int group, CubeState& s) {
    uint32_t ori_rank = idx % 64;
    uint32_t pp_rank  = idx / 64;

    int base = group * 6;

    uint8_t positions[6];
    partial_perm_unrank(pp_rank, positions, 12, 6);

    uint8_t oris[6];
    for (int i = 5; i >= 0; i--) {
        oris[i] = ori_rank % 2;
        ori_rank /= 2;
    }

    // Build state: place the group's cubies, fill rest with dummies
    // (for PDB generation, we only care about the group's cubies)
    for (int i = 0; i < N_EDGES; i++) { s.ep[i] = 255; s.eo[i] = 0; }
    for (int i = 0; i < N_CORNERS; i++) { s.cp[i] = i; s.co[i] = 0; }

    for (int c = 0; c < 6; c++) {
        s.ep[positions[c]] = base + c;
        s.eo[positions[c]] = oris[c];
    }
}

// ============================================================
//  Parse move string
// ============================================================
std::vector<Move> parse_moves(const std::string& str) {
    std::vector<Move> result;
    std::istringstream iss(str);
    std::string tok;

    while (iss >> tok) {
        int face = -1;
        if (tok[0] == 'U') face = 0;
        else if (tok[0] == 'D') face = 1;
        else if (tok[0] == 'R') face = 2;
        else if (tok[0] == 'L') face = 3;
        else if (tok[0] == 'F') face = 4;
        else if (tok[0] == 'B') face = 5;

        if (face < 0) continue;

        int variant = 0; // CW by default
        if (tok.size() > 1) {
            if (tok[1] == '2') variant = 1;      // half turn
            else if (tok[1] == '\'') variant = 2; // CCW
        }
        result.push_back(static_cast<Move>(face * 3 + variant));
    }
    return result;
}

// ============================================================
//  Debug Print
// ============================================================
void print_state(const CubeState& s) {
    std::cout << "CP: ";
    for (int i = 0; i < N_CORNERS; i++) std::cout << (int)s.cp[i] << " ";
    std::cout << "\nCO: ";
    for (int i = 0; i < N_CORNERS; i++) std::cout << (int)s.co[i] << " ";
    std::cout << "\nEP: ";
    for (int i = 0; i < N_EDGES; i++) std::cout << (int)s.ep[i] << " ";
    std::cout << "\nEO: ";
    for (int i = 0; i < N_EDGES; i++) std::cout << (int)s.eo[i] << " ";
    std::cout << "\n";
}
