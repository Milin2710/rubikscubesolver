#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

// ============================================================
//  Rubik's Cube — Cubie-Level Representation
// ============================================================
//
//  Corners (position index):
//    0:URF  1:UFL  2:ULB  3:UBR  4:DFR  5:DLF  6:DBL  7:DRB
//
//  Edges (position index):
//    0:UR  1:UF  2:UL  3:UB  4:DR  5:DF  6:DL  7:DB
//    8:FR  9:FL 10:BL 11:BR
//
//  Corner orientation (0/1/2):
//    0 = U/D sticker on U/D face
//    1 = U/D sticker rotated CW  (looking at corner from outside)
//    2 = U/D sticker rotated CCW
//
//  Edge orientation (0/1):
//    0 = correct, 1 = flipped
//    Rule: F/B quarter turns flip edges; others don't.
// ============================================================

constexpr int N_CORNERS = 8;
constexpr int N_EDGES   = 12;

// 6 faces × 3 variants (CW, CCW, half-turn) = 18 moves
enum Move : uint8_t {
    U1, U2, U3,   // U, U2, U'
    D1, D2, D3,
    R1, R2, R3,
    L1, L2, L3,
    F1, F2, F3,
    B1, B2, B3,
    N_MOVES = 18
};

// Which face a move belongs to (0-5: U,D,R,L,F,B)
inline int move_face(Move m) { return m / 3; }

// Human-readable move names
inline const char* move_name(Move m) {
    static const char* names[] = {
        "U","U2","U'", "D","D2","D'",
        "R","R2","R'", "L","L2","L'",
        "F","F2","F'", "B","B2","B'"
    };
    return names[m];
}

// ============================================================
//  Cube State
// ============================================================
struct CubeState {
    std::array<uint8_t, N_CORNERS> cp; // corner permutation
    std::array<uint8_t, N_CORNERS> co; // corner orientation
    std::array<uint8_t, N_EDGES>   ep; // edge permutation
    std::array<uint8_t, N_EDGES>   eo; // edge orientation

    // Solved (identity) state
    static CubeState solved();

    bool is_solved() const;

    // Apply a single move
    void apply_move(Move m);

    // Apply a sequence of moves
    void apply_moves(const std::vector<Move>& moves);

    bool operator==(const CubeState& o) const {
        return cp == o.cp && co == o.co && ep == o.ep && eo == o.eo;
    }
    bool operator!=(const CubeState& o) const { return !(*this == o); }
};

// ============================================================
//  Move Tables — internal permutation/orientation data
// ============================================================
struct MoveTable {
    std::array<uint8_t, N_CORNERS> cp;
    std::array<int8_t,  N_CORNERS> co; // orientation delta
    std::array<uint8_t, N_EDGES>   ep;
    std::array<int8_t,  N_EDGES>   eo;
};

// Get the move table for each of the 18 moves
const MoveTable& get_move_table(Move m);

// ============================================================
//  Ranking / Indexing functions (for pattern databases)
// ============================================================

// Corner index: encodes full corner state → [0, 88_179_840)
//   = perm_rank(cp) * 2187 + orientation_rank(co)
constexpr uint32_t N_CORNER_STATES = 88'179'840; // 8! * 3^7

uint32_t corner_index(const CubeState& s);
void     corner_unrank(uint32_t idx, CubeState& s);

// Edge group index: encodes 6-of-12 edge positions + orientations
//   = partial_perm_rank * 64 + orientation_rank
//   Group 1: edges 0-5,  Group 2: edges 6-11
constexpr uint32_t N_EDGE_STATES = 42'577'920; // P(12,6) * 2^6

uint32_t edge_index(const CubeState& s, int group); // group 0 or 1
void     edge_unrank(uint32_t idx, int group, CubeState& s);

// ============================================================
//  Utility
// ============================================================

// Parse a move string like "R U' F2" into a vector of Moves
std::vector<Move> parse_moves(const std::string& s);

// Print a state for debugging
void print_state(const CubeState& s);
