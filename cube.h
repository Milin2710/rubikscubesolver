#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <array>

// --- Constants ---
constexpr int N_CORNERS = 8;
constexpr int N_EDGES   = 12;
constexpr int N_MOVES   = 18;

// PDB sizes (from Korf's decomposition)
constexpr uint32_t N_CORNER_STATES = 88179840;   // 8! * 3^7
constexpr uint32_t N_EDGE_STATES   = 42577920;   // P(12,6) * 2^6

// --- Move enumeration ---
// 6 faces × 3 turns = 18 moves
// Index: face * 3 + turn  (turn: 0=CW, 1=half, 2=CCW)
enum Move : uint8_t {
    U1, U2, U3,   // 0-2
    D1, D2, D3,   // 3-5
    R1, R2, R3,   // 6-8
    L1, L2, L3,   // 9-11
    F1, F2, F3,   // 12-14
    B1, B2, B3    // 15-17
};

inline int move_face(Move m)  { return m / 3; }
inline int move_turn(Move m)  { return m % 3; }

// --- Move table: describes the effect of a single move ---
struct MoveTable {
    uint8_t cp[N_CORNERS];   // corner permutation
    int8_t  co[N_CORNERS];   // corner orientation delta
    uint8_t ep[N_EDGES];     // edge permutation
    int8_t  eo[N_EDGES];     // edge orientation delta
};

// --- Cube state ---
struct CubeState {
    uint8_t cp[N_CORNERS];  // corner permutation
    uint8_t co[N_CORNERS];  // corner orientation (0,1,2)
    uint8_t ep[N_EDGES];    // edge permutation
    uint8_t eo[N_EDGES];    // edge orientation (0,1)

    // Initialize to solved state
    void set_solved();
    bool is_solved() const;

    // Apply a move (creates new state)
    CubeState apply(Move m) const;

    // Apply move IN-PLACE (modifies this state)
    void apply_inplace(Move m);

    // Undo a move IN-PLACE (applies inverse)
    void undo_inplace(Move m);

    bool operator==(const CubeState& o) const;
};

// --- Global move tables (initialized once) ---
extern MoveTable move_tables[N_MOVES];
void init_move_tables();

// --- Indexing functions ---
uint32_t corner_index(const CubeState& s);
uint32_t edge_index(const CubeState& s, int group); // group 0: edges 0-5, group 1: edges 6-11

// Unranking (for PDB generation via BFS)
CubeState corner_unrank(uint32_t idx);
CubeState edge_unrank(uint32_t idx, int group);

// --- Move parsing ---
Move parse_move(const std::string& s);
std::string move_to_string(Move m);
std::vector<Move> parse_scramble(const std::string& s);
std::string solution_to_string(const std::vector<Move>& moves);

// --- Move pruning ---
inline bool is_move_redundant(int cur_face, int last_face) {
    if (last_face < 0) return false;           // no previous move
    if (cur_face == last_face) return true;     // same face
    // opposite faces commute — enforce ordering (lower index first)
    if (cur_face / 2 == last_face / 2 && cur_face > last_face)
        return true;
    return false;
}
