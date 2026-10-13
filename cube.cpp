#include "cube.h"
#include <cstring>
#include <sstream>
#include <algorithm>
#include <stdexcept>

// ========== Cube State ==========

void CubeState::set_solved() {
    for (int i = 0; i < N_CORNERS; i++) { cp[i] = i; co[i] = 0; }
    for (int i = 0; i < N_EDGES;   i++) { ep[i] = i; eo[i] = 0; }
}

bool CubeState::is_solved() const {
    for (int i = 0; i < N_CORNERS; i++)
        if (cp[i] != i || co[i] != 0) return false;
    for (int i = 0; i < N_EDGES; i++)
        if (ep[i] != i || eo[i] != 0) return false;
    return true;
}

bool CubeState::operator==(const CubeState& o) const {
    return memcmp(cp, o.cp, N_CORNERS) == 0 &&
           memcmp(co, o.co, N_CORNERS) == 0 &&
           memcmp(ep, o.ep, N_EDGES) == 0 &&
           memcmp(eo, o.eo, N_EDGES) == 0;
}

// Apply a move — returns NEW state
CubeState CubeState::apply(Move m) const {
    const MoveTable& t = move_tables[m];
    CubeState ns;
    for (int i = 0; i < N_CORNERS; i++) {
        ns.cp[i] = cp[t.cp[i]];
        ns.co[i] = (co[t.cp[i]] + t.co[i]) % 3;
    }
    for (int i = 0; i < N_EDGES; i++) {
        ns.ep[i] = ep[t.ep[i]];
        ns.eo[i] = (eo[t.ep[i]] + t.eo[i]) % 2;
    }
    return ns;
}

// Apply move IN-PLACE — modifies this state directly (avoids allocation)
void CubeState::apply_inplace(Move m) {
    const MoveTable& t = move_tables[m];
    // We need temps because source and dest overlap
    uint8_t tcp[N_CORNERS], tco[N_CORNERS];
    for (int i = 0; i < N_CORNERS; i++) {
        tcp[i] = cp[t.cp[i]];
        tco[i] = (co[t.cp[i]] + t.co[i]) % 3;
    }
    memcpy(cp, tcp, N_CORNERS);
    memcpy(co, tco, N_CORNERS);

    uint8_t tep[N_EDGES], teo[N_EDGES];
    for (int i = 0; i < N_EDGES; i++) {
        tep[i] = ep[t.ep[i]];
        teo[i] = (eo[t.ep[i]] + t.eo[i]) % 2;
    }
    memcpy(ep, tep, N_EDGES);
    memcpy(eo, teo, N_EDGES);
}

// Undo a move — apply the inverse move in-place
void CubeState::undo_inplace(Move m) {
    // Inverse of CW = CCW, inverse of half = half, inverse of CCW = CW
    int face = move_face(m);
    int turn = move_turn(m);
    int inv_turn;
    if (turn == 0)      inv_turn = 2;  // CW -> CCW
    else if (turn == 2) inv_turn = 0;  // CCW -> CW
    else                inv_turn = 1;  // half -> half
    apply_inplace(static_cast<Move>(face * 3 + inv_turn));
}

// ========== Move Table Construction ==========

MoveTable move_tables[N_MOVES];

// Helper: make identity move table
static MoveTable identity_table() {
    MoveTable t;
    for (int i = 0; i < N_CORNERS; i++) { t.cp[i] = i; t.co[i] = 0; }
    for (int i = 0; i < N_EDGES;   i++) { t.ep[i] = i; t.eo[i] = 0; }
    return t;
}

// Helper: compose two move tables (apply b then a, i.e., a(b(x)))
static MoveTable compose(const MoveTable& a, const MoveTable& b) {
    MoveTable r;
    for (int i = 0; i < N_CORNERS; i++) {
        r.cp[i] = b.cp[a.cp[i]];
        r.co[i] = (b.co[a.cp[i]] + a.co[i]) % 3;
    }
    for (int i = 0; i < N_EDGES; i++) {
        r.ep[i] = b.ep[a.ep[i]];
        r.eo[i] = (b.eo[a.ep[i]] + a.eo[i]) % 2;
    }
    return r;
}

// Apply a 4-cycle to corners: positions a->b->c->d->a with orientations
static void corner_cycle(MoveTable& t, int a, int b, int c, int d,
                         int oa, int ob, int oc, int od) {
    t.cp[a] = d; t.cp[b] = a; t.cp[c] = b; t.cp[d] = c;
    t.co[a] = od; t.co[b] = oa; t.co[c] = ob; t.co[d] = oc;
}

// Apply a 4-cycle to edges: positions a->b->c->d->a with orientations
static void edge_cycle(MoveTable& t, int a, int b, int c, int d,
                       int oa, int ob, int oc, int od) {
    t.ep[a] = d; t.ep[b] = a; t.ep[c] = b; t.ep[d] = c;
    t.eo[a] = od; t.eo[b] = oa; t.eo[c] = ob; t.eo[d] = oc;
}

void init_move_tables() {
    // Corner positions:
    //   0:URF 1:UFL 2:ULB 3:UBR 4:DFR 5:DLF 6:DBL 7:DRB
    // Edge positions:
    //   0:UR 1:UF 2:UL 3:UB 4:DR 5:DF 6:DL 7:DB 8:FR 9:FL 10:BL 11:BR

    // Define the 6 CW quarter turns
    // U face CW: corners 0->3->2->1, edges 0->3->2->1; no orientation change
    MoveTable u = identity_table();
    corner_cycle(u, 0,3,2,1, 0,0,0,0);
    edge_cycle(u, 0,3,2,1, 0,0,0,0);

    // D face CW: corners 4->5->6->7, edges 4->5->6->7; no orientation change
    MoveTable d = identity_table();
    corner_cycle(d, 4,5,6,7, 0,0,0,0);
    edge_cycle(d, 4,5,6,7, 0,0,0,0);

    // R face CW: corners 0->4->7->3, edges 0->8->4->11; corner orientations change
    MoveTable r = identity_table();
    corner_cycle(r, 0,4,7,3, 1,2,1,2);
    edge_cycle(r, 0,8,4,11, 0,0,0,0);

    // L face CW: corners 1->2->6->5, edges 2->10->6->9; corner orientations change
    MoveTable l = identity_table();
    corner_cycle(l, 1,2,6,5, 2,1,2,1);
    edge_cycle(l, 2,10,6,9, 0,0,0,0);

    // F face CW: corners 0->1->5->4, edges 1->9->5->8; both corners and edges change orientation
    MoveTable f = identity_table();
    corner_cycle(f, 0,1,5,4, 2,1,2,1);
    edge_cycle(f, 1,9,5,8, 1,1,1,1);

    // B face CW: corners 2->3->7->6, edges 3->11->7->10; both change orientation
    MoveTable b = identity_table();
    corner_cycle(b, 2,3,7,6, 1,2,1,2);
    edge_cycle(b, 3,11,7,10, 1,1,1,1);

    MoveTable cw[6] = {u, d, r, l, f, b};

    // Generate all 18 moves: CW, Half, CCW for each face
    for (int face = 0; face < 6; face++) {
        move_tables[face * 3 + 0] = cw[face];                          // CW
        move_tables[face * 3 + 1] = compose(cw[face], cw[face]);       // Half = CW^2
        move_tables[face * 3 + 2] = compose(move_tables[face*3+1], cw[face]); // CCW = CW^3
    }
}

// ========== Ranking Functions ==========

// Lehmer code ranking for full permutation of n elements
static uint32_t perm_rank(const uint8_t* perm, int n) {
    uint32_t rank = 0;
    for (int i = 0; i < n; i++) {
        int count = 0;
        for (int j = i + 1; j < n; j++) {
            if (perm[j] < perm[i]) count++;
        }
        rank = rank * (n - i) + count;
    }
    return rank;
}

// Unrank a full permutation
static void perm_unrank(uint32_t rank, uint8_t* perm, int n) {
    // Extract Lehmer code digits
    int lehmer[16];
    for (int i = n - 1; i >= 0; i--) {
        lehmer[i] = rank % (n - i);
        rank /= (n - i);
    }
    // Convert Lehmer code to permutation
    bool used[16] = {};
    for (int i = 0; i < n; i++) {
        int count = 0;
        for (int j = 0; j < n; j++) {
            if (!used[j]) {
                if (count == lehmer[i]) {
                    perm[i] = j;
                    used[j] = true;
                    break;
                }
                count++;
            }
        }
    }
}

// Partial permutation rank: k items from n slots
// positions[i] = slot where item i sits (0-based)
static uint32_t partial_perm_rank(const uint8_t* positions, int n, int k) {
    uint32_t rank = 0;
    bool used[16] = {};
    for (int i = 0; i < k; i++) {
        int pos = positions[i];
        // Count unused positions less than pos
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
    // Extract digits
    int digits[16];
    for (int i = k - 1; i >= 0; i--) {
        digits[i] = rank % (n - i);
        rank /= (n - i);
    }
    bool used[16] = {};
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

// --- Corner index: perm_rank(cp, 8) * 2187 + ori_rank(co) ---
// Range: [0, 88,179,840)
uint32_t corner_index(const CubeState& s) {
    uint32_t prank = perm_rank(s.cp, N_CORNERS);
    // Orientation rank: base-3 number using first 7 values (8th is determined)
    uint32_t orank = 0;
    for (int i = 0; i < N_CORNERS - 1; i++) {
        orank = orank * 3 + s.co[i];
    }
    return prank * 2187 + orank;
}

// --- Edge index for group 0 (edges 0-5) or group 1 (edges 6-11) ---
// Range: [0, 42,577,920)
uint32_t edge_index(const CubeState& s, int group) {
    int base = group * 6;  // 0 for group 0, 6 for group 1

    // Find positions of the 6 edges in this group
    uint8_t positions[6];
    for (int i = 0; i < 6; i++) {
        // Find where edge (base+i) currently sits
        for (int j = 0; j < N_EDGES; j++) {
            if (s.ep[j] == base + i) {
                positions[i] = j;
                break;
            }
        }
    }

    uint32_t prank = partial_perm_rank(positions, N_EDGES, 6);

    // Orientation rank: 6-bit number
    uint32_t orank = 0;
    for (int i = 0; i < 6; i++) {
        orank = orank * 2 + s.eo[positions[i]];
    }

    return prank * 64 + orank;
}

// Unrank corner state (for PDB generation)
CubeState corner_unrank(uint32_t idx) {
    CubeState s;
    s.set_solved();

    uint32_t orank = idx % 2187;
    uint32_t prank = idx / 2187;

    perm_unrank(prank, s.cp, N_CORNERS);

    // Decode orientation
    int sum = 0;
    for (int i = N_CORNERS - 2; i >= 0; i--) {
        s.co[i] = orank % 3;
        sum += s.co[i];
        orank /= 3;
    }
    s.co[N_CORNERS - 1] = (3 - sum % 3) % 3;

    return s;
}

// Unrank edge state (for PDB generation)
CubeState edge_unrank(uint32_t idx, int group) {
    CubeState s;
    s.set_solved();
    int base = group * 6;

    uint32_t orank = idx % 64;
    uint32_t prank = idx / 64;

    uint8_t positions[6];
    partial_perm_unrank(prank, positions, N_EDGES, 6);

    // Set edge permutation: place the 6 group edges at their ranked positions
    // First, mark all edge positions as "don't care" (we only track our 6)
    for (int i = 0; i < N_EDGES; i++) s.ep[i] = 255;  // sentinel
    for (int i = 0; i < 6; i++) {
        s.ep[positions[i]] = base + i;
    }
    // Fill remaining positions with other edges (in order, for consistency)
    int fill = (group == 0) ? 6 : 0;
    for (int i = 0; i < N_EDGES; i++) {
        if (s.ep[i] == 255) {
            s.ep[i] = fill++;
            if (group == 0 && fill == 6) fill = 6;  // skip past group 0
            if (group == 1 && fill == 6) fill = 12;  // skip past group 1 (already handled)
        }
    }

    // Decode orientation for the 6 group edges
    for (int i = 0; i < N_EDGES; i++) s.eo[i] = 0;
    for (int i = 5; i >= 0; i--) {
        s.eo[positions[i]] = orank % 2;
        orank /= 2;
    }

    return s;
}

// ========== Move Parsing ==========

static const char* move_names[] = {
    "U", "U2", "U'", "D", "D2", "D'",
    "R", "R2", "R'", "L", "L2", "L'",
    "F", "F2", "F'", "B", "B2", "B'"
};

Move parse_move(const std::string& s) {
    for (int i = 0; i < N_MOVES; i++) {
        if (s == move_names[i]) return static_cast<Move>(i);
    }
    throw std::runtime_error("Invalid move: " + s);
}

std::string move_to_string(Move m) {
    return move_names[static_cast<int>(m)];
}

std::vector<Move> parse_scramble(const std::string& s) {
    std::vector<Move> moves;
    std::istringstream iss(s);
    std::string token;
    while (iss >> token) {
        moves.push_back(parse_move(token));
    }
    return moves;
}

std::string solution_to_string(const std::vector<Move>& moves) {
    std::string result;
    for (size_t i = 0; i < moves.size(); i++) {
        if (i > 0) result += " ";
        result += move_to_string(moves[i]);
    }
    return result;
}
