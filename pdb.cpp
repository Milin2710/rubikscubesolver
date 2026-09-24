#include "pdb.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <queue>
#include <chrono>
#include <cassert>

// ============================================================
//  Nibble-packed storage (2 entries per byte, max value 15)
// ============================================================

void PatternDB::set_val(uint32_t idx, uint8_t val) {
    uint32_t byte_idx = idx / 2;
    if (idx % 2 == 0)
        data_[byte_idx] = (data_[byte_idx] & 0xF0) | (val & 0x0F);
    else
        data_[byte_idx] = (data_[byte_idx] & 0x0F) | ((val & 0x0F) << 4);
}

uint8_t PatternDB::get_val(uint32_t idx) const {
    uint32_t byte_idx = idx / 2;
    if (idx % 2 == 0)
        return data_[byte_idx] & 0x0F;
    else
        return (data_[byte_idx] >> 4) & 0x0F;
}

uint32_t PatternDB::index(const CubeState& s) const {
    switch (type_) {
        case CORNERS:      return corner_index(s);
        case EDGE_GROUP_0: return edge_index(s, 0);
        case EDGE_GROUP_1: return edge_index(s, 1);
    }
    return 0;
}

uint8_t PatternDB::lookup(const CubeState& s) const {
    return get_val(index(s));
}

// ============================================================
//  BFS Generation
//
//  We do a reverse BFS from the solved state. For each state at
//  depth d, we apply all 18 moves to get neighbors at depth d+1.
//  Since we work on a sub-problem, we only index the relevant
//  part (corners or one edge group), ignoring the rest.
//
//  For corners: we just track corner permutation + orientation.
//  For edge groups: we track 6-of-12 edge positions + orientations.
//    The other 6 edges are "don't care" — we apply moves to the
//    full state but only index the tracked edges.
// ============================================================

void PatternDB::generate(Type type) {
    type_ = type;

    switch (type) {
        case CORNERS:      size_ = N_CORNER_STATES; break;
        case EDGE_GROUP_0: size_ = N_EDGE_STATES;   break;
        case EDGE_GROUP_1: size_ = N_EDGE_STATES;   break;
    }

    // Allocate nibble-packed storage, initialize to 0xFF (= 15, "unvisited")
    data_.assign((size_ + 1) / 2, 0xFF);

    std::cout << "Generating PDB type " << type << " (" << size_ << " states)...\n";
    auto t0 = std::chrono::steady_clock::now();

    // Start BFS from solved state
    CubeState solved = CubeState::solved();
    uint32_t solved_idx = index(solved);
    set_val(solved_idx, 0);

    // BFS using "frontier" approach (no explicit queue of states):
    // For each depth d, scan the full table and expand states at depth d.
    // This is memory-efficient (no queue), though slower than a real BFS queue.
    // For production, a proper BFS queue with encoded states is faster,
    // but this approach is simpler and works fine.

    // Actually, let's do proper BFS with a queue for reasonable speed.
    // We enqueue just the index (uint32_t), and unrank when expanding.

    // For corner PDB, we can reconstruct the state from the index.
    // For edge PDB, same — unrank gives us the edge positions.

    std::queue<uint32_t> current, next;
    current.push(solved_idx);

    uint64_t total_visited = 1;
    int depth = 0;

    while (!current.empty()) {
        std::cout << "  Depth " << depth << ": " << current.size() << " states"
                  << " (total: " << total_visited << ")\n";

        while (!current.empty()) {
            uint32_t idx = current.front();
            current.pop();

            // Reconstruct state from index
            CubeState state;
            if (type == CORNERS) {
                corner_unrank(idx, state);
            } else {
                int group = (type == EDGE_GROUP_0) ? 0 : 1;
                edge_unrank(idx, group, state);
            }

            // Try all 18 moves
            for (int m = 0; m < 18; m++) {
                CubeState neighbor = state;
                neighbor.apply_move(static_cast<Move>(m));

                uint32_t n_idx = index(neighbor);
                if (get_val(n_idx) == 15) { // unvisited
                    set_val(n_idx, depth + 1);
                    next.push(n_idx);
                    total_visited++;
                }
            }
        }

        std::swap(current, next);
        depth++;

        if (depth >= 15) break; // safety (max depth fits in nibble)
    }

    auto t1 = std::chrono::steady_clock::now();
    double secs = std::chrono::duration<double>(t1 - t0).count();
    std::cout << "  Done! " << total_visited << " states in "
              << secs << "s, max depth = " << (depth - 1) << "\n";
}

// ============================================================
//  Save / Load
// ============================================================

bool PatternDB::save(const std::string& filename) const {
    std::ofstream ofs(filename, std::ios::binary);
    if (!ofs) return false;

    // Header: type, size
    ofs.write(reinterpret_cast<const char*>(&type_), sizeof(type_));
    ofs.write(reinterpret_cast<const char*>(&size_), sizeof(size_));
    ofs.write(reinterpret_cast<const char*>(data_.data()), data_.size());

    std::cout << "Saved PDB to " << filename << " (" << data_.size() << " bytes)\n";
    return ofs.good();
}

bool PatternDB::load(const std::string& filename) {
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs) return false;

    ifs.read(reinterpret_cast<char*>(&type_), sizeof(type_));
    ifs.read(reinterpret_cast<char*>(&size_), sizeof(size_));

    data_.resize((size_ + 1) / 2);
    ifs.read(reinterpret_cast<char*>(data_.data()), data_.size());

    std::cout << "Loaded PDB from " << filename << " (" << data_.size() << " bytes)\n";
    return ifs.good();
}

// ============================================================
//  Heuristic: max of 3 PDBs
// ============================================================

void Heuristic::init(const std::string& dir) {
    auto load_or_gen = [&](PatternDB& pdb, PatternDB::Type type, const std::string& name) {
        std::string path = dir + "/" + name;
        if (pdb.load(path)) return;
        pdb.generate(type);
        pdb.save(path);
    };

    load_or_gen(corner_pdb_, PatternDB::CORNERS,      "corner.pdb");
    load_or_gen(edge0_pdb_,  PatternDB::EDGE_GROUP_0,  "edge0.pdb");
    load_or_gen(edge1_pdb_,  PatternDB::EDGE_GROUP_1,  "edge1.pdb");
    ready_ = true;
}

uint8_t Heuristic::operator()(const CubeState& s) const {
    uint8_t h = corner_pdb_.lookup(s);
    h = std::max(h, edge0_pdb_.lookup(s));
    h = std::max(h, edge1_pdb_.lookup(s));
    return h;
}

// ============================================================
//  Simple Heuristic
//
//  Three admissible components, take the max:
//  1. ceil(misplaced_corners / 4) — each move displaces 4 corners
//  2. ceil(misplaced_edges / 4)   — each move displaces 4 edges
//  3. ceil(total_corner_twist / 4) — a quarter turn twists ≤4 corners
//     (each twist ≠ 0 needs at least 1 move contribution)
//
//  This is weak (~2-4 for a typical scramble) but provably admissible,
//  so IDA* still finds optimal solutions — it just takes longer.
// ============================================================

uint8_t SimpleHeuristic::operator()(const CubeState& s) const {
    int bad_corners = 0, bad_edges = 0;
    int corner_twist = 0, edge_flip = 0;

    for (int i = 0; i < N_CORNERS; i++) {
        if (s.cp[i] != i || s.co[i] != 0) bad_corners++;
        if (s.co[i] != 0) corner_twist++;
    }
    for (int i = 0; i < N_EDGES; i++) {
        if (s.ep[i] != i || s.eo[i] != 0) bad_edges++;
        if (s.eo[i] != 0) edge_flip++;
    }

    int h1 = (bad_corners + 3) / 4;  // ceil div
    int h2 = (bad_edges + 3) / 4;
    int h3 = (corner_twist + 3) / 4;
    int h4 = (edge_flip + 3) / 4;

    return std::max({h1, h2, h3, h4});
}
