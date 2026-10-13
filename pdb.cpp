#include "pdb.h"
#include <queue>
#include <iostream>
#include <fstream>
#include <cstring>
#include <algorithm>
#include <chrono>

// ========== PatternDB ==========

PatternDB::PatternDB(Type type) : type_(type) {
    if (type == CORNER) size_ = N_CORNER_STATES;
    else                size_ = N_EDGE_STATES;
}

void PatternDB::set_val(uint32_t idx, uint8_t val) {
    uint32_t byte_idx = idx / 2;
    if (idx % 2 == 0)
        data_[byte_idx] = (data_[byte_idx] & 0xF0) | (val & 0x0F);
    else
        data_[byte_idx] = (data_[byte_idx] & 0x0F) | ((val & 0x0F) << 4);
}

uint8_t PatternDB::get_val(uint32_t idx) const {
    uint32_t byte_idx = idx / 2;
    if (idx % 2 == 0) return data_[byte_idx] & 0x0F;
    else              return (data_[byte_idx] >> 4) & 0x0F;
}

uint32_t PatternDB::index(const CubeState& s) const {
    if (type_ == CORNER) return corner_index(s);
    if (type_ == EDGE0)  return edge_index(s, 0);
    return edge_index(s, 1);
}

CubeState PatternDB::unrank(uint32_t idx) const {
    if (type_ == CORNER) return corner_unrank(idx);
    if (type_ == EDGE0)  return edge_unrank(idx, 0);
    return edge_unrank(idx, 1);
}

void PatternDB::generate() {
    auto t0 = std::chrono::steady_clock::now();

    uint32_t bytes = (size_ + 1) / 2;
    data_.assign(bytes, 0xFF);  // 0xFF = both nibbles = 15 (unvisited)

    // BFS from solved state
    CubeState solved;
    solved.set_solved();
    uint32_t solved_idx = index(solved);
    set_val(solved_idx, 0);

    std::queue<uint32_t> q;
    q.push(solved_idx);

    uint64_t visited = 1;
    uint8_t current_depth = 0;

    std::cout << "  Generating PDB (" << size_ << " states)..." << std::endl;

    while (!q.empty()) {
        size_t level_size = q.size();
        for (size_t i = 0; i < level_size; i++) {
            uint32_t idx = q.front(); q.pop();
            uint8_t depth = get_val(idx);

            if (depth != current_depth) {
                // We've moved to the next depth — print progress
                current_depth = depth;
            }

            CubeState s = unrank(idx);

            for (int m = 0; m < N_MOVES; m++) {
                CubeState ns = s.apply(static_cast<Move>(m));
                uint32_t nidx = index(ns);
                if (get_val(nidx) == 15) {  // unvisited
                    set_val(nidx, depth + 1);
                    q.push(nidx);
                    visited++;
                }
            }
        }
        auto t1 = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(t1 - t0).count();
        std::cout << "    Depth " << (int)current_depth
                  << ": " << visited << " states visited ("
                  << (100.0 * visited / size_) << "%, "
                  << elapsed << "s)" << std::endl;
    }

    generated_ = true;
    auto t1 = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(t1 - t0).count();
    std::cout << "  Done: " << visited << " / " << size_
              << " states in " << elapsed << "s" << std::endl;
}

uint8_t PatternDB::lookup(const CubeState& s) const {
    if (!generated_) return 0;
    uint32_t idx = index(s);
    uint8_t val = get_val(idx);
    return (val == 15) ? 0 : val;  // 15 = unreachable, treat as 0
}

bool PatternDB::save(const std::string& filename) const {
    if (!generated_) return false;
    std::ofstream f(filename, std::ios::binary);
    if (!f) return false;

    // Header: type (1 byte) + size (4 bytes)
    uint8_t t = static_cast<uint8_t>(type_);
    f.write(reinterpret_cast<const char*>(&t), 1);
    f.write(reinterpret_cast<const char*>(&size_), 4);
    f.write(reinterpret_cast<const char*>(data_.data()), data_.size());
    return f.good();
}

bool PatternDB::load(const std::string& filename) {
    std::ifstream f(filename, std::ios::binary);
    if (!f) return false;

    uint8_t t;
    uint32_t sz;
    f.read(reinterpret_cast<char*>(&t), 1);
    f.read(reinterpret_cast<char*>(&sz), 4);

    if (t != static_cast<uint8_t>(type_) || sz != size_) return false;

    uint32_t bytes = (size_ + 1) / 2;
    data_.resize(bytes);
    f.read(reinterpret_cast<char*>(data_.data()), bytes);

    if (f.good()) {
        generated_ = true;
        return true;
    }
    return false;
}

// ========== Combined Heuristic ==========

Heuristic::Heuristic()
    : corner_pdb_(PatternDB::CORNER),
      edge0_pdb_(PatternDB::EDGE0),
      edge1_pdb_(PatternDB::EDGE1) {}

bool Heuristic::load_from_disk(const std::string& dir) {
    bool ok = true;
    ok &= corner_pdb_.load(dir + "/corners.pdb");
    ok &= edge0_pdb_.load(dir + "/edges0.pdb");
    ok &= edge1_pdb_.load(dir + "/edges1.pdb");
    ready_ = ok;
    return ok;
}

void Heuristic::generate_all() {
    std::cout << "Generating Corner PDB..." << std::endl;
    corner_pdb_.generate();
    std::cout << "Generating Edge Group 0 PDB..." << std::endl;
    edge0_pdb_.generate();
    std::cout << "Generating Edge Group 1 PDB..." << std::endl;
    edge1_pdb_.generate();
    ready_ = true;
}

void Heuristic::save_to_disk(const std::string& dir) const {
    corner_pdb_.save(dir + "/corners.pdb");
    edge0_pdb_.save(dir + "/edges0.pdb");
    edge1_pdb_.save(dir + "/edges1.pdb");
}

uint8_t Heuristic::operator()(const CubeState& s) const {
    uint8_t h1 = corner_pdb_.lookup(s);
    uint8_t h2 = edge0_pdb_.lookup(s);
    uint8_t h3 = edge1_pdb_.lookup(s);
    return std::max({h1, h2, h3});
}

// ========== Simple Heuristic ==========

uint8_t SimpleHeuristic::operator()(const CubeState& s) const {
    int bad_cp = 0, bad_co = 0, bad_ep = 0, bad_eo = 0;
    for (int i = 0; i < N_CORNERS; i++) {
        if (s.cp[i] != i) bad_cp++;
        if (s.co[i] != 0) bad_co++;
    }
    for (int i = 0; i < N_EDGES; i++) {
        if (s.ep[i] != i) bad_ep++;
        if (s.eo[i] != 0) bad_eo++;
    }
    // Each move displaces at most 4 cubies in each category
    int h1 = (bad_cp + 3) / 4;
    int h2 = (bad_ep + 3) / 4;
    int h3 = (bad_co + 3) / 4;
    int h4 = (bad_eo + 3) / 4;
    return static_cast<uint8_t>(std::max({h1, h2, h3, h4}));
}
