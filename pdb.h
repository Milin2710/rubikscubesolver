#pragma once
#include "cube.h"
#include <string>
#include <vector>
#include <cstdint>

// ============================================================
//  Pattern Database
//
//  Stores the minimum number of moves to solve a sub-problem
//  (corners only, or a group of 6 edges) from any configuration.
//
//  Built via reverse BFS from the solved state.
//  Values are stored as nibbles (4 bits each, max 15) to save memory.
// ============================================================

class PatternDB {
public:
    // Type of PDB
    enum Type { CORNERS, EDGE_GROUP_0, EDGE_GROUP_1 };

    PatternDB() = default;

    // Generate by BFS (takes a few minutes for corners, ~1 min for edges)
    void generate(Type type);

    // Look up the heuristic value for a cube state
    uint8_t lookup(const CubeState& s) const;

    // Save / load to disk
    bool save(const std::string& filename) const;
    bool load(const std::string& filename);

    // Info
    Type   get_type()  const { return type_; }
    size_t get_size()  const { return size_; }
    size_t get_bytes() const { return data_.size(); }
    bool   is_ready()  const { return !data_.empty(); }

private:
    Type type_;
    uint32_t size_ = 0;            // number of entries
    std::vector<uint8_t> data_;    // nibble-packed: 2 entries per byte

    void set_val(uint32_t idx, uint8_t val);
    uint8_t get_val(uint32_t idx) const;

    // Index a state for this PDB type
    uint32_t index(const CubeState& s) const;
};

// ============================================================
//  Combined Heuristic: max of all loaded PDBs
// ============================================================

class Heuristic {
public:
    // Load or generate all 3 PDBs. directory = where to cache files.
    void init(const std::string& directory = ".");

    // h(state) = max of all PDB lookups (admissible)
    uint8_t operator()(const CubeState& s) const;

    bool is_ready() const { return ready_; }

private:
    bool ready_ = false;
    PatternDB corner_pdb_;
    PatternDB edge0_pdb_;
    PatternDB edge1_pdb_;
};

// ============================================================
//  Simple Heuristic (no PDBs needed, weaker but instant)
//
//  Uses corner + edge misplacement and orientation counts.
//  Admissible but weak — good for testing IDA* on short scrambles.
// ============================================================

class SimpleHeuristic {
public:
    uint8_t operator()(const CubeState& s) const;
};
