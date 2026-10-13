#pragma once
#include "cube.h"
#include <vector>
#include <string>
#include <cstdint>
#include <functional>

// --- Pattern Database with nibble-packed storage ---
class PatternDB {
public:
    enum Type { CORNER, EDGE0, EDGE1 };

    PatternDB(Type type);

    // Generate via BFS from solved state
    void generate();

    // Lookup depth for a cube state
    uint8_t lookup(const CubeState& s) const;

    // Save/load binary files
    bool save(const std::string& filename) const;
    bool load(const std::string& filename);

    uint32_t size() const { return size_; }
    bool is_generated() const { return generated_; }

private:
    Type type_;
    uint32_t size_;
    std::vector<uint8_t> data_;  // nibble-packed: 2 entries per byte
    bool generated_ = false;

    void set_val(uint32_t idx, uint8_t val);
    uint8_t get_val(uint32_t idx) const;

    // Index function for this PDB type
    uint32_t index(const CubeState& s) const;

    // Unrank function for BFS generation
    CubeState unrank(uint32_t idx) const;
};

// --- Combined PDB heuristic: max of all 3 PDBs ---
class Heuristic {
public:
    Heuristic();

    // Try to load PDBs from disk; returns true if all loaded
    bool load_from_disk(const std::string& dir = ".");

    // Generate all PDBs (slow, one-time)
    void generate_all();

    // Save all PDBs to disk
    void save_to_disk(const std::string& dir = ".") const;

    // Heuristic value: max of 3 PDB lookups
    uint8_t operator()(const CubeState& s) const;

    bool ready() const { return ready_; }

private:
    PatternDB corner_pdb_;
    PatternDB edge0_pdb_;
    PatternDB edge1_pdb_;
    bool ready_ = false;
};

// --- Simple admissible heuristic (no precomputation, weaker) ---
class SimpleHeuristic {
public:
    uint8_t operator()(const CubeState& s) const;
};
