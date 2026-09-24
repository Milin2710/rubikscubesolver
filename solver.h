#pragma once
#include "cube.h"
#include "pdb.h"
#include <vector>
#include <cstdint>
#include <functional>

// ============================================================
//  IDA* Solver (Korf's Algorithm)
//
//  Iterative Deepening A*: runs depth-limited DFS with
//  f(n) = g(n) + h(n) as the bound. Each iteration increases
//  the threshold to the minimum f-value that exceeded it.
//
//  Move pruning:
//    - Don't apply the same face consecutively (e.g., R after R2)
//    - For opposite faces (U/D, R/L, F/B), enforce ordering to
//      avoid equivalent sequences (e.g., always U before D)
// ============================================================

struct SolverResult {
    bool        found = false;
    int         depth = 0;
    uint64_t    nodes_expanded = 0;
    double      time_seconds = 0;
    std::vector<Move> solution;
};

// Heuristic is a callable: CubeState → uint8_t
using HeuristicFn = std::function<uint8_t(const CubeState&)>;

class IDASolver {
public:
    explicit IDASolver(HeuristicFn h) : heuristic_(std::move(h)) {}

    // Solve a scrambled cube. Returns the optimal solution.
    SolverResult solve(const CubeState& start);

    // Set a node limit (0 = no limit)
    void set_node_limit(uint64_t limit) { node_limit_ = limit; }

private:
    HeuristicFn heuristic_;
    uint64_t node_limit_ = 0;
    uint64_t nodes_ = 0;

    // Path being explored
    std::vector<Move> path_;

    uint8_t search(CubeState& state, uint8_t g, uint8_t threshold, int last_face);
    static bool is_move_redundant(int move_face, int last_face);
};
