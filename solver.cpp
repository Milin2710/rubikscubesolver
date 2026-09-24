#include "solver.h"
#include <iostream>
#include <chrono>
#include <algorithm>

// ============================================================
//  Move Pruning
//
//  Faces: 0=U, 1=D, 2=R, 3=L, 4=F, 5=B
//  Opposite pairs: (U,D)=(0,1), (R,L)=(2,3), (F,B)=(4,5)
//
//  Rules:
//  1. Never apply same face twice in a row (redundant with single move)
//  2. For opposite faces, enforce the lower-index face first.
//     e.g., U-then-D is OK, D-then-U is pruned (they commute).
// ============================================================

bool IDASolver::is_move_redundant(int mf, int last_face) {
    if (last_face < 0) return false;           // no previous move
    if (mf == last_face) return true;           // same face
    // Opposite faces: (0,1), (2,3), (4,5). They share the same pair index.
    if ((mf / 2 == last_face / 2) && mf > last_face) return true;
    return false;
}

// ============================================================
//  IDA* Search
// ============================================================

uint8_t IDASolver::search(CubeState& state, uint8_t g, uint8_t threshold, int last_face) {
    uint8_t h = heuristic_(state);
    uint8_t f = g + h;

    if (f > threshold) return f;
    if (h == 0 && state.is_solved()) return 0; // FOUND

    if (node_limit_ > 0 && nodes_ >= node_limit_) return 255;

    uint8_t min_exceeded = 255;

    for (int m = 0; m < 18; m++) {
        int mf = m / 3;
        if (is_move_redundant(mf, last_face)) continue;

        nodes_++;

        // Apply move
        CubeState child = state;
        child.apply_move(static_cast<Move>(m));

        path_.push_back(static_cast<Move>(m));
        uint8_t result = search(child, g + 1, threshold, mf);

        if (result == 0) return 0;   // solution found (path_ has it)
        if (result == 255) return 255; // aborted

        if (result < min_exceeded) min_exceeded = result;
        path_.pop_back();
    }

    return min_exceeded;
}

SolverResult IDASolver::solve(const CubeState& start) {
    SolverResult result;
    auto t0 = std::chrono::steady_clock::now();

    if (start.is_solved()) {
        result.found = true;
        result.depth = 0;
        result.time_seconds = 0;
        return result;
    }

    nodes_ = 0;
    path_.clear();

    uint8_t threshold = heuristic_(start);
    std::cout << "IDA* starting, initial h = " << (int)threshold << "\n";

    while (threshold < 30) { // God's number is 20; 30 is generous safety margin
        std::cout << "  Threshold = " << (int)threshold << ", nodes so far = " << nodes_ << "\n";

        CubeState state = start;
        uint8_t t = search(state, 0, threshold, -1);

        if (t == 0) {
            // Solution found
            auto t1 = std::chrono::steady_clock::now();
            result.found = true;
            result.depth = path_.size();
            result.solution = path_;
            result.nodes_expanded = nodes_;
            result.time_seconds = std::chrono::duration<double>(t1 - t0).count();

            std::cout << "  SOLVED in " << result.depth << " moves, "
                      << nodes_ << " nodes, "
                      << result.time_seconds << "s\n";
            return result;
        }

        if (t == 255) {
            std::cout << "  Aborted (node limit reached)\n";
            break;
        }

        threshold = t;
    }

    auto t1 = std::chrono::steady_clock::now();
    result.found = false;
    result.nodes_expanded = nodes_;
    result.time_seconds = std::chrono::duration<double>(t1 - t0).count();
    return result;
}
