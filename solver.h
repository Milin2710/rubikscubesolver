#pragma once
#include "cube.h"
#include <vector>
#include <cstdint>
#include <functional>
#include <chrono>
#include <iostream>

struct SolverResult {
    bool found = false;
    bool timed_out = false;
    int depth = -1;
    uint64_t nodes_expanded = 0;
    double time_seconds = 0.0;
    std::vector<Move> solution;
};

// IDA* solver with in-place move application (no CubeState copies during search)
// and configurable time limit to prevent hanging on deep scrambles.
template<typename HeuristicFn>
class IDASolver {
public:
    IDASolver(HeuristicFn h, uint64_t node_limit = 0, double time_limit_sec = 0.0)
        : heuristic_(h), node_limit_(node_limit), time_limit_sec_(time_limit_sec) {}

    SolverResult solve(const CubeState& start);

private:
    HeuristicFn heuristic_;
    uint64_t node_limit_;
    double time_limit_sec_;  // 0 = no limit

    // Search state (avoids passing through recursion)
    CubeState state_;           // single mutable state (apply/undo in-place)
    std::vector<Move> path_;    // current path from root
    uint64_t nodes_;
    bool aborted_;
    std::chrono::steady_clock::time_point start_time_;

    // Returns: 0 = found, f-value = exceeded threshold, 255 = aborted
    uint8_t dfs(int g, uint8_t threshold, int last_face);

    bool check_limits();
};

// ========== Template implementation ==========

template<typename HeuristicFn>
SolverResult IDASolver<HeuristicFn>::solve(const CubeState& start) {
    SolverResult result;
    start_time_ = std::chrono::steady_clock::now();

    state_ = start;
    path_.clear();
    nodes_ = 0;
    aborted_ = false;

    uint8_t h0 = heuristic_(start);
    if (h0 == 0 && start.is_solved()) {
        result.found = true;
        result.depth = 0;
        result.nodes_expanded = 0;
        auto end = std::chrono::steady_clock::now();
        result.time_seconds = std::chrono::duration<double>(end - start_time_).count();
        return result;
    }

    uint8_t threshold = h0;
    std::cout << "  Initial h = " << (int)h0 << std::endl;

    while (threshold < 30) {  // God's number is 20 in half-turn metric
        nodes_ = 0;
        aborted_ = false;

        auto iter_start = std::chrono::steady_clock::now();
        uint8_t t = dfs(0, threshold, -1);
        auto iter_end = std::chrono::steady_clock::now();
        double iter_time = std::chrono::duration<double>(iter_end - iter_start).count();

        std::cout << "  Threshold " << (int)threshold
                  << ": " << nodes_ << " nodes, "
                  << iter_time << "s" << std::endl;

        if (t == 0) {
            // Found
            result.found = true;
            result.depth = path_.size();
            result.solution = path_;
            result.nodes_expanded = nodes_;
            auto end = std::chrono::steady_clock::now();
            result.time_seconds = std::chrono::duration<double>(end - start_time_).count();
            return result;
        }

        if (aborted_ || t == 255) {
            // Hit limit
            auto end = std::chrono::steady_clock::now();
            result.time_seconds = std::chrono::duration<double>(end - start_time_).count();
            result.nodes_expanded = nodes_;
            result.timed_out = true;
            std::cout << "  Search aborted (";
            if (time_limit_sec_ > 0) std::cout << "time limit " << time_limit_sec_ << "s";
            if (node_limit_ > 0) std::cout << "node limit " << node_limit_;
            std::cout << ")" << std::endl;
            return result;
        }

        threshold = t;
    }

    return result;
}

template<typename HeuristicFn>
bool IDASolver<HeuristicFn>::check_limits() {
    if (node_limit_ > 0 && nodes_ >= node_limit_) return true;
    if (time_limit_sec_ > 0) {
        // Check time every 100K nodes to avoid clock overhead
        if (nodes_ % 100000 == 0) {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - start_time_).count();
            if (elapsed >= time_limit_sec_) return true;
        }
    }
    return false;
}

template<typename HeuristicFn>
uint8_t IDASolver<HeuristicFn>::dfs(int g, uint8_t threshold, int last_face) {
    uint8_t h = heuristic_(state_);
    uint8_t f = g + h;

    if (f > threshold) return f;

    if (h == 0 && state_.is_solved()) return 0;  // found!

    if (check_limits()) { aborted_ = true; return 255; }

    uint8_t min_exceeded = 255;

    for (int m = 0; m < N_MOVES; m++) {
        int mf = move_face(static_cast<Move>(m));
        if (is_move_redundant(mf, last_face)) continue;

        nodes_++;
        Move mv = static_cast<Move>(m);

        // Apply move IN-PLACE (no copy!)
        state_.apply_inplace(mv);
        path_.push_back(mv);

        uint8_t t = dfs(g + 1, threshold, mf);

        if (t == 0) return 0;  // Found! Keep path intact

        // Undo move (restore state) — only on non-solution branches
        path_.pop_back();
        state_.undo_inplace(mv);

        if (aborted_) return 255;
        if (t < min_exceeded) min_exceeded = t;
    }

    return min_exceeded;
}
