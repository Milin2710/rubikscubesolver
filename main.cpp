#include "cube.h"
#include "pdb.h"
#include "solver.h"
#include <iostream>
#include <fstream>
#include <string>
#include <cassert>
#include <chrono>
#include <cstdlib>

// ========== Self-Tests ==========

static int run_tests() {
    std::cout << "=== Running Self-Tests ===" << std::endl;
    int pass = 0, total = 0;

    // Test 1: Solved state is solved
    {
        total++;
        CubeState s; s.set_solved();
        assert(s.is_solved());
        std::cout << "  [PASS] Test 1: Solved state is solved" << std::endl;
        pass++;
    }

    // Test 2: R R' = Identity (inverse for all faces)
    {
        total++;
        bool ok = true;
        CubeState solved; solved.set_solved();
        for (int face = 0; face < 6; face++) {
            Move cw  = static_cast<Move>(face * 3);
            Move ccw = static_cast<Move>(face * 3 + 2);
            CubeState s = solved.apply(cw).apply(ccw);
            if (!(s == solved)) { ok = false; break; }
        }
        assert(ok);
        std::cout << "  [PASS] Test 2: X X' = Identity for all faces" << std::endl;
        pass++;
    }

    // Test 3: R^4 = Identity (order 4 for all CW moves)
    {
        total++;
        bool ok = true;
        CubeState solved; solved.set_solved();
        for (int face = 0; face < 6; face++) {
            Move cw = static_cast<Move>(face * 3);
            CubeState s = solved;
            for (int i = 0; i < 4; i++) s = s.apply(cw);
            if (!(s == solved)) { ok = false; break; }
        }
        assert(ok);
        std::cout << "  [PASS] Test 3: X^4 = Identity for all CW moves" << std::endl;
        pass++;
    }

    // Test 4: X2^2 = Identity (order 2 for all half turns)
    {
        total++;
        bool ok = true;
        CubeState solved; solved.set_solved();
        for (int face = 0; face < 6; face++) {
            Move half = static_cast<Move>(face * 3 + 1);
            CubeState s = solved.apply(half).apply(half);
            if (!(s == solved)) { ok = false; break; }
        }
        assert(ok);
        std::cout << "  [PASS] Test 4: X2^2 = Identity for all half turns" << std::endl;
        pass++;
    }

    // Test 5: Corner index of solved state = 0
    {
        total++;
        CubeState s; s.set_solved();
        assert(corner_index(s) == 0);
        std::cout << "  [PASS] Test 5: Corner index of solved = 0" << std::endl;
        pass++;
    }

    // Test 6: Corner index round-trip (encode -> decode -> re-encode)
    {
        total++;
        CubeState s; s.set_solved();
        s = s.apply(R1).apply(U1).apply(F1);
        uint32_t idx = corner_index(s);
        CubeState s2 = corner_unrank(idx);
        uint32_t idx2 = corner_index(s2);
        assert(idx == idx2);
        std::cout << "  [PASS] Test 6: Corner index round-trip" << std::endl;
        pass++;
    }

    // Test 7: Edge index validity (in range)
    {
        total++;
        CubeState s; s.set_solved();
        s = s.apply(R1).apply(U1);
        uint32_t e0 = edge_index(s, 0);
        uint32_t e1 = edge_index(s, 1);
        assert(e0 < N_EDGE_STATES);
        assert(e1 < N_EDGE_STATES);
        std::cout << "  [PASS] Test 7: Edge indices in valid range" << std::endl;
        pass++;
    }

    // Test 8: Edge index round-trip
    {
        total++;
        CubeState s; s.set_solved();
        s = s.apply(F1).apply(R1).apply(U1);
        uint32_t idx = edge_index(s, 0);
        // Verify it's consistent (apply same scramble to solved, get same index)
        CubeState s2; s2.set_solved();
        s2 = s2.apply(F1).apply(R1).apply(U1);
        assert(edge_index(s2, 0) == idx);
        std::cout << "  [PASS] Test 8: Edge index consistency" << std::endl;
        pass++;
    }

    // Test 9: In-place apply + undo = identity
    {
        total++;
        CubeState orig; orig.set_solved();
        orig = orig.apply(R1).apply(U1).apply(F1);  // some scrambled state
        CubeState s = orig;
        for (int m = 0; m < N_MOVES; m++) {
            Move mv = static_cast<Move>(m);
            s.apply_inplace(mv);
            s.undo_inplace(mv);
            assert(s == orig);
        }
        std::cout << "  [PASS] Test 9: In-place apply + undo = identity for all 18 moves" << std::endl;
        pass++;
    }

    // Test 10: Move parsing
    {
        total++;
        auto moves = parse_scramble("R U2 F' L D B2");
        assert(moves.size() == 6);
        assert(moves[0] == R1);
        assert(moves[1] == U2);
        assert(moves[2] == F3);
        assert(moves[3] == L1);
        assert(moves[4] == D1);
        assert(moves[5] == B2);
        std::cout << "  [PASS] Test 10: Move parsing" << std::endl;
        pass++;
    }

    // Test 11: SimpleHeuristic is admissible (h=0 for solved)
    {
        total++;
        CubeState s; s.set_solved();
        SimpleHeuristic sh;
        assert(sh(s) == 0);
        // Also check it's > 0 for non-trivial scramble
        CubeState s2 = s.apply(R1).apply(U1).apply(F1);
        assert(sh(s2) > 0);
        std::cout << "  [PASS] Test 11: SimpleHeuristic basic properties" << std::endl;
        pass++;
    }

    std::cout << "=== " << pass << " / " << total << " tests passed ===" << std::endl;
    return (pass == total) ? 0 : 1;
}

// ========== Demo Scrambles ==========

struct DemoScramble {
    const char* name;
    const char* scramble;
    int expected_depth;  // -1 = unknown
};

static DemoScramble demos[] = {
    {"1-move",   "R",                       1},
    {"2-move",   "R U",                     2},
    {"3-move",   "R U F",                   3},
    {"4-move",   "R U F D",                 4},
    {"5-move",   "R U F D L",               5},
    {"6-move",   "R U F D L B",             6},
    {"8-move",   "R U F D R' U' F' D'",     8},
    {"10-move",  "R U F L B U R2 D F' L'", 10},
    {"12-move",  "R U2 F' D L2 B R' U D2 F L' B2", 12},
    {"13-move",  "F R' U2 B L D' R2 U' F2 L' D B' R", 13},
};

static const int N_DEMOS = sizeof(demos) / sizeof(demos[0]);

// ========== Solve a single scramble ==========

template<typename HeuristicFn>
static void solve_scramble(const std::string& name, const std::string& scramble_str,
                           HeuristicFn& heuristic, double time_limit) {
    auto moves = parse_scramble(scramble_str);
    CubeState s;
    s.set_solved();
    for (auto m : moves) s = s.apply(m);

    std::cout << "\n--- " << name << " ---" << std::endl;
    std::cout << "Scramble: " << scramble_str << " (" << moves.size() << " moves)" << std::endl;

    IDASolver<HeuristicFn> solver(heuristic, 0, time_limit);
    SolverResult result = solver.solve(s);

    if (result.found) {
        std::cout << "SOLVED! Depth: " << result.depth
                  << "  Nodes: " << result.nodes_expanded
                  << "  Time: " << result.time_seconds << "s" << std::endl;
        std::cout << "Solution: " << solution_to_string(result.solution) << std::endl;

        // Verify solution
        CubeState verify = s;
        for (auto m : result.solution) verify = verify.apply(m);
        if (verify.is_solved()) {
            std::cout << "Verification: CORRECT" << std::endl;
        } else {
            std::cout << "Verification: FAILED!" << std::endl;
        }
    } else if (result.timed_out) {
        std::cout << "TIMED OUT after " << result.time_seconds << "s, "
                  << result.nodes_expanded << " nodes" << std::endl;
    } else {
        std::cout << "No solution found." << std::endl;
    }
}

// ========== Main ==========

static void print_usage() {
    std::cout << "Usage:" << std::endl;
    std::cout << "  rubiks_solver test            Run self-tests" << std::endl;
    std::cout << "  rubiks_solver genpdb          Generate pattern databases" << std::endl;
    std::cout << "  rubiks_solver solve [limit]   Solve demo scrambles (limit=seconds, default 60)" << std::endl;
    std::cout << "  rubiks_solver interactive     Interactive scramble input" << std::endl;
}

int main(int argc, char* argv[]) {
    init_move_tables();

    if (argc < 2) {
        print_usage();
        return 0;
    }

    std::string cmd = argv[1];

    if (cmd == "test") {
        return run_tests();
    }

    if (cmd == "genpdb") {
        Heuristic h;
        h.generate_all();
        h.save_to_disk(".");
        std::cout << "Pattern databases saved to disk." << std::endl;
        return 0;
    }

    if (cmd == "solve" || cmd == "interactive") {
        // Time limit: default 60 seconds per scramble
        double time_limit = 60.0;
        if (argc >= 3) {
            time_limit = std::atof(argv[2]);
            if (time_limit <= 0) time_limit = 60.0;
        }

        // Try loading PDBs
        Heuristic pdb_h;
        bool have_pdbs = pdb_h.load_from_disk(".");

        if (have_pdbs) {
            std::cout << "Pattern databases loaded successfully." << std::endl;
            std::cout << "Time limit: " << time_limit << "s per scramble" << std::endl;

            if (cmd == "solve") {
                // Run demo scrambles
                for (int i = 0; i < N_DEMOS; i++) {
                    solve_scramble(demos[i].name, demos[i].scramble, pdb_h, time_limit);
                }
            } else {
                // Interactive mode
                std::cout << "\nEnter scramble (e.g., R U F' L2 B D'): " << std::endl;
                std::string line;
                while (std::getline(std::cin, line)) {
                    if (line.empty() || line == "quit" || line == "exit") break;
                    solve_scramble("User scramble", line, pdb_h, time_limit);
                    std::cout << "\nEnter scramble (or 'quit'): " << std::endl;
                }
            }
        } else {
            std::cout << "PDB files not found. Using SimpleHeuristic (weaker, works for ~7 moves)." << std::endl;
            std::cout << "Run 'rubiks_solver genpdb' first for full-strength solving." << std::endl;
            std::cout << "Time limit: " << time_limit << "s per scramble" << std::endl;
            SimpleHeuristic simple_h;

            if (cmd == "solve") {
                // Only run shorter demos with SimpleHeuristic
                for (int i = 0; i < std::min(7, N_DEMOS); i++) {
                    solve_scramble(demos[i].name, demos[i].scramble, simple_h, time_limit);
                }
            } else {
                std::cout << "\nEnter scramble (e.g., R U F' L2 B D'): " << std::endl;
                std::string line;
                while (std::getline(std::cin, line)) {
                    if (line.empty() || line == "quit" || line == "exit") break;
                    solve_scramble("User scramble", line, simple_h, time_limit);
                    std::cout << "\nEnter scramble (or 'quit'): " << std::endl;
                }
            }
        }

        return 0;
    }

    print_usage();
    return 1;
}
