#include "cube.h"
#include "pdb.h"
#include "solver.h"
#include <iostream>
#include <fstream>
#include <string>
#include <cassert>

// ============================================================
//  Self-tests for cube mechanics
// ============================================================
static void run_tests() {
    std::cout << "=== Running self-tests ===\n";

    // Test 1: Solved state is solved
    {
        CubeState s = CubeState::solved();
        assert(s.is_solved());
        std::cout << "[PASS] Solved state identity\n";
    }

    // Test 2: R R' = identity
    {
        CubeState s = CubeState::solved();
        s.apply_move(R1);
        assert(!s.is_solved());
        s.apply_move(R3); // R' = R inverse
        assert(s.is_solved());
        std::cout << "[PASS] R R' = identity\n";
    }

    // Test 3: R4 = identity
    {
        CubeState s = CubeState::solved();
        for (int i = 0; i < 4; i++) s.apply_move(R1);
        assert(s.is_solved());
        std::cout << "[PASS] R^4 = identity\n";
    }

    // Test 4: U4 = identity
    {
        CubeState s = CubeState::solved();
        for (int i = 0; i < 4; i++) s.apply_move(U1);
        assert(s.is_solved());
        std::cout << "[PASS] U^4 = identity\n";
    }

    // Test 5: All face^4 = identity
    {
        const Move cw_moves[] = {U1, D1, R1, L1, F1, B1};
        for (Move m : cw_moves) {
            CubeState s = CubeState::solved();
            for (int i = 0; i < 4; i++) s.apply_move(m);
            assert(s.is_solved());
        }
        std::cout << "[PASS] All face^4 = identity\n";
    }

    // Test 6: Half turn squared = identity
    {
        const Move h_moves[] = {U2, D2, R2, L2, F2, B2};
        for (Move m : h_moves) {
            CubeState s = CubeState::solved();
            s.apply_move(m);
            s.apply_move(m);
            assert(s.is_solved());
        }
        std::cout << "[PASS] All half-turn^2 = identity\n";
    }

    // Test 7: Superflip (known 20-move position) doesn't crash
    {
        CubeState s = CubeState::solved();
        auto moves = parse_moves("U R2 F B R B2 R U2 L B2 R U' D' R2 F R' L B2 U2 F2");
        s.apply_moves(moves);
        assert(!s.is_solved());
        std::cout << "[PASS] Superflip applied (20 moves)\n";
    }

    // Test 8: Corner index round-trip
    {
        CubeState s = CubeState::solved();
        s.apply_move(R1);
        s.apply_move(U1);
        uint32_t idx = corner_index(s);
        CubeState s2;
        corner_unrank(idx, s2);
        assert(corner_index(s2) == idx);
        std::cout << "[PASS] Corner index round-trip\n";
    }

    // Test 9: Solved state has corner index 0
    {
        CubeState s = CubeState::solved();
        assert(corner_index(s) == 0);
        std::cout << "[PASS] Solved corner index = 0\n";
    }

    // Test 10: Edge index consistency
    {
        CubeState s = CubeState::solved();
        // Group 0 (cubies 0-5 at positions 0-5): rank should be 0
        assert(edge_index(s, 0) == 0);
        // Group 1 (cubies 6-11 at positions 6-11): nonzero but consistent
        uint32_t idx1 = edge_index(s, 1);
        assert(idx1 < N_EDGE_STATES);
        std::cout << "[PASS] Edge indices valid (group0=0, group1=" << idx1 << ")\n";
    }

    // Test 11: Parse moves
    {
        auto moves = parse_moves("R U' F2");
        assert(moves.size() == 3);
        assert(moves[0] == R1);
        assert(moves[1] == U3);
        assert(moves[2] == F2);
        std::cout << "[PASS] Move parsing\n";
    }

    std::cout << "=== All tests passed! ===\n\n";
}

// ============================================================
//  Solve a scramble and print results
// ============================================================
static void solve_scramble(IDASolver& solver, const std::string& scramble_str) {
    auto moves = parse_moves(scramble_str);
    if (moves.empty()) {
        std::cout << "No valid moves. Use notation: R U' F2 D B2 L\n";
        return;
    }

    std::cout << "Scramble (" << moves.size() << " moves): ";
    for (Move m : moves) std::cout << move_name(m) << " ";
    std::cout << "\n";

    CubeState cube = CubeState::solved();
    cube.apply_moves(moves);

    std::cout << "Solving...\n";
    SolverResult result = solver.solve(cube);

    if (result.found) {
        std::cout << "\nSolution (" << result.depth << " moves): ";
        for (Move m : result.solution) std::cout << move_name(m) << " ";
        std::cout << "\n";
        std::cout << "Nodes: " << result.nodes_expanded
                  << ", Time: " << result.time_seconds << "s\n";

        // Verify
        CubeState verify = cube;
        verify.apply_moves(result.solution);
        std::cout << (verify.is_solved() ? "Solution VERIFIED.\n"
                                         : "WARNING: Verification FAILED!\n");
    } else {
        std::cout << "No solution found (node limit or too deep).\n";
    }
}

// ============================================================
//  Main
// ============================================================
int main(int argc, char* argv[]) {
    std::cout << "============================================\n";
    std::cout << "  Rubik's Cube Solver — Korf's IDA*\n";
    std::cout << "============================================\n\n";

    run_tests();

    std::string mode = (argc > 1) ? argv[1] : "solve";

    if (mode == "test") return 0;

    if (mode == "genpdb") {
        std::cout << "Generating pattern databases...\n";
        std::cout << "(This takes several minutes and ~170MB RAM)\n";
        std::cout << "Run on your own machine, not in a sandbox.\n\n";
        Heuristic h;
        h.init(".");
        std::cout << "\nAll PDBs generated and saved to .pdb files.\n";
        return 0;
    }

    if (mode == "solve") {
        // Try loading PDBs; fall back to simple heuristic
        HeuristicFn hfn;
        Heuristic pdb_heuristic;
        SimpleHeuristic simple_heuristic;

        bool use_pdb = false;
        // Check if PDB files exist
        {
            std::ifstream f("corner.pdb");
            if (f.good()) {
                std::cout << "PDB files found, loading...\n";
                pdb_heuristic.init(".");
                use_pdb = true;
                hfn = [&](const CubeState& s) { return pdb_heuristic(s); };
            }
        }

        if (!use_pdb) {
            std::cout << "No PDB files found. Using simple heuristic.\n";
            std::cout << "(Works for scrambles up to ~7-8 moves. For harder ones,\n";
            std::cout << " generate PDBs with: ./rubiks_solver genpdb)\n\n";
            hfn = [&](const CubeState& s) { return simple_heuristic(s); };
        }

        IDASolver solver(hfn);
        if (!use_pdb) solver.set_node_limit(50'000'000); // safety limit

        // Demo solves
        std::cout << "\n--- Demo Solves ---\n\n";

        std::cout << "[1] Scramble: R\n";
        solve_scramble(solver, "R");

        std::cout << "\n[2] Scramble: R U\n";
        solve_scramble(solver, "R U");

        std::cout << "\n[3] Scramble: R U F\n";
        solve_scramble(solver, "R U F");

        std::cout << "\n[4] Scramble: R U R' U'\n";
        solve_scramble(solver, "R U R' U'");

        std::cout << "\n[5] Scramble: F R U R' U' F'\n";
        solve_scramble(solver, "F R U R' U' F'");

        std::cout << "\n[6] Scramble: R U2 L' B D'\n";
        solve_scramble(solver, "R U2 L' B D'");

        // Interactive
        std::cout << "\n--- Interactive Mode (type 'quit' to exit) ---\n";
        std::string line;
        while (true) {
            std::cout << "\nScramble> ";
            if (!std::getline(std::cin, line)) break;
            if (line == "quit" || line == "exit") break;
            if (line.empty()) continue;
            solve_scramble(solver, line);
        }
        return 0;
    }

    std::cout << "Usage: " << argv[0] << " [test|genpdb|solve]\n";
    return 0;
}
