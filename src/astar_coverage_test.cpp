#include "game.hpp"
#include "game_util.hpp"
#include "shortest_path.hpp"
#include "cell_tree_agent.hpp" // Added for cell_tree_parents
#include <iostream>
#include <iomanip>
#include <random>

// Simple test to verify A* distance coverage vs BFS coverage
int main(int argc, char* argv[]) {
    int width = 10;
    int height = 10;
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--width" && i + 1 < argc) {
            width = std::stoi(argv[++i]);
        } else if (arg == "--height" && i + 1 < argc) {
            height = std::stoi(argv[++i]);
        }
    }
    
    // Create a game with proper constructor
    CoordRange dims{width, height};
    RNG rng; // Default constructor
    Game game(dims, rng, false);
    
    
    // Basic edge function with cell tree constraints for A*
    auto cell_parents = cell_tree_parents(game.dimensions(), game.snake);
    auto edge = [&](Coord a, Coord b, Dir dir) {
        if (can_move_in_cell_tree(cell_parents, a, b, dir) && !game.grid[b]) {
            return 1000; // Base cost
        } else {
            return INT_MAX;
        }
    };
    
    // Calculate distances using A* (as used in cell_tree_agent)
    auto astar_dists = astar_shortest_path(
        game.grid.coords(), 
        edge, 
        game.snake_pos(), 
        game.apple_pos,
        1000);
    
    // Calculate distances using BFS (the reliable method for all cells)
    auto bfs_dists = shortest_path(game.grid, game.snake_pos());
    
    // Count cells
    int total_cells = game.grid.w * game.grid.h;
    int empty_cells = 0;
    for (auto cell : game.grid) {
        if (!cell) empty_cells++;
    }
    
    int astar_covered = 0;
    for (auto& step : astar_dists) {
        if (step.dist < INT_MAX) astar_covered++;
    }
    
    int bfs_covered = 0;
    for (auto& step : bfs_dists) {
        if (step.dist < INT_MAX) bfs_covered++;
    }
    
    // Calculate percentage coverage
    float astar_percent = (empty_cells > 0) ? 
        (float)astar_covered / empty_cells * 100.0f : 0.0f;
    float bfs_percent = (empty_cells > 0) ? 
        (float)bfs_covered / empty_cells * 100.0f : 0.0f;
    
    // Find cells covered by BFS but not by A*
    int missing_cells = 0;
    std::vector<Coord> missing_coords;
    for (Coord c : game.grid.coords()) {
        if (!game.grid[c]) {  // If it's an empty cell
            if (bfs_dists[c].dist < INT_MAX && astar_dists[c].dist == INT_MAX) {
                missing_cells++;
                if (missing_coords.size() < 10) {  // Show up to 10 example coords
                    missing_coords.push_back(c);
                }
            }
        }
    }
    
    // Print results
    std::cout << "A* Distance Coverage Validation\n";
    std::cout << "-----------------------------\n";
    std::cout << "Grid size: " << width << "x" << height << "\n";
    std::cout << "Total cells: " << total_cells << "\n";
    std::cout << "Empty cells: " << empty_cells << "\n\n";
    
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "A* method (cell_tree_agent):\n";
    std::cout << "  Cells with valid distances: " << astar_covered 
              << " (" << astar_percent << "%)\n";
    
    std::cout << "BFS method (reference):\n";
    std::cout << "  Cells with valid distances: " << bfs_covered 
              << " (" << bfs_percent << "%)\n\n";
    
    std::cout << "Cells missing from A* but present in BFS: " << missing_cells << "\n";
    
    if (!missing_coords.empty()) {
        std::cout << "Example missing coords:\n";
        for (auto& coord : missing_coords) {
            std::cout << "  " << coord << "\n";
        }
    }
    
    // Determine if this is a problem
    if (astar_covered < bfs_covered) {
        std::cout << "\nCONCLUSION: A* is not calculating distances to all reachable cells!\n";
        std::cout << "This may affect unreachable cell detection accuracy.\n";
        return 1;
    } else {
        std::cout << "\nCONCLUSION: A* is calculating distances to all reachable cells.\n";
        std::cout << "Unreachable cell detection should be accurate.\n";
        return 0;
    }
}
