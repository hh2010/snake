#include "game.hpp"
#include "game_util.hpp"
#include "shortest_path.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <fstream>
#include <iomanip>

// Validation result structure
struct UnreachableValidationResult {
    int total_empty_cells;
    int astar_covered_cells;
    int bfs_covered_cells;
    int astar_unreachable_cells;
    int bfs_unreachable_cells;
    int false_positives;  // Cells reported as unreachable by A* but reachable by BFS
    int false_negatives;  // Cells reported as reachable by A* but unreachable by BFS
    float astar_coverage_percent;
    float bfs_coverage_percent;
    float discrepancy_percent;
    
    // Output results to a stream
    void print(std::ostream& out) const {
        out << "Unreachable Cell Detection Validation\n";
        out << "-------------------------------------\n";
        out << "Total empty cells: " << total_empty_cells << "\n";
        out << std::fixed << std::setprecision(2);
        out << "A* (current method):\n";
        out << "  Cells with valid distances: " << astar_covered_cells 
            << " (" << astar_coverage_percent << "%)\n";
        out << "  Unreachable cells detected: " << astar_unreachable_cells << "\n";
        out << "BFS (reference method):\n";
        out << "  Cells with valid distances: " << bfs_covered_cells 
            << " (" << bfs_coverage_percent << "%)\n";
        out << "  Unreachable cells detected: " << bfs_unreachable_cells << "\n";
        out << "Discrepancies:\n";
        out << "  False positives (A* says unreachable, BFS says reachable): " << false_positives << "\n";
        out << "  False negatives (A* says reachable, BFS says unreachable): " << false_negatives << "\n";
        out << "  Total discrepancy rate: " << discrepancy_percent << "%\n";
    }
    
    // Save results to a CSV file
    bool save_to_csv(const std::string& filepath, bool append = true) const {
        std::ofstream file;
        bool file_exists = std::ifstream(filepath).good() && append;
        
        if (append) {
            file.open(filepath, std::ios_base::app);
        } else {
            file.open(filepath);
        }
        
        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filepath << std::endl;
            return false;
        }
        
        // Write header if file is new
        if (!file_exists) {
            file << "total_empty_cells,astar_covered_cells,bfs_covered_cells,"
                 << "astar_unreachable_cells,bfs_unreachable_cells,"
                 << "false_positives,false_negatives,"
                 << "astar_coverage_percent,bfs_coverage_percent,discrepancy_percent\n";
        }
        
        // Write data
        file << total_empty_cells << ","
             << astar_covered_cells << ","
             << bfs_covered_cells << ","
             << astar_unreachable_cells << ","
             << bfs_unreachable_cells << ","
             << false_positives << ","
             << false_negatives << ","
             << astar_coverage_percent << ","
             << bfs_coverage_percent << ","
             << discrepancy_percent << "\n";
             
        file.close();
        return true;
    }
};

// Count cells with valid distances in a distance grid
int count_valid_distances(const Grid<Step>& dists) {
    int count = 0;
    for (auto& step : dists) {
        if (step.dist < INT_MAX) {
            count++;
        }
    }
    return count;
}

// Count empty cells in a grid
int count_empty_cells(const Grid<bool>& grid) {
    int count = 0;
    for (auto cell : grid) {
        if (!cell) {
            count++;
        }
    }
    return count;
}

// Validate unreachable cell detection by comparing A* and BFS methods
UnreachableValidationResult validate_unreachables(
    const GameBase& game,
    const Grid<Step>& astar_dists,
    const std::function<int(Coord, Coord, Dir)>& edge_function) {
    
    // Calculate BFS distances (correct reference implementation)
    auto bfs_dists = shortest_path(game.grid, game.snake_pos());
    
    // Calculate unreachable cells using both methods
    auto astar_unreachable = cell_tree_unreachables(game, astar_dists);
    auto bfs_unreachable = cell_tree_unreachables(game, bfs_dists);
    
    // Count cells
    int total_empty_cells = count_empty_cells(game.grid);
    int astar_covered_cells = count_valid_distances(astar_dists);
    int bfs_covered_cells = count_valid_distances(bfs_dists);
    
    // Count unreachable cells
    int astar_unreachable_count = astar_unreachable.countUnreachableCells();
    int bfs_unreachable_count = bfs_unreachable.countUnreachableCells();
    
    int false_positives = 0;
    int false_negatives = 0;
    
    for (size_t i = 0; i < astar_unreachable.reachable.size(); i++) {
        bool astar_says_reachable = astar_unreachable.reachable[i];
        bool bfs_says_reachable = bfs_unreachable.reachable[i];
        
        // False positive: A* says unreachable but BFS says reachable
        if (!astar_says_reachable && bfs_says_reachable) {
            false_positives++;
        }
        
        // False negative: A* says reachable but BFS says unreachable
        if (astar_says_reachable && !bfs_says_reachable) {
            false_negatives++;
        }
    }
    
    // Calculate percentages
    float astar_coverage_percent = (total_empty_cells > 0) ? 
        (float)astar_covered_cells / total_empty_cells * 100.0f : 0.0f;
    float bfs_coverage_percent = (total_empty_cells > 0) ? 
        (float)bfs_covered_cells / total_empty_cells * 100.0f : 0.0f;
    float discrepancy_percent = (astar_unreachable.reachable.size() > 0) ? 
        (float)(false_positives + false_negatives) / astar_unreachable.reachable.size() * 100.0f : 0.0f;
    
    // Return the validation result
    return {
        total_empty_cells,
        astar_covered_cells,
        bfs_covered_cells,
        astar_unreachable_count,
        bfs_unreachable_count,
        false_positives,
        false_negatives,
        astar_coverage_percent,
        bfs_coverage_percent,
        discrepancy_percent
    };
}

// Function to run a game simulation and validate unreachable detection
void run_validation_test(int width, int height, int num_turns, bool log_each_turn = true) {
    std::cout << "Running validation test on " << width << "x" << height << " grid for " << num_turns << " turns.\n";
    
    // Create a game
    Game game(width, height);
    
    // Variables to track validation stats
    int total_turns_with_unreachables = 0;
    int total_turns_with_discrepancies = 0;
    float avg_false_positives = 0.0f;
    float avg_false_negatives = 0.0f;
    
    // Setup random direction generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dir_dist(0, 3);
    const Dir directions[4] = {Dir::up, Dir::down, Dir::left, Dir::right};
    
    // Log file path
    std::string csv_path = "unreachable_validation_" + std::to_string(width) + "x" + std::to_string(height) + ".csv";
    
    // Write header row to CSV file
    std::ofstream csv_file(csv_path);
    if (csv_file.is_open()) {
        csv_file << "turn,total_empty_cells,astar_covered_cells,bfs_covered_cells,"
                 << "astar_unreachable_cells,bfs_unreachable_cells,"
                 << "false_positives,false_negatives,"
                 << "astar_coverage_percent,bfs_coverage_percent,discrepancy_percent\n";
        csv_file.close();
    }
    
    // Simulate game for specified number of turns
    for (int turn = 0; turn < num_turns; turn++) {
        // Create edge function for A* path finding
        auto cell_parents = cell_tree_parents(game.dimensions(), game.snake);
        auto edge = [&cell_parents, &game](Coord a, Coord b, Dir dir) {
            if (can_move_in_cell_tree(cell_parents, a, b, dir) && !game.grid[b]) {
                return 1000; // Base cost
            } else {
                return INT_MAX;
            }
        };
        
        // Calculate A* path (as used in cell_tree_agent)
        auto astar_dists = astar_shortest_path(
            game.grid.coords(), 
            edge, 
            game.snake_pos(), 
            game.apple_pos,
            1000);
        
        // Validate results
        auto result = validate_unreachables(game, astar_dists, edge);
        
        // Log result for this turn
        if (log_each_turn) {
            std::cout << "Turn " << turn << " validation:\n";
            result.print(std::cout);
            std::cout << "----------------------------------------\n";
        }
        
        // Save to CSV
        result.save_to_csv(csv_path);
        
        // Update stats
        if (result.astar_unreachable_cells > 0 || result.bfs_unreachable_cells > 0) {
            total_turns_with_unreachables++;
        }
        
        if (result.false_positives > 0 || result.false_negatives > 0) {
            total_turns_with_discrepancies++;
            avg_false_positives += result.false_positives;
            avg_false_negatives += result.false_negatives;
        }
        
        // Move snake randomly (but avoid game over)
        bool moved = false;
        int attempts = 0;
        const int max_attempts = 10; // Try at most 10 random directions
        
        while (!moved && attempts < max_attempts) {
            Dir random_dir = directions[dir_dist(gen)];
            Coord next_head = game.snake_pos() + random_dir;
            
            if (game.grid.valid(next_head) && !game.grid[next_head]) {
                game.move(random_dir);
                moved = true;
            }
            
            attempts++;
        }
        
        // If we couldn't move, the snake is trapped - end the simulation
        if (!moved) {
            std::cout << "Snake trapped after " << turn << " turns, ending simulation.\n";
            break;
        }
    }
    
    // Calculate averages for summary
    if (total_turns_with_discrepancies > 0) {
        avg_false_positives /= total_turns_with_discrepancies;
        avg_false_negatives /= total_turns_with_discrepancies;
    }
    
    // Print summary
    std::cout << "\nValidation Test Summary:\n";
    std::cout << "------------------------\n";
    std::cout << "Total turns: " << num_turns << "\n";
    std::cout << "Turns with unreachable cells: " << total_turns_with_unreachables << "\n";
    std::cout << "Turns with discrepancies between A* and BFS: " << total_turns_with_discrepancies << "\n";
    std::cout << "Average false positives per discrepancy: " << avg_false_positives << "\n";
    std::cout << "Average false negatives per discrepancy: " << avg_false_negatives << "\n";
    std::cout << "CSV log saved to: " << csv_path << "\n";
}

int main(int argc, char* argv[]) {
    // Default parameters
    int width = 10;
    int height = 10;
    int num_turns = 100;
    bool verbose = false;
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--width" && i + 1 < argc) {
            width = std::stoi(argv[++i]);
        } else if (arg == "--height" && i + 1 < argc) {
            height = std::stoi(argv[++i]);
        } else if (arg == "--turns" && i + 1 < argc) {
            num_turns = std::stoi(argv[++i]);
        } else if (arg == "--verbose") {
            verbose = true;
        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n";
            std::cout << "Options:\n";
            std::cout << "  --width WIDTH       Set the grid width (default: 10)\n";
            std::cout << "  --height HEIGHT     Set the grid height (default: 10)\n";
            std::cout << "  --turns TURNS       Set the number of turns to simulate (default: 100)\n";
            std::cout << "  --verbose           Enable detailed logging for each turn\n";
            std::cout << "  --help              Display this help message\n";
            return 0;
        }
    }
    
    // Run the validation test
    run_validation_test(width, height, num_turns, verbose);
    
    return 0;
}
