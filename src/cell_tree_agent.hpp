#include "agent.hpp"
#include "game_util.hpp"
#include "shortest_path.hpp"
#include <functional> // Add missing include for std::function

//------------------------------------------------------------------------------
// Agent: tree based
//------------------------------------------------------------------------------

// We should be able to use a shortest-path algorithm on the original snake-level that maintains the cell and tree constraints
// 1. Cell contraint means that only some moves are possible (2 dirs in each coord instead of 4)
//    See can_move_in_tree_cell
// 2. Tree constraint means that we can't move into a cell except from its direct children
//    If we take the snake's tail to be the root of the tree then
//     * moving to parent from child means retracing our steps, this is always possible
//     * moving to unvisited cells is always possible
//     * moving to an existing child from a parent never happens
// 3. We have to be able to cover all cells with a tree.
//    Equivalently, 
// Condition 1 and 2 are doable, but combining with 3 is (probably) NP-hard (it is in the general graph case)

// Simple heuristic (3A):
//  * use shortest path satisfying 1,2
//  * if the resulting move makes some parts of the grid unreachable, then perform the another move instead
//    (there are always at most two possible moves)

// Better(?) heuristic (3B):
//  * first find shortest path satisfying 1,2.
//  * then check the state after following the path to the goal.
//  * if some coords become unreachable at that time, then we have clearly failed to maintain a tree.
//  * in that case, instead use the shortest path to one of the unreachable cells 

// Bonus (4):
// It would also be good to hug walls, to avoid creating large almost closed regions
// that could be added as a factor to the shortest path code


// Find current tree (represented as parent pointers)
// note: the returned grid is only w/2 * h/2
// {-1,-1} indicates cell is not visited
// {-2,-2} indicates cell is root
Grid<CellCoord> cell_tree_parents(CoordRange dims, RingBuffer<Coord> const& snake) {
  Grid<CellCoord> parents(dims.w/2, dims.h/2, NOT_VISITED);
  CellCoord parent = ROOT;
  for (int i=snake.size()-1 ; i >= 0; --i) {
    Coord c = snake[i];
    CellCoord cell_coord = cell(c);
    if (parents[cell_coord] == NOT_VISITED) {
      parents[cell_coord] = parent;
    }
    parent = cell_coord;
  }
  return parents;
}

// can you move from a to b?
bool can_move_in_cell_tree(Grid<Coord> const& cell_parents, Coord a, Coord b, Dir dir) {
  // condition 1
  if (!is_cell_move(a, dir)) return false;
  // condition 2 (only move to parent or unvisted cell)
  Coord cell_a = cell(a);
  Coord cell_b = cell(b);
  return cell_b == cell_a || cell_parents[cell_b] == NOT_VISITED || cell_parents[cell_a] == cell_b;
}

Dir move_to_parent(Grid<Coord> const& cell_parents, Coord a) {
  Coord cell_a = cell(a);
  Coord parent = cell_parents[cell_a];
  int x = a.x % 2, y = a.y % 2;
  if (x == 1 && y == 0) return parent.y < cell_a.y ? Dir::up    : Dir::left;
  if (x == 0 && y == 1) return parent.y > cell_a.y ? Dir::down  : Dir::right;
  if (x == 0 && y == 0) return parent.x < cell_a.x ? Dir::left  : Dir::down;
  if (x == 1 && y == 1) return parent.x > cell_a.x ? Dir::right : Dir::up;
  throw "move_to_parent";
}


Unreachables cell_tree_unreachables(GameBase const& game, Grid<Step> const& dists) {
  auto cell_parents = cell_tree_parents(game.dimensions(), game.snake);
  auto can_move = [&](Coord from, Coord to, Dir dir) {
    return can_move_in_cell_tree(cell_parents, from, to, dir) && !game.grid[to];
  };
  return unreachables(can_move, game, dists);
}

// Get all unreachable cells from the Unreachables object
std::vector<Coord> get_all_unreachable_cells(const Unreachables& unreachable, const CoordRange& dims) {
  std::vector<Coord> result;
  for (auto coord : dims) {
    if (!unreachable.reachable[coord]) {
      result.push_back(coord);
    }
  }
  return result;
}

// Check if all unreachable cells can be visited from the current position using BFS
// Returns true if all unreachables can be visited, false otherwise
bool can_visit_all_unreachables(
    const Game& game, 
    const std::vector<Coord>& unreachable_cells,
    const std::function<int(Coord, Coord, Dir)>& edge_fn) {

  assert(!unreachable_cells.empty() && "Unreachable cells list should not be empty");

  // Start timing
  auto start_time = std::chrono::high_resolution_clock::now();

  // Use a more efficient approach with a single BFS traversal
  // Start from current snake position
  Coord start_pos = game.snake_pos();
  std::vector<bool> unreachable_visited(unreachable_cells.size(), false);
  int remaining_unreachables = unreachable_cells.size();
  
  // Create a mapping from coordinates to unreachable indices for quick lookup
  Grid<int> unreachable_indices(game.dimensions(), -1);
  for (size_t i = 0; i < unreachable_cells.size(); i++) {
    unreachable_indices[unreachable_cells[i]] = i;
  }
  
  // BFS queue
  std::vector<Coord> queue;
  Grid<bool> visited(game.dimensions(), false);
  
  // Initialize with start position
  queue.push_back(start_pos);
  visited[start_pos] = true;
  
  // If start position is an unreachable cell, mark it
  int start_idx = unreachable_indices[start_pos];
  if (start_idx >= 0) {
    unreachable_visited[start_idx] = true;
    remaining_unreachables--;
    
    // If we've visited all unreachables, we're done
    if (remaining_unreachables == 0) {
      // End timing
      auto end_time = std::chrono::high_resolution_clock::now();
      BFSTimer::total_time += end_time - start_time;
      BFSTimer::call_count++;
      return true;
    }
  }
  
  // Standard BFS implementation
  for (size_t i = 0; i < queue.size(); i++) {
    Coord current = queue[i];
    
    // Try all four directions
    for (Dir dir : dirs) {
      Coord next = current + dir;
      
      // Check if this is a valid move
      if (game.dimensions().valid(next) && !visited[next] && edge_fn(current, next, dir) != INT_MAX) {
        // Mark as visited
        visited[next] = true;
        queue.push_back(next);
        
        // Check if this is an unreachable cell
        int unreachable_idx = unreachable_indices[next];
        if (unreachable_idx >= 0 && !unreachable_visited[unreachable_idx]) {
          unreachable_visited[unreachable_idx] = true;
          remaining_unreachables--;
          
          // If we've visited all unreachables, we're done
          if (remaining_unreachables == 0) {
            // End timing
            auto end_time = std::chrono::high_resolution_clock::now();
            BFSTimer::total_time += end_time - start_time;
            BFSTimer::call_count++;
            return true;
          }
        }
      }
    }
  }
  
  // End timing
  auto end_time = std::chrono::high_resolution_clock::now();
  BFSTimer::total_time += end_time - start_time;
  BFSTimer::call_count++;
  
  // If the BFS is exhausted and we still have unreachable cells,
  // then at least one unreachable cell cannot be visited
  return false;
}

enum class DetourStrategy {
  none,
  any,
  nearest_unreachable
};

// Metrics for tracking unreachable cells
struct UnreachableMetrics {
  int first_unreachable_step = -1;  // -1 means no unreachable cells have been detected yet
  int steps_with_unreachables = 0;
  int cumulative_unreachable_cells = 0;
  int length_at_first_unreachable = 0;
};

struct CellTreeAgent : Agent {
public:
  // config
  bool recalculate_path = true;
  Lookahead lookahead = Lookahead::many_move_tail;
  DetourStrategy detour = DetourStrategy::nearest_unreachable;
  // penalties
  int same_cell_penalty = 0;
  int new_cell_penalty = 0;
  int parent_cell_penalty = 0;
  int edge_penalty_in = 0, edge_penalty_out = 0;
  int wall_penalty_in = 0, wall_penalty_out = 0;
  int open_penalty_in = 0, open_penalty_out = 0;

private:
  std::vector<Coord> cached_path;
  UnreachableMetrics metrics;

  void logUnreachableMetrics(const Game& game, AgentLog* log) {
    std::vector<int> metrics_data = {
      metrics.first_unreachable_step,
      metrics.steps_with_unreachables,
      metrics.cumulative_unreachable_cells,
      metrics.length_at_first_unreachable
    };
    log->add(game.turn, AgentLog::Key::unreachable_metrics, metrics_data);
  }

  Dir move_to_unreachable(Game const& game, Grid<Step> const& dists, Coord pos, Unreachables const& unreachable, AgentLog* log, std::vector<Coord> path) {
    // move to an unreachable coord first
    Coord next_step_to_unreachable = first_step(dists, pos, unreachable.nearest);
    if (log) {
      logUnreachableMetrics(game, log);
    }
    if ((cached_path.back() != next_step_to_unreachable) && (cached_path.size() > 0))
    {
      cached_path.clear();
    }
    else
    {
      cached_path = std::move(path);
      cached_path.pop_back();
    }
    return next_step_to_unreachable - pos;
  }

public:
  Dir operator () (Game const& game, AgentLog* log = nullptr) override {
    Coord pos = game.snake_pos();
    if (!cached_path.empty() && !recalculate_path) {
      Coord pos2 = cached_path.back();
      cached_path.pop_back();
      return pos2 - pos;
    }
    
    // Find shortest path satisfying 1,2
    auto cell_parents = cell_tree_parents(game.dimensions(), game.snake);
    auto edge = [&](Coord a, Coord b, Dir dir) {
      if (can_move_in_cell_tree(cell_parents, a, b, dir) && !game.grid[b]) {
        // small penalty for moving to same/different cell
        bool to_parent = cell(b) == cell_parents[cell(a)];
        bool to_same   = cell(b) == cell(a);
        Dir right = rotate_clockwise(dir);
        bool hugs_edge = !game.grid.valid(b+right);
        bool hugs_wall = !hugs_edge && game.grid[b+right];
        return 1000
          + (to_parent ? parent_cell_penalty : to_same ? same_cell_penalty : new_cell_penalty)
          + (to_same ? (hugs_edge ? edge_penalty_in  : hugs_wall ? wall_penalty_in  : open_penalty_in)
                     : (hugs_edge ? edge_penalty_out : hugs_wall ? wall_penalty_out : open_penalty_out));
      } else {
        return INT_MAX;  // feel like the game should just assert out here
      }
    };
    auto dists = astar_shortest_path(game.grid.coords(), edge, pos, game.apple_pos, 1000);
    auto path = read_path(dists, pos, game.apple_pos);
    auto next_step = path.back();
    
    if (log) {
      auto path_copy = path;
      path_copy.push_back(pos);
      log->add(game.turn, AgentLog::Key::plan, std::move(path_copy));
    }
    
    if (next_step == INVALID) {
      if (!cached_path.empty()) {
        next_step = cached_path.back();
      } else {
        // We somehow divided the grid into two parts.
        // Hack: if we pretend that we are at the goal, then the code below will trigger
        // because the current pos is unreachable from there.
        // path == {apple_pos,INVALID};
        path.pop_back();
        next_step = path.back();
      }
    }
    
    // Heuristic 3: prevent making parts of the grid unreachable
    if (detour != DetourStrategy::none) {
      auto after = after_moves(game, path, lookahead);
      auto unreachable = cell_tree_unreachables(after, dists);
      
      // Store the "after" snake position for visualization
      if (log) {
        std::vector<Coord> after_snake_pos;
        for (const auto& pos : after.snake) {
          after_snake_pos.push_back(pos);
        }
        log->add(game.turn, AgentLog::Key::after_snake, after_snake_pos);
      }
      
      if (unreachable.any) {
        // Update metrics for unreachable cells
        if (metrics.first_unreachable_step == -1) {
          metrics.first_unreachable_step = game.turn;
          metrics.length_at_first_unreachable = game.snake.size();
        }
        metrics.steps_with_unreachables++;
        
        // Count unreachable cells
        int unreachable_count = 0;
        for (bool r : unreachable.reachable) {
          if (!r) unreachable_count++;
        }
        metrics.cumulative_unreachable_cells += unreachable_count;
        
        if (log) {
          Grid<bool> unreachable_grid(game.dimensions());
          std::transform(unreachable.reachable.begin(), unreachable.reachable.end(), unreachable_grid.begin(), [](bool r){ return !r; });
          log->add(game.turn, AgentLog::Key::unreachable, unreachable_grid);
        }
        
        if (detour == DetourStrategy::any) {
          // 3A: move in any other direction
          for (auto dir : dirs) {
            if (edge(pos,pos+dir,dir) != INT_MAX && pos+dir != next_step) {
              //std::cout << game << "Moving " << dir << " instead of " << (pos2-pos) << " to avoid unreachables" << std::endl;
              cached_path.clear();
              return dir;
            }
          }
        } else if (detour == DetourStrategy::nearest_unreachable) {
          // Get all unreachable cells
          std::vector<Coord> all_unreachables = get_all_unreachable_cells(unreachable, game.dimensions());
          
          // Check if all unreachable cells can be visited
          bool all_unreachables_can_be_visited = can_visit_all_unreachables(game, all_unreachables, edge);
          
          if (all_unreachables_can_be_visited) {
            return move_to_unreachable(game, dists, pos, unreachable, log, path);
          }
          
          // failed to find detour
          // This can happen because it previously looked like everything would be reachable upon reaching the apple,
          // but moving the snake's tail opened up a shorter path
          // Solution: just continue along previous path
          if (!cached_path.empty()) {
            next_step = cached_path.back();
            cached_path.pop_back();
            return next_step - pos;
          }
        }
        if (0) {
          std::cout << game;
          std::cout << "Unreachable grid points (will) exist, but no alternative moves or cached path" << std::endl;
        }
      }
    }
    
    // use as new cached path
    cached_path = std::move(path);
    cached_path.pop_back();
    
    return next_step - pos;
  }
  
  // Getter for the metrics
  UnreachableMetrics getMetrics() const {
    return metrics;
  }
};

