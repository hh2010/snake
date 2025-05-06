#pragma once

#include "agent.hpp"
#include "game_util.hpp"
#include "shortest_path.hpp"
#include "snake_path_planner.hpp"
#include "cell_tree_utils.hpp"

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
  Lookahead lookahead = Lookahead::many_keep_tail;
  DetourStrategy detour = DetourStrategy::nearest_unreachable;
  int extra_steps_desired = 0;
  // penalties
  int same_cell_penalty = 0;
  int new_cell_penalty = 0;
  int parent_cell_penalty = 0;
  int edge_penalty_in = 0, edge_penalty_out = 0;
  int wall_penalty_in = 0, wall_penalty_out = 0;
  int open_penalty_in = 0, open_penalty_out = 0;

  CellTreeAgent() : path_planner(extra_steps_desired) {}

  // Getter for the metrics
  UnreachableMetrics getMetrics() const {
    return metrics;
  }

private:
  std::vector<Coord> cached_path;
  UnreachableMetrics metrics;
  SnakePathPlanner path_planner;

  void logUnreachableMetrics(const Game& game, AgentLog* log) {
    std::vector<int> metrics_data = {
      metrics.first_unreachable_step,
      metrics.steps_with_unreachables,
      metrics.cumulative_unreachable_cells,
      metrics.length_at_first_unreachable
    };
    log->add(game.turn, AgentLog::Key::unreachable_metrics, metrics_data);
  }

  // Create edge function for path finding
  // TODO: i think this can be refractored to return an int
  std::function<int(Coord, Coord, Dir)> createEdgeFunction(const GameBase& game, const Grid<Coord>& cell_parents) {
    return [&](Coord a, Coord b, Dir dir) {
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
        return INT_MAX;
      }
    };
  }

  Dir operator () (Game const& game, AgentLog* log = nullptr) override {    
    Coord pos = game.snake_pos();
    if (!cached_path.empty() && !recalculate_path) {
      Coord pos2 = cached_path.back();
      cached_path.pop_back();
      return pos2 - pos;
    }
    recalculate_path = true;
    
    // Find shortest path satisfying cell tree constraints
    auto cell_parents = cell_tree_parents(game.dimensions(), game.snake);
    auto edge = createEdgeFunction(game, cell_parents);
    auto dists = astar_shortest_path(game.grid.coords(), edge, pos, game.apple_pos, 1000);
    auto path = read_path(dists, pos, game.apple_pos);
    // int path_size_diff = cached_path.size() - path.size();  // need to make this 0 if the cached path was cleared on the previous turn
    auto path_size_diff = extra_steps_desired;
    path_planner.setExtraStepsDesired(path_size_diff);
    auto next_step = path.back();

    if (path_size_diff > 0) {
      std::cout << "shorter path found ";
      std::cout << game.turn << std::endl;
    }
    
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
      // TODO: make this its own detour strategy rather than always doing it
      // TODO: right now this only uses the new path if it clears all unreachables; might want to use if it clears some
      auto after_move_tail = after_moves(game, path, Lookahead::many_move_tail);
      auto unreachable_move_tail = cell_tree_unreachables(after_move_tail, dists);
      
      // auto after = after_moves(game, path, Lookahead::many_keep_tail);
      // auto unreachable = cell_tree_unreachables(after, dists);
      // Store the "after" snake position for visualization
      if (log) {
        std::vector<Coord> after_snake_pos;
        for (const auto& pos : after_move_tail.snake) {
          after_snake_pos.push_back(pos);
        }
        log->add(game.turn, AgentLog::Key::after_snake, after_snake_pos);
      }
      
      if (unreachable_move_tail.any) {
        if (path_size_diff > 0) {
          // Use path planner to find extended path
          auto extendedPath = path_planner.findExtendedPath(game, path, edge);
          
          if ((extendedPath.size() - path.size()) >= path_size_diff) {
            if (log) {
              log->add(game.turn, AgentLog::Key::safe_steps, extendedPath);  // i think we only want to log the detour steps here?
            }
            
            path = std::move(extendedPath);
            // recalculate_path = false;  // do this for the detour steps only?
          }
        }
        // Update metrics for unreachable cells
        if (metrics.first_unreachable_step == -1) {
          metrics.first_unreachable_step = game.turn;
          metrics.length_at_first_unreachable = game.snake.size();
        }
        metrics.steps_with_unreachables++;
        
        // Count unreachable cells
        int unreachable_count = 0;
        for (bool r : unreachable_move_tail.reachable) {
          if (!r) unreachable_count++;
        }
        metrics.cumulative_unreachable_cells += unreachable_count;
        
        if (log) {
          Grid<bool> unreachable_grid(game.dimensions());
          std::transform(unreachable_move_tail.reachable.begin(), unreachable_move_tail.reachable.end(), 
                        unreachable_grid.begin(), [](bool r){ return !r; });
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
          // 3B: move to one of the unreachable coords
          if (unreachable_move_tail.dist_to_nearest < INT_MAX) {
            // move to an unreachable coord first
            next_step = first_step(dists, pos, unreachable_move_tail.nearest);
            cached_path.clear();
            if (log) {
              // should probably log this below as well?
              logUnreachableMetrics(game, log);
            }
            return next_step - pos;
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
};

