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

struct CellTreeAgent : public Agent {
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

  CellTreeAgent() : path_planner(0) {}

  // Getter for the metrics
  UnreachableMetrics getMetrics() const {
    return metrics;
  }

private:
  std::vector<Coord> cached_path;
  UnreachableMetrics metrics;
  SnakePathPlanner path_planner;

  Coord handleInvalidNextStep(Coord next_step, std::vector<Coord>& path) {
    if (next_step == INVALID) {
      if (!cached_path.empty()) {
        return cached_path.back();
      } else {
        // We somehow divided the grid into two parts.
        // Hack: if we pretend that we are at the goal, then the code below will trigger
        // because the current pos is unreachable from there.
        // path == {apple_pos,INVALID};
        path.pop_back();
        return path.back();
      }
    }
    return next_step;
  }

  // should these private functions be better organized?
  void logUnreachableMetrics(const Game& game, AgentLog* log) {
    std::vector<int> metrics_data = {
      metrics.first_unreachable_step,
      metrics.steps_with_unreachables,
      metrics.cumulative_unreachable_cells,
      metrics.length_at_first_unreachable
    };
    log->add(game.turn, AgentLog::Key::unreachable_metrics, metrics_data);
  }

  void updateUnreachableMetrics(const Game& game, AgentLog* log, const Unreachables& unreachable_move_tail) {
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

  bool should_use_cached_path_for_move_tail(const Unreachables& unreachable, Lookahead lookahead, const std::vector<Coord>& cached_path) {
    if (lookahead == Lookahead::many_move_tail) {
      if ((unreachable.any) && (unreachable.dist_to_farthest >= INT_MAX) && !cached_path.empty()) {
        return true;
      }
    }
    return false;
  }

  Unreachables get_unreachables(
      const GameBase& game, 
      const std::vector<Coord>& path, 
      Lookahead lookahead, 
      const Grid<Step>& dists) {
    if (lookahead == Lookahead::many_move_tail) {
      auto after = after_moves(game, path, Lookahead::many_move_tail);
      auto unreachable = cell_tree_unreachables(after, dists);

      if (!unreachable.any) {
        return unreachable;
      } else {
        auto after = after_moves(game, path, Lookahead::many_keep_tail);
        auto unreachable = cell_tree_unreachables(after, dists);
        return unreachable;
      }
    } else {
      auto after = after_moves(game, path, lookahead);
      auto unreachable = cell_tree_unreachables(after, dists);
      return unreachable;
    }
  }

  // figure out if i even need this
  // void logAfterSnakePosition(const Game& game, AgentLog* log, const GameBase& after) {
  //   // Store the "after" snake position for visualization
  //   if (log) {
  //     std::vector<Coord> after_snake_pos;
  //     for (const auto& pos : after.snake) {
  //       after_snake_pos.push_back(pos);
  //     }
  //     log->add(game.turn, AgentLog::Key::after_snake, after_snake_pos);
  //   }
  // }

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
    auto next_step = path.back();
    
    if (log) {
      auto path_copy = path;
      path_copy.push_back(pos); // do you even need to do this? i think it will just cut off at the grid in front of snake head?
      log->add(game.turn, AgentLog::Key::plan, std::move(path_copy));
    }

    next_step = handleInvalidNextStep(next_step, path);

    // Heuristic 3: prevent making parts of the grid unreachable
    if (detour != DetourStrategy::none) {
      Unreachables unreachable = get_unreachables(game, path, lookahead, dists);
      if (should_use_cached_path_for_move_tail(unreachable, lookahead, cached_path)) {
        Coord pos2 = cached_path.back();
        cached_path.pop_back();
        return pos2 - pos;
      }

      if (unreachable.any) {
        // Calculate steps_to_clear_unreachables based on unreachable cells
        int unreachable_count = 0;
        for (bool r : unreachable.reachable) {
          if (!r) unreachable_count++;
        }
        int steps_to_clear_unreachables = unreachable.dist_to_farthest == INT_MAX ? INT_MAX : unreachable.dist_to_farthest + unreachable_count;
        int extra_steps_desired = std::min(10, steps_to_clear_unreachables / 2);
        assert(extra_steps_desired >= 0);

        std::cout << "Turn " << game.turn << ": Unreachable cells detected, finding extended path with "
              << extra_steps_desired << " extra steps desired" << std::endl;
        path_planner.setExtraStepsDesired(extra_steps_desired);
        PathPlanningResult pathResult = path_planner.findExtendedPath(game, path, edge, unreachable);

        //   TODO: Decide what to do about this logging
        //   // kinda inefficient to always log the plan twice, even when it doesnt change?
        //   if (log) {
        //     log->add(game.turn, AgentLog::Key::plan_extended, pathResult.path);
        //   }

        path = std::move(pathResult.path);
        // Don't assign unreachable = pathResult.unreachables as it can cause issues
        updateUnreachableMetrics(game, log, pathResult.unreachable);
        detour = pathResult.unreachable.any ? detour : DetourStrategy::none;
        next_step = path.back();
        next_step = handleInvalidNextStep(next_step, path);

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
            if (pathResult.unreachable.dist_to_nearest < INT_MAX) {
              // move to an unreachable coord first
              next_step = first_step(dists, pos, pathResult.unreachable.nearest);
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
