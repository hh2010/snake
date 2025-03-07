#pragma once

#include "agent.hpp"
#include "game_util.hpp"
#include "shortest_path.hpp"

//------------------------------------------------------------------------------
// Agent: Cheat agent
//------------------------------------------------------------------------------

// Takes the shortest path to the apple, ignoring snake collisions
// Only respects walls (grid boundaries)
struct CheatAgent : Agent {
  // Store the cached path to the apple
  std::vector<Coord> cached_path;
  
  Dir operator () (Game const& game, AgentLog* log = nullptr) override {
    Coord pos = game.snake_pos();
    Coord apple = game.apple_pos;
    
    // Always recalculate path when apple position changes
    if (cached_path.empty() || apple != game.apple_pos) {
      // Define an edge function that only checks for wall collisions, ignoring snake body
      auto edge = [&](Coord a, Coord b, Dir) {
        // Only check if the position is valid (inside grid boundaries)
        return game.grid.valid(b) ? 1 : INT_MAX;
      };
      
      // Find the shortest path to the apple
      auto dists = astar_shortest_path(game.grid.coords(), edge, pos, apple);
      cached_path = read_path(dists, pos, apple);
      
      // Log the path if logging is enabled
      if (log) {
        auto path_copy = cached_path;
        path_copy.push_back(pos);
        log->add(game.turn, AgentLog::Key::plan, std::move(path_copy));
      }
    }
    
    // If no path found or invalid path (should never happen unless apple is unreachable)
    if (cached_path.empty() || cached_path.back() == INVALID) {
      // Just move in any valid direction as a fallback
      for (auto dir : dirs) {
        Coord next = pos + dir;
        if (game.grid.valid(next)) {
            return dir;
        }
      }
      return Dir::right; // Default direction
    }
    
    // Get the next position from the path
    Coord next_pos = cached_path.back();
    cached_path.pop_back();
    
    // Return the direction to move
    return next_pos - pos;
  }
};