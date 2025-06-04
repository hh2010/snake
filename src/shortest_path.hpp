#pragma once
#include "util.hpp"
#include <queue>
#include <chrono>

//------------------------------------------------------------------------------
// Shortest paths by breath first search
//------------------------------------------------------------------------------

struct ShortestPathTimer {
  static std::chrono::nanoseconds total_time;
  static int call_count;
  
  static void reset() {
    total_time = std::chrono::nanoseconds(0);
    call_count = 0;
  }
  
  static void print_stats() {
    if (call_count > 0) {
      double avg_ms = std::chrono::duration<double, std::milli>(total_time).count() / call_count;
      std::cout << "\nShortest path timing stats:" << std::endl;
      std::cout << "  Total calls: " << call_count << std::endl;
      std::cout << "  Total time: " << std::chrono::duration<double, std::milli>(total_time).count() << " ms" << std::endl;
      std::cout << "  Average time: " << avg_ms << " ms per call" << std::endl;
    }
  }
};

std::chrono::nanoseconds ShortestPathTimer::total_time = std::chrono::nanoseconds(0);
int ShortestPathTimer::call_count = 0;

// Timer for A* algorithm
struct AStarTimer {
  static std::chrono::nanoseconds total_time;
  static int call_count;
  
  static void reset() {
    total_time = std::chrono::nanoseconds(0);
    call_count = 0;
  }
  
  static void print_stats() {
    if (call_count > 0) {
      double avg_ms = std::chrono::duration<double, std::milli>(total_time).count() / call_count;
      std::cout << "\nA* shortest path timing stats:" << std::endl;
      std::cout << "  Total calls: " << call_count << std::endl;
      std::cout << "  Total time: " << std::chrono::duration<double, std::milli>(total_time).count() << " ms" << std::endl;
      std::cout << "  Average time: " << avg_ms << " ms per call" << std::endl;
    }
  }
};

std::chrono::nanoseconds AStarTimer::total_time = std::chrono::nanoseconds(0);
int AStarTimer::call_count = 0;

// Find shortest paths using breath first search
struct Step {
  int dist;
  Coord from;
  bool reachable() const {
    return dist < INT_MAX;
  }
};

template <typename CanMove>
Grid<Step> generic_shortest_path(CoordRange dims, CanMove const& can_move, Coord from, Coord to = {-1,-1}) {
  auto start_time = std::chrono::high_resolution_clock::now();
  
  Grid<Step> out(dims, Step{INT_MAX, NOT_VISITED});
  std::vector<Coord> queue, next;
  queue.push_back(from);
  out[from].dist = 0;
  out[from].from = ROOT;
  int dist = 0;
  while (!queue.empty()) {
    dist++;
    for (auto a : queue) {
      for (auto d : dirs) {
        Coord b = a + d;
        if (dims.valid(b) && can_move(a,b,d) && out[b].dist > dist) {
          out[b].dist = dist;
          out[b].from = a;
          next.push_back(b);
          if (b == to) {
            auto end_time = std::chrono::high_resolution_clock::now();
            ShortestPathTimer::total_time += end_time - start_time;
            ShortestPathTimer::call_count++;
            return out;
          }
        }
      }
    }
    std::swap(queue,next);
    next.clear();
  }
  
  auto end_time = std::chrono::high_resolution_clock::now();
  ShortestPathTimer::total_time += end_time - start_time;
  ShortestPathTimer::call_count++;
  
  return out;
}

Grid<Step> shortest_path(Grid<bool> const& grid, Coord from, Coord to = {-1,-1}) {
  return generic_shortest_path(grid.coords(), [&grid](Coord a, Coord b, Dir d){ return !grid[b]; }, from, to);
}

Coord first_step(Grid<Step> const& path, Coord from, Coord to) {
  while (to != ROOT && path[to].from != from) {
    if (to == NOT_VISITED) break;
    to = path[to].from;
  }
  return to;
}

// Find all coords in a path from from to to, excluding the start point
// Note: returned in reverse order, that is result.back() is the first step, result.front() == to
std::vector<Coord> read_path(Grid<Step> const& paths, Coord from, Coord to) {
  std::vector<Coord> steps;
  while (to != ROOT && to != from) {
    steps.push_back(to);
    if (to == NOT_VISITED) break;
    to = paths[to].from;
  }
  return steps;
}

// Add operator for Grid<string>
std::ostream& operator << (std::ostream& out, Grid<std::string> const& grid) {
  for (int y = 0; y < grid.h; ++y) {
    for (int x = 0; x < grid.w; ++x) {
      out << grid[{x,y}];
    }
    out << std::endl;
  }
  return out;
}

std::ostream& operator << (std::ostream& out, Grid<Step> const& paths) {
  Grid<std::string> vis(paths.dimensions());
  std::transform(paths.begin(), paths.end(), vis.begin(), [](Step x) {
    return x.dist == INT_MAX ? "." : "#";
  });
  return out << vis;
}

//------------------------------------------------------------------------------
// Shortest paths with A-star algorithm
//------------------------------------------------------------------------------

template <typename Edge>
Grid<Step> astar_shortest_path(CoordRange dims, Edge const& edges, Coord from, Coord to, int min_distance_cost=1) {
  // Start timing
  auto start_time = std::chrono::high_resolution_clock::now();
  
  Grid<Step> out(dims, Step{INT_MAX, INVALID});
  struct Item {
    Coord c;
    int dist;
    inline bool operator < (Item const& b) const {
      return dist > b.dist;
    }
  };
  std::priority_queue<Item> queue;
  auto bound = [=](Coord a) { return min_distance_cost * (abs(a.x-to.x) + abs(a.y-to.y));};
  out[from].dist = 0;
  queue.push(Item{from, 0+bound(from)});
  while (!queue.empty()) {
    auto item = queue.top();
    queue.pop();
    if (item.c == to) break;
    for (auto d : dirs) {
      Coord b = item.c + d;
      if (!dims.valid(b)) continue;
      auto edge = edges(item.c,b,d);  // whats the impact of edge being 1000 instead of 0? or just calculate distance here insetad of rely on bound function
      if (edge == INT_MAX) continue;  // what happens if all are max?
      int new_dist = out[item.c].dist + edge;
      if (new_dist < out[b].dist) {  // so it will just pick the first one of U,D,L,R if impact is same
        out[b].dist = new_dist;
        out[b].from = item.c;
        queue.push(Item{b, new_dist+bound(b)});
      }
    }
  }
  
  // End timing and update stats
  auto end_time = std::chrono::high_resolution_clock::now();
  AStarTimer::total_time += end_time - start_time;
  AStarTimer::call_count++;
  
  return out;
}

// Dynamic A* that accounts for snake movement during pathfinding
template <typename Edge>
Grid<Step> astar_shortest_path_dynamic_snake(
    CoordRange dims, 
    Edge const& edges, 
    GameBase const& initial_game,
    Coord from, 
    Coord to, 
    int min_distance_cost=1) {
  
  Grid<Step> out(dims, Step{INT_MAX, INVALID});
  
  struct Item {
    Coord c;
    int dist;
    int steps_taken; // Track how many steps we've taken
    inline bool operator < (Item const& b) const {
      return dist > b.dist;
    }
  };
  
  std::priority_queue<Item> queue;
  auto bound = [=](Coord a) { return min_distance_cost * (abs(a.x-to.x) + abs(a.y-to.y));};
  
  out[from].dist = 0;
  queue.push(Item{from, 0+bound(from), 0});
  
  while (!queue.empty()) {
    auto item = queue.top();
    queue.pop();
    if (item.c == to) continue;
    
    // Create projected game state after 'steps_taken' moves
    auto projected_game = project_game_state_for_pathfinding(initial_game, item.steps_taken);
    
    for (auto d : dirs) {
      Coord b = item.c + d;
      if (!dims.valid(b)) continue;
      
      // Use the projected game state for edge calculation
      auto edge_cost = edges(item.c, b, d, projected_game);
      if (edge_cost == INT_MAX) continue;
      
      int new_dist = out[item.c].dist + edge_cost;
      if (new_dist < out[b].dist) {
        out[b].dist = new_dist;
        out[b].from = item.c;
        queue.push(Item{b, new_dist+bound(b), item.steps_taken + 1});
      }
    }
  }
  return out;
}

// Helper function to simulate game state after n moves without eating apples
GameBase project_game_state_for_pathfinding(GameBase const& game, int steps) {
  GameBase projected = game;
  
  // Simulate snake tail movement (assuming no apples are eaten)
  // We free up tail positions that would be vacated by the time we reach this step
  int tail_moves = std::min(steps, static_cast<int>(projected.snake.size()) - 1);
  
  // Remove tail segments from both grid and snake
  for (int i = 0; i < tail_moves; ++i) {
    if (projected.snake.size() > 1) {
      Coord tail = projected.snake.back();
      projected.grid[tail] = false; // Free up tail position in grid
      projected.snake.pop_back();   // Remove tail from snake
    }
  }
  
  return projected;
}

//------------------------------------------------------------------------------
// Flood fill
//------------------------------------------------------------------------------

// Flood fill debug structure and functions
struct FloodFillDebug {
    int turn = -1;
    CoordRange size;
    std::vector<Coord> snake_pos;
    int snake_size;
    Coord apple_pos;
    Coord start_coord;
    std::vector<Grid<bool>> fill_states;

    static FloodFillDebug* active_debug;

    FloodFillDebug() {
        active_debug = nullptr;
    }
    ~FloodFillDebug() {
        if (active_debug == this) {
            active_debug = nullptr;
        }
    }

    void capture_state(const Grid<bool>& state) {
        if (active_debug == this) {
            fill_states.push_back(state);
        }
    }
};

FloodFillDebug* FloodFillDebug::active_debug = nullptr;

// Flood fill implementation
template <typename CanMove>
void flood_fill_go(Grid<bool>& out, CanMove const& can_move, Coord a) {
    if (!out.valid(a)) {
        std::cout << "Invalid coordinate: " << a << std::endl;
        return;
    }

    auto fill_horizontal_line = [&](int y, int& min_x, int& max_x) {
        // Find leftmost point
        min_x = a.x;
        while (min_x > 0 && can_move(Coord{min_x,y}, Coord{min_x-1,y}, Dir::left) 
               && !out[Coord{min_x-1,y}]) {
            min_x--;
        }

        // Find rightmost point
        max_x = a.x;
        while (max_x + 1 < out.w && can_move(Coord{max_x,y}, Coord{max_x+1,y}, Dir::right) 
               && !out[Coord{max_x+1,y}]) {
            max_x++;
        }
        std::fill(&out[Coord{min_x,y}], &out[Coord{max_x,y}] + 1, true);

        if (FloodFillDebug::active_debug) {
              FloodFillDebug::active_debug->capture_state(out);
        }
    };

    int y = a.y;
    int min_x, max_x;
    
    fill_horizontal_line(y, min_x, max_x);

    // Recursively fill connected lines above and below
    for (int x = min_x; x <= max_x; ++x) {
        if (y > 0 && can_move(Coord{x,y}, Coord{x,y-1}, Dir::up) 
            && !out[Coord{x,y-1}]) {
            flood_fill_go(out, can_move, Coord{x,y-1});
        }
        if (y+1 < out.h && can_move(Coord{x,y}, Coord{x,y+1}, Dir::down) 
            && !out[Coord{x,y+1}]) {
            flood_fill_go(out, can_move, Coord{x,y+1});
        }
    }
}

template <typename CanMove>
Grid<bool> flood_fill(CoordRange dims, CanMove const& can_move, Coord from) {
  Grid<bool> out(dims, false);
  flood_fill_go(out, can_move, from);
  return out;
}

// flood fill starting at a neihbor of from
Grid<bool> flood_fill_from_neighbor(Grid<bool> const& grid, Coord from) {
  Grid<bool> out(grid.dimensions(), false);
  for (Dir d : dirs) {
    if (grid.is_clear(from + d)) {
      flood_fill_go(out, [&](Coord, Coord to, Dir){ return !grid[to]; }, from + d);
      break;
    }
  }
  return out;
}
// flood fill from a single position
Grid<bool> flood_fill(Grid<bool> const& grid) {
  Grid<bool> out(grid.dimensions(), false);
  for (Coord c : grid.coords()) {
    if (!grid[c]) {
      flood_fill_go(out, [&](Coord, Coord to, Dir){ return !grid[to]; }, c);
      break;
    }
  }
  return out;
}

