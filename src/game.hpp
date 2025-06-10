#pragma once

#include "util.hpp"
#include "random.hpp"
#include <string>
#include <variant>
#include <vector>
#include <random>
#include <memory>

// Forward declaration
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

//------------------------------------------------------------------------------
// Game state
//------------------------------------------------------------------------------

using Snake = RingBuffer<Coord>;

// State of a game of snake, without tracking anything else
class GameBase {
public:
  Grid<bool> grid;
  Snake snake;
  Coord apple_pos;
  
  GameBase(CoordRange range)
    : grid(range, false)
    , snake(range.size() + 1)
    , apple_pos(INVALID)
  {}
  GameBase(GameBase const& that)
    : grid(that.grid)
    , snake(that.snake)
    , apple_pos(that.apple_pos)
  {}
  
  inline Coord snake_pos() const {
    return snake.front();
  }
  inline CoordRange dimensions() const {
    return grid.coords();
  }
};

class Game : public GameBase {
public:
  int turn = 0;
  enum class State {
    playing, loss, win
  } state = State::playing;
  enum class Event {
    none, move, eat, lose
  };
  
  // Flag to ignore snake self-collisions (for cheat agent)
  bool ignore_snake_collisions = false;
  
  // Added for flood fill debugging
  std::shared_ptr<FloodFillDebug> flood_fill_debug;
  
  inline bool win()  const { return state == State::win; }
  inline bool loss() const { return state == State::loss; }
  inline bool done() const { return state != State::playing; }
  
  Game(CoordRange dimensions, RNG const& rng = global_rng.next_rng(), bool ignore_snake_collisions = false);
  Game(Game const&) = delete;
  Event move(Dir dir);

private:
  RNG rng;
  
  Coord random_free_coord();
  int max_turns() const { return grid.size() * grid.size(); }
};

std::ostream& operator << (std::ostream& out, Game const& game);

//------------------------------------------------------------------------------
// Game
//------------------------------------------------------------------------------

#include <algorithm>

Game::Game(CoordRange dims, RNG const& base_rng, bool ignore_snake_collisions)
  : GameBase(dims)
  , rng(base_rng)
  , ignore_snake_collisions(ignore_snake_collisions)
{
  Coord start = dims.random(rng);
  snake.push_front(start);
  grid[start] = true;
  apple_pos = random_free_coord();
}

Coord Game::random_free_coord() {
  int n = grid.size() - snake.size();
  int pos = rng.random(n);
  for (auto c : grid.coords()) {
    if (!grid[c]) {
      if (pos == 0) return c;
      else pos--;
    }
  }
  throw std::runtime_error("no free coord");
}

Game::Event Game::move(Dir dir) {
  if (state != State::playing) return Event::none;
  turn++;
  Coord next = snake.front() + dir;
  if (!grid.valid(next) || (!ignore_snake_collisions && grid[next])) {
    state = State::loss;
    return Event::lose;
  }
  
  // lose after taking too long
  int max_turns = grid.size() * grid.size();
  if (turn > max_turns) {
    state = State::loss;
    return Event::lose;
  }
  // move
  snake.push_front(next);
  grid[next] = true;
  if (next == apple_pos) {
    // grow
    if (snake.size() == grid.size()) {
      state = State::win;
    } else {
      apple_pos = random_free_coord();
    }
    return Event::eat;
  } else {
    // remove tail
    grid[snake.back()] = false;
    snake.pop_back();
    return Event::move;
  }
}

//------------------------------------------------------------------------------
// Printing game state
//------------------------------------------------------------------------------

std::string white(std::string const& x) {
  return x;
}
std::string red(std::string const& x) {
  return "\033[31;1m" + x + "\033[0m";
}
std::string green(std::string const& x) {
  return "\033[32m" + x + "\033[0m";
}
std::string yellow(std::string const& x) {
  return "\033[33m" + x + "\033[0m";
}
std::string gray(std::string const& x) {
  return "\033[30;1m" + x + "\033[0m";
}

bool use_color = true;

template <typename Path, typename Color>
void draw_path(Grid<std::string>& grid, Path const& path, Color color, bool cycle=false) {
  for (auto it = path.begin(); it != path.end(); ++it) {
    Coord c = *it;
    if (!cycle && it == path.begin()) {
      grid[c] = color("■");
    } else {
      auto prev_it = it;
      if (prev_it == path.begin()) prev_it = path.end();
      --prev_it;
      auto next_it = it;
      ++next_it;
      if (next_it == path.end()) next_it = path.begin();
      Dir d = c - *prev_it;
      if (!cycle && next_it == path.begin()) {
        if (d == Dir::up)    grid[c] = color("╷");
        if (d == Dir::down)  grid[c] = color("╵");
        if (d == Dir::left)  grid[c] = color("╶");
        if (d == Dir::right) grid[c] = color("╴");
      } else {
        Dir e = *next_it - c;
        if (d == Dir::up) {
          if (e == Dir::up)    grid[c] = color("│");
          if (e == Dir::down)  grid[c] = color("│");
          if (e == Dir::left)  grid[c] = color("┐");
          if (e == Dir::right) grid[c] = color("┌");
        } else if (d == Dir::down) {
          if (e == Dir::up)    grid[c] = color("│");
          if (e == Dir::down)  grid[c] = color("│");
          if (e == Dir::left)  grid[c] = color("┘");
          if (e == Dir::right) grid[c] = color("└");
        } else if (d == Dir::left) {
          if (e == Dir::up)    grid[c] = color("└");
          if (e == Dir::down)  grid[c] = color("┌");
          if (e == Dir::left)  grid[c] = color("─");
          if (e == Dir::right) grid[c] = color("─");
        } else if (d == Dir::right) {
          if (e == Dir::up)    grid[c] = color("┘");
          if (e == Dir::down)  grid[c] = color("┐");
          if (e == Dir::left)  grid[c] = color("─");
          if (e == Dir::right) grid[c] = color("─");
        }
      }
    }
  }
}

void draw_snake(Grid<std::string>& grid, Snake const& snake, bool color = use_color) {
  draw_path(grid, snake, color ? green : white);
}

Grid<std::string> box_draw_grid(GameBase const& game, bool color = use_color) {
  Grid<std::string> grid(game.grid.coords(), "·");
  if (color) {
    for (int y=0; y<grid.h; y+=2) {
      for (int x=0; x<grid.w; x+=2) {
        grid[{x,  y  }] = gray("╭");
        grid[{x+1,y  }] = gray("╮");
        grid[{x,  y+1}] = gray("╰");
        grid[{x+1,y+1}] = gray("╯");
      }
    }
  }
  grid[game.apple_pos] = (color ? red : white)("●");
  draw_snake(grid, game.snake);
  return grid;
}

std::ostream& operator << (std::ostream& out, Grid<std::string> const& grid) {
  for (int y = 0; y < grid.h; ++y) {
    for (int x = 0; x < grid.w; ++x) {
      out << grid[{x,y}];
    }
    out << std::endl;
  }
  return out;
}

std::ostream& operator << (std::ostream& out, Game const& game) {
  out << "turn " << game.turn << ", size " << game.snake.size() << (game.win() ? " WIN!" : game.loss() ? " LOSS" : "") << std::endl;
  return out << box_draw_grid(game, use_color);
}

std::ostream& operator << (std::ostream& out, Grid<bool> const& grid) {
  Grid<std::string> vis(grid.dimensions());
  std::transform(grid.begin(), grid.end(), vis.begin(), [](bool x) {
    return x ? "#" : ".";
  });
  return out << vis;
}

// Static member definition
FloodFillDebug* FloodFillDebug::active_debug = nullptr;

