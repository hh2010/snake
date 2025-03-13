#pragma once

#include "util.hpp"
#include "random.hpp"
#include "shortest_path.hpp"
#include <string>
#include <variant>
#include <vector>
#include <random>
#include <memory>

//------------------------------------------------------------------------------
// Game state
//------------------------------------------------------------------------------

using Snake = RingBuffer<Coord>;

// State of a game of snake, without tracking anything else
class GameBase {
public:
  // Grid: true = contains part of the snake, false = empty
  Grid<bool> grid;
  // Coordinates that contain the snake, front = head of snake, back = tail of snake
  Snake snake;
  // Coordinate of the next goal/apple
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

// An actual game of snake
class Game : public GameBase {
  RNG rng;
  enum class State { playing, win, loss } state = State::playing;
  
public:
  enum class Event { none, move, eat, lose };
  
  std::shared_ptr<FloodFillDebug> flood_fill_debug;
  bool ignore_snake_collisions;
  int turn = 0;

  Game(CoordRange dims, RNG const& rng, bool ignore_snake_collisions = false) 
    : GameBase(dims)
    , rng(rng)
    , ignore_snake_collisions(ignore_snake_collisions) 
  {
    // Initialize snake
    Coord start = {dims.w/4, dims.h/2};
    snake.push_back(start);
    grid[start] = true;
    apple_pos = random_free_coord();
  }

  bool win() const { return state == State::win; }
  bool loss() const { return state == State::loss; }
  bool done() const { return state != State::playing; }
  
  Coord random_free_coord();
  Event move(Dir dir);

private:
  int max_turns() const { return grid.size() * grid.size(); }
};

std::ostream& operator << (std::ostream& out, Game const& game);

//------------------------------------------------------------------------------
// Game
//------------------------------------------------------------------------------

#include <algorithm>

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

