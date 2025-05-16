#ifndef CELL_TREE_UTILS_HPP
#define CELL_TREE_UTILS_HPP

#include "agent.hpp"
#include "game_util.hpp"
#include "shortest_path.hpp"

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

// Overload that takes a distance grid
Unreachables cell_tree_unreachables(GameBase const& game, Grid<Step> const& dists) {
  auto cell_parents = cell_tree_parents(game.dimensions(), game.snake);
  auto can_move = [&](Coord from, Coord to, Dir dir) {
    return can_move_in_cell_tree(cell_parents, from, to, dir) && !game.grid[to];
  };
  return unreachables(can_move, game, dists);
}

// Overload that doesn't require a distance grid
Unreachables cell_tree_unreachables(GameBase const& game) {
  auto cell_parents = cell_tree_parents(game.dimensions(), game.snake);
  auto can_move = [&](Coord from, Coord to, Dir dir) {
    return can_move_in_cell_tree(cell_parents, from, to, dir) && !game.grid[to];
  };
  return unreachables(can_move, game);
}

#endif // CELL_TREE_UTILS_HPP
