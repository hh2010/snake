#pragma once

#include "game_util.hpp"
#include "cell_tree_utils.hpp"
#include <vector>

struct PathPlanningResult {
    std::vector<Coord> path;
    Unreachables unreachable;
    
    PathPlanningResult(std::vector<Coord> p, Unreachables u)
        : path(p), unreachable(u) {}
};