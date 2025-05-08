#pragma once

#include "game_util.hpp"
#include "cell_tree_utils.hpp"
#include <vector>

struct PathPlanningResult {
    std::vector<Coord>& path;
    Unreachables& unreachables;
    GameBase& after;

    
    PathPlanningResult(std::vector<Coord>& p, Unreachables& u, GameBase& a)
        : path(p), unreachables(u), after(a) {}
};