#pragma once

#include "game.hpp"
#include "game_util.hpp"
#include "shortest_path.hpp"
#include "util.hpp"
#include "cell_tree_utils.hpp"
#include "path_planning_result.hpp"
#include <chrono>
#include <iostream>

struct PathExtensionTimer {
    static std::chrono::nanoseconds total_time;
    static std::chrono::nanoseconds detour_search_time;
    static std::chrono::nanoseconds simulation_time;
    static std::chrono::nanoseconds reconnect_time;
    static int call_count;
    static int successful_detours;
    
    static void reset() {
        total_time = std::chrono::nanoseconds(0);
        detour_search_time = std::chrono::nanoseconds(0);
        simulation_time = std::chrono::nanoseconds(0);
        reconnect_time = std::chrono::nanoseconds(0);
        call_count = 0;
        successful_detours = 0;
    }
    
    static void print_stats() {
        if (call_count > 0) {
            double total_ms = std::chrono::duration<double, std::milli>(total_time).count();
            double detour_ms = std::chrono::duration<double, std::milli>(detour_search_time).count();
            double sim_ms = std::chrono::duration<double, std::milli>(simulation_time).count();
            double reconnect_ms = std::chrono::duration<double, std::milli>(reconnect_time).count();
            
            std::cout << "\nPath Extension timing stats:" << std::endl;
            std::cout << "  Total calls: " << call_count << std::endl;
            std::cout << "  Successful detours: " << successful_detours << std::endl;
            std::cout << "  Total time: " << total_ms << " ms" << std::endl;
            std::cout << "  Average time: " << total_ms / call_count << " ms per call" << std::endl;
            std::cout << "  Detour search time: " << detour_ms << " ms (" << (detour_ms / total_ms * 100) << "%)" << std::endl;
            std::cout << "  Game simulation time: " << sim_ms << " ms (" << (sim_ms / total_ms * 100) << "%)" << std::endl;
            std::cout << "  Path reconnection time: " << reconnect_ms << " ms (" << (reconnect_ms / total_ms * 100) << "%)" << std::endl;
        }
    }
};

std::chrono::nanoseconds PathExtensionTimer::total_time = std::chrono::nanoseconds(0);
std::chrono::nanoseconds PathExtensionTimer::detour_search_time = std::chrono::nanoseconds(0);
std::chrono::nanoseconds PathExtensionTimer::simulation_time = std::chrono::nanoseconds(0);
std::chrono::nanoseconds PathExtensionTimer::reconnect_time = std::chrono::nanoseconds(0);
int PathExtensionTimer::call_count = 0;
int PathExtensionTimer::successful_detours = 0;

class SnakePathPlanner {
public:
    SnakePathPlanner(int extraStepsDesired) : extra_steps_desired(extraStepsDesired) {}
    
    void setExtraStepsDesired(int extraStepsDesired) {
        extra_steps_desired = extraStepsDesired;
    }
    
    int getExtraStepsDesired() const {
        return extra_steps_desired;
    }
    
    // For logging the direction name
    std::string dir_name(Dir dir) {
        if (dir == Dir::up) return "up";
        if (dir == Dir::down) return "down";
        if (dir == Dir::left) return "left";
        if (dir == Dir::right) return "right";
        return "unknown";
    }
    
    // Check if a potential detour position is valid
    bool isValidDetour(
        const GameBase& gameState, 
        const Coord& currentPos, 
        const Coord& nextPos, 
        const Coord& detourPos, 
        Dir dir,
        const Grid<Coord>& cell_parents,
        const Coord& applePos) {
        
        return gameState.grid.valid(detourPos) && 
               !gameState.grid[detourPos] && 
               can_move_in_cell_tree(cell_parents, currentPos, detourPos, dir) &&
               detourPos != applePos &&
               detourPos != nextPos;
    }
    
    GameBase applyDetourMove(const GameBase& gameState, const Coord& detourPos) {
        auto newGameState = gameState;
        newGameState.snake.push_front(detourPos);
        newGameState.grid[detourPos] = true;
        newGameState.grid[newGameState.snake.back()] = false;
        newGameState.snake.pop_back();  // this should probably only be if lookahead == many_move_tail
        return newGameState;
    }

    std::vector<Coord> createPathToDetourPos(const std::vector<Coord>& path, const Coord& targetPos) {
        std::vector<Coord> pathToDetourPos;

        bool haveVisitedTarget = false;
        for (size_t j = 0; j < path.size(); j++) {
            if (haveVisitedTarget) {
                pathToDetourPos.push_back(path[j]);
            }
            if (path[j] == targetPos) {
                haveVisitedTarget = true;
            }
        }
        return pathToDetourPos;
    }

    std::vector<Coord> findDetours(
        const Game& game,
        std::vector<Coord> resultPath,
        const std::function<int(Coord, Coord, Dir)>& edgeFunction
    ) {
        
        int totalExtraSteps = 0;
        int detourAttempts = 0;
        int detourFound = 0;
        Coord nextPos = game.snake_pos();

        auto detour_start_time = std::chrono::high_resolution_clock::now();

        while (totalExtraSteps < extra_steps_desired) {
            bool foundDetour = false;
            detourAttempts++;
            std::vector<Coord> lastCompletePath = resultPath;  // lot of vector copies. can we do other ways? and dont love the variable names
            
            std::cout << "  Searching for detour (iteration " << detourAttempts 
                      << "), current extra steps: " << totalExtraSteps << "/" << extra_steps_desired << std::endl;
            
            for (size_t i = 0; (i <= lastCompletePath.size() - 1) && totalExtraSteps < extra_steps_desired; ++i) {
                std::cout << totalExtraSteps << " / " << extra_steps_desired << std::endl;
                if (i == lastCompletePath.size() - 1) continue;

                Coord currentPos = nextPos;
                nextPos = lastCompletePath[lastCompletePath.size() - 1 - i];

                std::cout << "    Checking position " << i << ": current=" << currentPos << ", next=" << nextPos << std::endl;
                std::vector<Coord> pathToDetourPos = createPathToDetourPos(resultPath, nextPos);
                
                auto sim_start_time = std::chrono::high_resolution_clock::now();
                auto afterAtDetourPos = (i > 0) ? after_moves(game, pathToDetourPos, Lookahead::many_move_tail) : GameBase(game);
                auto cell_parents = cell_tree_parents(afterAtDetourPos.dimensions(), afterAtDetourPos.snake);
                auto sim_end_time = std::chrono::high_resolution_clock::now();
                PathExtensionTimer::simulation_time += sim_end_time - sim_start_time;
                
                bool foundDetourHere = tryFindDetour(
                    game,
                    afterAtDetourPos,
                    currentPos,
                    nextPos,
                    cell_parents,
                    edgeFunction,
                    pathToDetourPos,
                    resultPath,
                    i,
                    totalExtraSteps);

                if (foundDetourHere) {
                    foundDetour = true;
                    detourFound++;
                    std::cout << "    Found detour at position " << i << std::endl;
                }
                
                std::cout << "    No valid detours found at this position" << std::endl;
            }
            
            if (!foundDetour) {
                std::cout << "  No more detours found after checking all positions" << std::endl;
                break;
            }
        }
        
        auto detour_end_time = std::chrono::high_resolution_clock::now();
        PathExtensionTimer::detour_search_time += detour_end_time - detour_start_time;
        
        // Log the results of the detour search
        std::cout << "Detour search complete: " << detourFound << " detours found in " << detourAttempts << " iterations" << std::endl;
        std::cout << "Final path length: " << resultPath.size() << " (+" << totalExtraSteps << " steps)" << std::endl;

        return resultPath;
    }
    
    bool tryFindDetour(
        const Game& game,
        const GameBase& gameAtPos,
        Coord currentPos,
        Coord nextPos,
        const Grid<Coord>& cell_parents,
        const std::function<int(Coord, Coord, Dir)>& edgeFunction,
        std::vector<Coord>& pathToDetourPos,
        std::vector<Coord>& resultPath,
        size_t positionIndex,
        int& totalExtraSteps) {
        
        for (auto dir : dirs) {
            Coord detourPos = currentPos + dir;
            
            std::cout << "      Trying direction " << dir_name(dir) << " to " << detourPos;
            
            if (!isValidDetour(gameAtPos, currentPos, nextPos, detourPos, dir, cell_parents, game.apple_pos)) {
                std::cout << " - Invalid move" << std::endl;
                continue;
            }
            
            auto gameAfterDetour = applyDetourMove(gameAtPos, detourPos);
            
            std::cout << " - Looking for reconnect path to " << nextPos << std::endl;
            
            auto reconnect_start_time = std::chrono::high_resolution_clock::now();
            // we may need a different edge function here? not sure its compatible with cell-variant
            auto reconnectPath = findReconnectPath(gameAfterDetour, detourPos, nextPos, edgeFunction);
            auto reconnect_end_time = std::chrono::high_resolution_clock::now();
            PathExtensionTimer::reconnect_time += reconnect_end_time - reconnect_start_time;
            
            if (reconnectPath.empty() || reconnectPath.back() == INVALID) {
                std::cout << "        No valid reconnect path found" << std::endl;
                continue;
            }
            
            std::cout << "        Found reconnect path of length " << reconnectPath.size() << std::endl;
            
            std::vector<Coord> newPath = buildNewPath(resultPath, reconnectPath, detourPos, pathToDetourPos);
            int extraStepsAdded = newPath.size() - resultPath.size();
            
            std::cout << "        New path length: " << newPath.size() 
                    << " (+" << extraStepsAdded << " steps)" << std::endl;
            std::cout << newPath << std::endl;
            
            if (extraStepsAdded > 0) {
                std::cout << "        Detour added " << extraStepsAdded << " extra steps" << std::endl;
                totalExtraSteps += extraStepsAdded;
                resultPath = newPath;
                PathExtensionTimer::successful_detours++;
            }
            else {
                continue;
            }
            
            return true;
        }
        
        return false;
    }
    
    PathPlanningResult evaluateExtendedPath(
        const Game& game,
        std::vector<Coord>& originalPath,
        std::vector<Coord>& extendedPath,
        Unreachables& originalUnreachable,
            const std::function<int(Coord, Coord, Dir)>& edgeFunction) {
        
        auto eval_start_time = std::chrono::high_resolution_clock::now();
        
        auto afterExtended = after_moves(game, extendedPath, Lookahead::many_move_tail);
        auto extendedUnreachable = cell_tree_unreachables(afterExtended);
        
        auto eval_end_time = std::chrono::high_resolution_clock::now();
        PathExtensionTimer::simulation_time += eval_end_time - eval_start_time;
        
        int originalUnreachableCount = originalUnreachable.countUnreachableCells();
        int extendedUnreachableCount = extendedUnreachable.countUnreachableCells();

        std::cout << "Original unreachable cells: " << originalUnreachableCount << std::endl;
        std::cout << "Extended unreachable cells: " << extendedUnreachableCount << std::endl;
        
        if (extendedUnreachableCount < originalUnreachableCount) {
            std::cout << "Extended path reduces unreachable cells, returning extended path" << std::endl;
            return PathPlanningResult(extendedPath, extendedUnreachable);
        }
        
        std::cout << "Extended path does not improve unreachable cells, returning original path" << std::endl;
        return PathPlanningResult(originalPath, originalUnreachable);
    }
    
    std::vector<Coord> findReconnectPath(
        const GameBase& gameState, 
        Coord fromPos, 
        Coord toPos,
        const std::function<int(Coord, Coord, Dir)>& edgeFunction) {
            
        auto reconnectDists = astar_shortest_path(
            gameState.grid.coords(), 
            edgeFunction, 
            fromPos, 
            toPos, 
            1000);
            
        return read_path(reconnectDists, fromPos, toPos);
    }
    
    std::vector<Coord> buildNewPath(
        const std::vector<Coord>& resultPath, 
        const std::vector<Coord>& reconnectPath,
        Coord detourPos,
        std::vector<Coord>& pathToDetourPos) {

        std::vector<Coord> newPath;
        std::cout << resultPath << std::endl;
        
        // Add elements from resultPath until we find reconnectPath[0], but don't include it
        for (const Coord& step : resultPath) {
            if (!reconnectPath.empty() && step == reconnectPath[0]) {
                // Stop when we find the first coordinate of reconnectPath
                break;
            }
            newPath.push_back(step);
        }

        // Add reconnect path (which leads to next position)
        for (auto& step : reconnectPath) {
            newPath.push_back(step);
        }

        newPath.push_back(detourPos);

        // Keep all steps before detour point
        for (const Coord& step : pathToDetourPos) {
            newPath.push_back(step);
        }
        return newPath;
    }

    PathPlanningResult findExtendedPath(
        const Game& game,
        std::vector<Coord>& originalPath,
        const std::function<int(Coord, Coord, Dir)>& edgeFunction,
        Unreachables& originalUnreachable) {

        if (extra_steps_desired == 0) {
            std::cout << "No extra steps desired, returning original path" << std::endl;
            return PathPlanningResult(originalPath, originalUnreachable);
        }

        auto start_time = std::chrono::high_resolution_clock::now();
        PathExtensionTimer::call_count++;
        
        std::cout << "Turn " << game.turn << ": Finding extended path, desired extra steps: " << extra_steps_desired << std::endl;
        std::cout << "Original path size: " << originalPath.size() << std::endl;
        std::cout << "Original unreachable cells: " << originalUnreachable.countUnreachableCells() << std::endl;

        if (originalPath.size() <= 1) {
            std::cout << "Path too short, returning original" << std::endl;
            auto end_time = std::chrono::high_resolution_clock::now();
            PathExtensionTimer::total_time += end_time - start_time;
            return PathPlanningResult(originalPath, originalUnreachable);
        }
        
        std::vector<Coord> resultPath = findDetours(game, originalPath, edgeFunction);
        PathPlanningResult evaluationResult = evaluateExtendedPath(game, originalPath, resultPath, originalUnreachable, edgeFunction);

        auto end_time = std::chrono::high_resolution_clock::now();
        PathExtensionTimer::total_time += end_time - start_time;
        
        return evaluationResult;
    }
    
private:
    int extra_steps_desired;
};
