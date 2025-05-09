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
        newGameState.snake.pop_back();
        return newGameState;
    }

    std::vector<Coord> createPathToDetourPos(const std::vector<Coord>& path, size_t detourIndex) {
        std::vector<Coord> pathToDetourPos;
        for (size_t j = path.size() - 1; j >= path.size() - 1 - detourIndex; j--) {
            pathToDetourPos.push_back(path[j]);
        }
        return pathToDetourPos;
    }

    void findDetours(
        const Game& game,
        std::vector<Coord>& resultPath,
        const std::function<int(Coord, Coord, Dir)>& edgeFunction
    ) {
        
        int totalExtraSteps = 0;
        int detourAttempts = 0;
        int detourFound = 0;

        auto detour_start_time = std::chrono::high_resolution_clock::now();
        
        // def need optimization here. not sure how to think about "keep detouring" vs "detour once, reconnect, then detour again"
        while (totalExtraSteps < extra_steps_desired) {
            bool foundDetour = false;
            detourAttempts++;
            
            std::cout << "  Searching for detour (iteration " << detourAttempts 
                      << "), current extra steps: " << totalExtraSteps << "/" << extra_steps_desired << std::endl;
            
            for (size_t i = 0; i < resultPath.size() - 1; i++) {
                if (i == resultPath.size() - 1) continue;
                
                Coord currentPos = resultPath[resultPath.size() - 1 - i];
                Coord nextPos = resultPath[resultPath.size() - 2 - i];
                
                std::cout << "    Checking position " << i << ": current=" << currentPos << ", next=" << nextPos << std::endl;
                
                std::vector<Coord> pathToDetourPos = createPathToDetourPos(resultPath, i);
                
                auto sim_start_time = std::chrono::high_resolution_clock::now();
                auto gameAtDetourPos = after_moves(game, pathToDetourPos, Lookahead::many_move_tail);
                auto cell_parents = cell_tree_parents(gameAtDetourPos.dimensions(), gameAtDetourPos.snake);
                auto sim_end_time = std::chrono::high_resolution_clock::now();
                PathExtensionTimer::simulation_time += sim_end_time - sim_start_time;
                
                bool foundDetourHere = tryFindDetour(
                    game, 
                    gameAtDetourPos, 
                    currentPos, 
                    nextPos, 
                    cell_parents, 
                    edgeFunction, 
                    resultPath, 
                    i, 
                    totalExtraSteps);
                    
                if (foundDetourHere) {
                    foundDetour = true;
                    detourFound++;
                    break;
                }
                
                std::cout << "    No valid detours found at this position" << std::endl;
            }
            
            if (!foundDetour) {
                std::cout << "  No more detours found after checking all positions" << std::endl;
                break;
            }
            
            if (totalExtraSteps >= extra_steps_desired) {
                std::cout << "  Reached desired extra steps: " << totalExtraSteps << "/" << extra_steps_desired << std::endl;
                break;
            }
        }
        
        auto detour_end_time = std::chrono::high_resolution_clock::now();
        PathExtensionTimer::detour_search_time += detour_end_time - detour_start_time;
        
        // Log the results of the detour search
        std::cout << "Detour search complete: " << detourFound << " detours found in " << detourAttempts << " iterations" << std::endl;
        std::cout << "Final path length: " << resultPath.size() << " (+" << totalExtraSteps << " steps)" << std::endl;
    }
    
    bool tryFindDetour(
        const Game& game,
        const GameBase& gameAtPos,
        Coord currentPos,
        Coord nextPos,
        const Grid<Coord>& cell_parents,
        const std::function<int(Coord, Coord, Dir)>& edgeFunction,
        std::vector<Coord>& resultPath,
        size_t positionIndex,
        int& totalExtraSteps) {
        
        int dirAttempts = 0;
        for (auto dir : dirs) {
            Coord detourPos = currentPos + dir;
            dirAttempts++;
            
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
            
            // double check that all connections are in the same order as original path? i think snake head is first?
            std::vector<Coord> newPath = buildNewPath(resultPath, positionIndex, reconnectPath);
            int extraStepsAdded = newPath.size() - resultPath.size();
            
            std::cout << "        New path length: " << newPath.size() 
                    << " (+" << extraStepsAdded << " steps)" << std::endl;
            
            totalExtraSteps += extraStepsAdded;
            resultPath = newPath;
            PathExtensionTimer::successful_detours++;
            
            std::cout << "    Found detour after " << dirAttempts << " direction attempts" << std::endl;
            return true;
        }
        
        return false;
    }
    
    PathPlanningResult evaluateExtendedPath(
        const Game& game,
        std::vector<Coord>& originalPath,
        std::vector<Coord>& extendedPath,
        Unreachables& originalUnreachable,
        GameBase& originalAfter,
        const std::function<int(Coord, Coord, Dir)>& edgeFunction) {
        
        auto eval_start_time = std::chrono::high_resolution_clock::now();
        
        auto afterExtended = after_moves(game, extendedPath, Lookahead::many_move_tail);
        auto extendedDists = astar_shortest_path(
            afterExtended.grid.coords(),
            edgeFunction,
            afterExtended.snake_pos(),
            game.apple_pos,
            1
        );
        auto extendedUnreachable = cell_tree_unreachables(afterExtended, extendedDists);
        
        auto eval_end_time = std::chrono::high_resolution_clock::now();
        PathExtensionTimer::simulation_time += eval_end_time - eval_start_time;
        
        int originalUnreachableCount = originalUnreachable.countUnreachableCells();
        int extendedUnreachableCount = extendedUnreachable.countUnreachableCells();

        std::cout << "Original unreachable cells: " << originalUnreachableCount << std::endl;
        std::cout << "Extended unreachable cells: " << extendedUnreachableCount << std::endl;
        
        if (extendedUnreachableCount < originalUnreachableCount) {
            std::cout << "Extended path reduces unreachable cells, returning extended path" << std::endl;
            return PathPlanningResult(extendedPath, extendedUnreachable, afterExtended);
        }
        
        std::cout << "Extended path does not improve unreachable cells, returning original path" << std::endl;
        return PathPlanningResult(originalPath, originalUnreachable, originalAfter);
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
        const std::vector<Coord>& originalPath, 
        size_t detourIndex, 
        const std::vector<Coord>& reconnectPath) {
            
        std::vector<Coord> newPath;
        
        // Keep all steps before detour point
        for (size_t j = originalPath.size() - 1; j >= originalPath.size() - detourIndex; j--) {
            newPath.push_back(originalPath[j]);
        }
        
        // Add reconnect path (which leads to next position)
        for (auto& step : reconnectPath) {
            newPath.push_back(step);
        }
        
        // Add all remaining steps after next position
        for (size_t j = originalPath.size() - detourIndex - 2; j > 0; j--) {
            newPath.push_back(originalPath[j]);
        }
        
        return newPath;
    }
    
    PathPlanningResult findExtendedPath(
        const Game& game, 
        std::vector<Coord>& originalPath,
        const std::function<int(Coord, Coord, Dir)>& edgeFunction,
        Unreachables& originalUnreachable,
        GameBase& originalAfter) {
            
        auto start_time = std::chrono::high_resolution_clock::now();
        PathExtensionTimer::call_count++;
        
        std::cout << "Turn " << game.turn << ": Finding extended path, desired extra steps: " << extra_steps_desired << std::endl;
        std::cout << "Original path size: " << originalPath.size() << std::endl;
        std::cout << "Original unreachable cells: " << originalUnreachable.countUnreachableCells() << std::endl;
        
        if (originalPath.size() <= 1) {
            std::cout << "Path too short, returning original" << std::endl;
            auto end_time = std::chrono::high_resolution_clock::now();
            PathExtensionTimer::total_time += end_time - start_time;
            return PathPlanningResult(originalPath, originalUnreachable, originalAfter);
        }
        
        std::vector<Coord> resultPath = originalPath;
        
        findDetours(game, resultPath, edgeFunction);        
        auto evaluationResult = evaluateExtendedPath(game, originalPath, resultPath, originalUnreachable, originalAfter, edgeFunction);

        auto end_time = std::chrono::high_resolution_clock::now();
        PathExtensionTimer::total_time += end_time - start_time;
        
        return evaluationResult;
    }
    
private:
    int extra_steps_desired;
};
