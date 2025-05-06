#pragma once

#include "game.hpp"
#include "game_util.hpp"
#include "shortest_path.hpp"
#include "util.hpp"

class SnakePathPlanner {
public:
    SnakePathPlanner(int extraStepsDesired) : extra_steps_desired(extraStepsDesired) {}
    
    void setExtraStepsDesired(int extraStepsDesired) {
        extra_steps_desired = extraStepsDesired;
    }
    
    int getExtraStepsDesired() const {
        return extra_steps_desired;
    }
    
    GameBase simulateGameAtPosition(const Game& game, const std::vector<Coord>& pathToPos) {
        return after_moves(game, pathToPos, Lookahead::many_move_tail);
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
    
    std::vector<Coord> findExtendedPath(
        const Game& game, 
        const std::vector<Coord>& originalPath,
        const std::function<int(Coord, Coord, Dir)>& edgeFunction) {
            
        if (originalPath.size() <= 1) {
            return originalPath;
        }
        
        std::vector<Coord> resultPath = originalPath;
        int totalExtraSteps = 0;
        
        while (totalExtraSteps < extra_steps_desired) {
            bool foundDetour = false;
            
            for (size_t i = 0; i < resultPath.size() - 1; i++) {
                if (i == resultPath.size() - 1) continue;
                
                Coord currentPos = resultPath[resultPath.size() - 1 - i];
                Coord nextPos = resultPath[resultPath.size() - 2 - i];
                
                std::vector<Coord> pathToDetourPos;
                for (size_t j = resultPath.size() - 1; j >= resultPath.size() - 1 - i; j--) {
                    pathToDetourPos.push_back(resultPath[j]);
                }
                
                auto gameAtDetourPos = simulateGameAtPosition(game, pathToDetourPos);
                auto cell_parents = cell_tree_parents(gameAtDetourPos.dimensions(), gameAtDetourPos.snake);
                
                for (auto dir : dirs) {
                    Coord detourPos = currentPos + dir;
                    
                    if (!gameAtDetourPos.grid.valid(detourPos) || 
                        gameAtDetourPos.grid[detourPos] || 
                        !can_move_in_cell_tree(cell_parents, currentPos, detourPos, dir) ||
                        detourPos == game.apple_pos ||
                        detourPos == nextPos) {
                        continue;
                    }
                    
                    auto gameAfterDetour = gameAtDetourPos;
                    gameAfterDetour.snake.push_front(detourPos);
                    gameAfterDetour.grid[detourPos] = true;
                    gameAfterDetour.grid[gameAfterDetour.snake.back()] = false;
                    gameAfterDetour.snake.pop_back();
                    
                    auto reconnectPath = findReconnectPath(gameAfterDetour, detourPos, nextPos, edgeFunction);
                    
                    if (reconnectPath.empty() || reconnectPath.back() == INVALID) {
                        continue;
                    }
                    
                    std::vector<Coord> newPath = buildNewPath(resultPath, i, reconnectPath);
                    
                    int extraStepsAdded = newPath.size() - resultPath.size();
                    
                    totalExtraSteps += extraStepsAdded;
                    resultPath = newPath;
                    foundDetour = true;
                    
                    break;
                }
                
                if (foundDetour) {
                    break;
                }
            }
            
            if (!foundDetour || totalExtraSteps >= extra_steps_desired) {
                break;
            }
        }
        
        if (totalExtraSteps >= extra_steps_desired) {
            auto afterExtended = after_moves(game, resultPath, Lookahead::many_move_tail);
            auto extendedDists = shortest_path(afterExtended.grid, afterExtended.snake_pos());
            auto extendedUnreachable = cell_tree_unreachables(afterExtended, extendedDists);
            
            if (!extendedUnreachable.any) {
                return resultPath;
            }
        }
        
        return originalPath;
    }
    
private:
    int extra_steps_desired;
};
