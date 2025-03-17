#pragma once
#include "util.hpp"
#include <vector>
#include <functional>
#include <algorithm>

//------------------------------------------------------------------------------
// Differential Connectivity using DSU (Disjoint Set Union)
//------------------------------------------------------------------------------

// A Disjoint Set Union (DSU) implementation with path compression and union-by-size
// Used for efficiently tracking connected components in a grid
class DisjointSetUnion {
private:
    std::vector<int> parent;  // Parent of each node
    std::vector<int> size;    // Size of each set (used for union-by-size)
    int components;           // Number of connected components

public:
    // Initialize DSU with n disjoint elements
    DisjointSetUnion(int n) : parent(n), size(n, 1), components(n) {
        for (int i = 0; i < n; ++i) {
            parent[i] = i;  // Each element is its own parent initially
        }
    }

    // Find the representative of the set containing element x (with path compression)
    int find(int x) {
        if (parent[x] != x) {
            parent[x] = find(parent[x]);  // Path compression
        }
        return parent[x];
    }

    // Union the sets containing elements x and y (union-by-size)
    void unite(int x, int y) {
        int rootX = find(x);
        int rootY = find(y);

        if (rootX != rootY) {
            // Union by size: attach smaller tree under root of larger tree
            if (size[rootX] < size[rootY]) {
                std::swap(rootX, rootY);
            }
            parent[rootY] = rootX;
            size[rootX] += size[rootY];
            --components;  // Decrease component count when merging
        }
    }

    // Check if elements x and y are in the same set
    bool connected(int x, int y) {
        return find(x) == find(y);
    }

    // Get the size of the set containing element x
    int getSize(int x) {
        return size[find(x)];
    }

    // Get the total number of connected components
    int getComponentCount() const {
        return components;
    }
};

// Helper function to convert a 2D coordinate to a 1D index
inline int coordToIndex(const Coord& c, int width) {
    return c.y * width + c.x;
}

// Helper function to convert a 1D index to a 2D coordinate
inline Coord indexToCoord(int idx, int width) {
    return Coord{idx % width, idx / width};
}

//------------------------------------------------------------------------------
// DifferentialConnectivity: Track grid connectivity with efficient updates
//------------------------------------------------------------------------------

class DifferentialConnectivity {
private:
    int width;
    int height;
    std::vector<bool> isOccupied;  // Grid to track occupied cells
    DisjointSetUnion dsu;          // DSU for connectivity tracking
    
    // Directions for adjacent cell checks
    const std::vector<Dir> directions = {Dir::up, Dir::right, Dir::down, Dir::left};

    // Check if a coordinate is valid within the grid
    bool isValidCoord(const Coord& c) const {
        return c.x >= 0 && c.x < width && c.y >= 0 && c.y < height;
    }
    
    // Connect adjacent cells if they're both free
    void connectAdjacentCells(const Coord& c) {
        int idx = coordToIndex(c, width);
        
        for (const auto& dir : directions) {
            Coord adj = c + dir;
            if (isValidCoord(adj) && !isOccupied[coordToIndex(adj, width)]) {
                dsu.unite(idx, coordToIndex(adj, width));
            }
        }
    }
    
    // Update DSU connectivity in a region around the changed cell
    void updateRegionConnectivity(const Coord& center) {
        // For cells nearby the changed cell, we need to rebuild their connectivity
        // This is a bounded version of the flood fill but using DSU
        
        // We'll rebuild connections for all adjacent free cells
        for (const auto& dir : directions) {
            Coord adj = center + dir;
            if (!isValidCoord(adj) || isOccupied[coordToIndex(adj, width)]) continue;
            
            // For each free adjacent cell, reconnect it to its free neighbors
            int adjIdx = coordToIndex(adj, width);
            for (const auto& dir2 : directions) {
                Coord nbr = adj + dir2;
                if (isValidCoord(nbr) && !isOccupied[coordToIndex(nbr, width)]) {
                    dsu.unite(adjIdx, coordToIndex(nbr, width));
                }
            }
        }
    }

public:
    // Initialize with grid dimensions and an obstacle predicate
    DifferentialConnectivity(CoordRange dims, std::function<bool(Coord)> isObstacleFn) 
        : width(dims.w), height(dims.h), 
          isOccupied(dims.w * dims.h, false), 
          dsu(dims.w * dims.h) {
        
        // Initialize grid state
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                Coord c{x, y};
                isOccupied[coordToIndex(c, width)] = isObstacleFn(c);
            }
        }
        
        // Initialize DSU connectivity
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                Coord c{x, y};
                int idx = coordToIndex(c, width);
                
                if (!isOccupied[idx]) {
                    // Connect to free neighbors (right and down are sufficient for initialization)
                    if (x+1 < width && !isOccupied[coordToIndex({x+1, y}, width)]) {
                        dsu.unite(idx, coordToIndex({x+1, y}, width));
                    }
                    if (y+1 < height && !isOccupied[coordToIndex({x, y+1}, width)]) {
                        dsu.unite(idx, coordToIndex({x, y+1}, width));
                    }
                }
            }
        }
    }
    
    // Update connectivity after a move: mark oldTail free, mark newHead occupied
    void updateForMove(Coord oldTail, Coord newHead, std::function<bool(Coord)> isObstacleFn) {
        // Make sure we're not working with invalid coordinates
        if (!isValidCoord(oldTail) || !isValidCoord(newHead)) {
            return;
        }
        
        // Mark old tail as free
        int oldTailIdx = coordToIndex(oldTail, width);
        isOccupied[oldTailIdx] = false;
        
        // Connect old tail to free adjacent cells
        connectAdjacentCells(oldTail);
        
        // Update region around old tail
        updateRegionConnectivity(oldTail);
        
        // Mark new head as occupied
        int newHeadIdx = coordToIndex(newHead, width);
        isOccupied[newHeadIdx] = true;
        
        // Update region around new head
        updateRegionConnectivity(newHead);
    }
    
    // Get size of connected component containing cell c
    int componentSize(Coord c) {
        if (!isValidCoord(c) || isOccupied[coordToIndex(c, width)]) {
            return 0;  // Invalid or occupied cells have no connected component
        }
        return dsu.getSize(coordToIndex(c, width));
    }
    
    // Calculate penalty for a move based on connectivity impact
    int penaltyForMove(Coord pos, Dir dir, int threshold, int penaltyCost) {
        Coord nextPos = pos + dir;
        
        // Check if the move is valid
        if (!isValidCoord(nextPos)) {
            return INT_MAX;  // Invalid move
        }
        
        // Temporary simulation of the move
        int nextPosIdx = coordToIndex(nextPos, width);
        if (isOccupied[nextPosIdx]) {
            return INT_MAX;  // Cell already occupied
        }
        
        // Find an adjacent free cell to check connectivity
        // Mark the new position as occupied temporarily
        isOccupied[nextPosIdx] = true;
        
        // Find a free cell adjacent to the next position
        Coord checkCell = INVALID;
        for (const auto& checkDir : directions) {
            Coord adj = nextPos + checkDir;
            if (isValidCoord(adj) && !isOccupied[coordToIndex(adj, width)]) {
                checkCell = adj;
                break;
            }
        }
        
        // If no free adjacent cell found, use another free cell in the grid
        if (checkCell == INVALID) {
            for (int y = 0; y < height && checkCell == INVALID; ++y) {
                for (int x = 0; x < width && checkCell == INVALID; ++x) {
                    Coord c{x, y};
                    if (!isOccupied[coordToIndex(c, width)] && !(c == nextPos)) {
                        checkCell = c;
                    }
                }
            }
        }
        
        int penalty = 0;
        if (checkCell != INVALID) {
            // Save the current state of the DSU
            DisjointSetUnion tempDSU = dsu;
            
            // Update connectivity with nextPos as occupied
            std::vector<Coord> affectedCells;
            for (const auto& tempDir : directions) {
                Coord adj = nextPos + tempDir;
                if (isValidCoord(adj) && !isOccupied[coordToIndex(adj, width)]) {
                    affectedCells.push_back(adj);
                }
            }
            
            // Connect all valid free neighbors of affected cells
            for (const auto& cell : affectedCells) {
                int cellIdx = coordToIndex(cell, width);
                for (const auto& tempDir : directions) {
                    Coord nbr = cell + tempDir;
                    if (isValidCoord(nbr) && !isOccupied[coordToIndex(nbr, width)] && nbr != nextPos) {
                        tempDSU.unite(cellIdx, coordToIndex(nbr, width));
                    }
                }
            }
            
            // Check component size
            int compSize = tempDSU.getSize(coordToIndex(checkCell, width));
            if (compSize < threshold) {
                penalty = penaltyCost;
            }
        }
        
        // Restore the grid state
        isOccupied[nextPosIdx] = false;
        
        return penalty;
    }
    
    // Check if two cells are connected (in the same component)
    bool areConnected(Coord a, Coord b) {
        if (!isValidCoord(a) || !isValidCoord(b) || 
            isOccupied[coordToIndex(a, width)] || isOccupied[coordToIndex(b, width)]) {
            return false;
        }
        return dsu.connected(coordToIndex(a, width), coordToIndex(b, width));
    }
    
    // Get the total number of free cells in the grid
    int totalFreeCells() const {
        return std::count(isOccupied.begin(), isOccupied.end(), false);
    }
};