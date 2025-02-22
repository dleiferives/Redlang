// astar_solver.cpp
#include <algorithm>
#include <array>
#include <exception>
#include <functional>
#include <iostream>
#include <optional>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cmath>

// ---------------------------------------------------------------------
// Type definitions
// ---------------------------------------------------------------------

// We use a 3-element array of ints for 3D coordinates.
using Coord3 = std::array<int, 3>;

// Provide a hash functor for Coord3 so it can be used in unordered_map/unordered_set.
struct Coord3Hash {
    std::size_t operator()(const Coord3 &c) const {
        std::size_t h1 = std::hash<int>()(c[0]);
        std::size_t h2 = std::hash<int>()(c[1]);
        std::size_t h3 = std::hash<int>()(c[2]);
        return ((h1 * 31 + h2) * 31) + h3;
    }
};

// ---------------------------------------------------------------------
// Exception classes
// ---------------------------------------------------------------------

class OutOfBoundsError : public std::runtime_error {
public:
    OutOfBoundsError(const std::string &msg) : std::runtime_error(msg) {}
};

class NoPathFoundError : public std::runtime_error {
public:
    NoPathFoundError(const std::string &msg) : std::runtime_error(msg) {}
};

// ---------------------------------------------------------------------
// Cell type definitions
// ---------------------------------------------------------------------

// The types of cells supported.
enum class CellType { Air, Block, Integrated };

// The integrated path “info” (our interpretation of the Python tuple).
struct IntegratedPathInfo {
    // endpoints of the integrated path
    std::pair<Coord3, Coord3> pid;
    // The integrated state. For clarity, we define an enum here.
    enum class State { TrueState, FalseState, Top, NoneState };
    State state;
    Coord3 nneb;              // a designated neighboring cell
    std::vector<Coord3> path;  // the integrated route
};

// A cell “kind” as returned by getKind.
struct CellInfo {
    CellType type = CellType::Air;
    // Only valid when type == Integrated.
    std::optional<IntegratedPathInfo> integrated;
};

// ---------------------------------------------------------------------
// A* Solver class
// ---------------------------------------------------------------------

class AStarSolver {
public:
    // The getKind function takes three ints and returns a CellInfo.
    using GetKindFunc = std::function<CellInfo(int, int, int)>;

    // bounds: a pair {minBounds, maxBounds} using Coord3.
    AStarSolver(const std::pair<Coord3, Coord3> &bounds, GetKindFunc getKind)
        : minBounds(bounds.first), maxBounds(bounds.second),
          getKindFunc(getKind)
    {
        // Pre‐define allowed moves.
        // Horizontal moves.
        horizontalMoves = { { {1, 0, 0} }, { {-1, 0, 0} },
                            { {0, 1, 0} }, { {0, -1, 0} } };
        // Staircase moves upward (requires a horizontal change).
        staircaseMovesUp = { { {1, 0, 1} }, { {-1, 0, 1} },
                             { {0, 1, 1} }, { {0, -1, 1} } };
        // Staircase moves downward.
        staircaseMovesDown = { { {1, 0, -1} }, { {-1, 0, -1} },
                               { {0, 1, -1} }, { {0, -1, -1} } };

        // Combine all allowed moves.
        allMoves.insert(allMoves.end(), horizontalMoves.begin(), horizontalMoves.end());
        allMoves.insert(allMoves.end(), staircaseMovesUp.begin(), staircaseMovesUp.end());
        allMoves.insert(allMoves.end(), staircaseMovesDown.begin(), staircaseMovesDown.end());
    }

    // Reset the solver’s caches.
    void resetCaches() {
        invalidLut.clear();
        invalidPathLut.clear();
        clearanceLut.clear();
        kindLut.clear();
    }

    // A* search. If maxIterations == -1 then a very large limit is used.
    // On success returns the path from start to goal (inclusive);
    // If no path is found, throws NoPathFoundError.
    std::vector<Coord3> solve(const Coord3 &start, const Coord3 &goal,
                              long long maxIterations = -1) {
        resetCaches();
        long long iterations = (maxIterations == -1) ? 100000000000000LL : maxIterations;

        if (!inBounds(start))
            throw OutOfBoundsError("Start position is out of bounds");
        if (!inBounds(goal))
            throw OutOfBoundsError("Goal position is out of bounds");

        // Define a node for the priority queue.
        struct Node {
            int priority;
            int cost;
            Coord3 pos;
            // We reverse the comparison so that std::priority_queue gives us the smallest priority.
            bool operator>(const Node &other) const {
                return priority > other.priority;
            }
        };

        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
        openSet.push({heuristic(start, goal), 0, start});
        // "cameFrom" is used for path reconstruction.
        std::unordered_map<Coord3, Coord3, Coord3Hash> cameFrom;
        cameFrom[start] = start;
        std::unordered_map<Coord3, int, Coord3Hash> costSoFar;
        costSoFar[start] = 0;

        long long itCounter = 0;
        while (!openSet.empty()) {
            if (itCounter > iterations)
                return {}; // or throw an exception if preferred
            itCounter++;

            Node currentNode = openSet.top();
            openSet.pop();
            Coord3 current = currentNode.pos;
            int currentCost = currentNode.cost;

            if (current == goal) {
                // Reconstruct the path.
                std::vector<Coord3> path;
                Coord3 p = goal;
                while (!(p == start)) {
                    path.push_back(p);
                    p = cameFrom[p];
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                return path;
            }

            for (const auto &move : allMoves) {
                // Count each neighbor-check toward iterations.
                if (itCounter > iterations)
                    return {};
                itCounter++;

                std::vector<std::pair<Coord3, int>> extraPaths;
                Coord3 neighbor = { current[0] + move[0],
                                    current[1] + move[1],
                                    current[2] + move[2] };

                if (invalidLut.find(neighbor) != invalidLut.end())
                    continue;
                if (invalidPathLut.find(neighbor) != invalidPathLut.end())
                    continue;
                if (!inBounds(neighbor)) {
                    invalidLut.insert(neighbor);
                    continue;
                }

                // For endpoints (start/goal) clearance is not enforced.
                if (!(neighbor == start || neighbor == goal)) {
                    CellInfo kind = getKind(neighbor);
                    if (kind.type == CellType::Block)
                        continue;
                    if (kind.type == CellType::Air) {
                        if (!hasClearance(neighbor, start, goal)) {
                            invalidLut.insert(neighbor);
                            continue;
                        }
                        extraPaths.push_back({neighbor, 0});
                    } else if (kind.type == CellType::Integrated) {
                        // Process the integrated path branch.
                        auto &info = kind.integrated.value();
                        if (info.pid.second == goal) {
                            int ncst = 0;
                            if (info.state == IntegratedPathInfo::State::TrueState) {
                                if (!inVector(info.path, info.nneb)) {
                                    invalidPathLut.insert(info.nneb);
                                    continue;
                                }
                            } else if (info.state == IntegratedPathInfo::State::FalseState ||
                                       info.state == IntegratedPathInfo::State::Top) {
                                invalidPathLut.insert(info.nneb);
                                continue;
                            } else { // NoneState
                                if (!inVector(info.path, info.nneb)) {
                                    invalidPathLut.insert(info.nneb);
                                    continue;
                                } else {
                                    ncst = 1;
                                    extraPaths.push_back({neighbor, 0});
                                }
                            }
                            int nindex = 0;
                            for (int i = 0; i < static_cast<int>(info.path.size()); i++) {
                                if (info.path[i] == info.nneb) {
                                    nindex = i;
                                    break;
                                }
                            }
                            for (int i = nindex; i < static_cast<int>(info.path.size()); i++) {
                                extraPaths.push_back({info.path[i], i + ncst});
                            }
                        }
                        // Otherwise, if the integrated path begins with the start:
                        else if (info.pid.first == start) {
                            if (info.state == IntegratedPathInfo::State::TrueState ||
                                info.state == IntegratedPathInfo::State::NoneState)
                                extraPaths.push_back({neighbor, 0});
                        }
                    }
                    for (const auto &alt : extraPaths) {
                        int extraCost = alt.second;
                        int newCost = costSoFar[current] + 1 + extraCost;
                        Coord3 altPos = alt.first;
                        if (costSoFar.find(altPos) == costSoFar.end() ||
                            newCost < costSoFar[altPos])
                        {
                            costSoFar[altPos] = newCost;
                            int priority = newCost + heuristic(altPos, goal);
                            openSet.push({priority, newCost, altPos});
                            cameFrom[altPos] = current;
                        }
                    }
                } else { // if neighbor is start or goal
                    int newCost = costSoFar[current] + 1;
                    if (costSoFar.find(neighbor) == costSoFar.end() ||
                        newCost < costSoFar[neighbor])
                    {
                        costSoFar[neighbor] = newCost;
                        int priority = newCost + heuristic(neighbor, goal);
                        openSet.push({priority, newCost, neighbor});
                        cameFrom[neighbor] = current;
                    }
                }
            } // end for each allowed move
        }

        throw NoPathFoundError("No valid path found from start to goal");
    }

private:
    Coord3 minBounds, maxBounds;
    GetKindFunc getKindFunc;

    // Allowed moves.
    std::vector<Coord3> horizontalMoves;
    std::vector<Coord3> staircaseMovesUp;
    std::vector<Coord3> staircaseMovesDown;
    std::vector<Coord3> allMoves;

    // Caches.
    std::unordered_map<Coord3, CellInfo, Coord3Hash> kindLut;
    std::unordered_map<Coord3, bool, Coord3Hash> clearanceLut;
    std::unordered_set<Coord3, Coord3Hash> invalidLut;
    std::unordered_set<Coord3, Coord3Hash> invalidPathLut;

    // Checks whether a position is within the bounding box.
    bool inBounds(const Coord3 &p) const {
        return (p[0] >= minBounds[0] && p[0] <= maxBounds[0] &&
                p[1] >= minBounds[1] && p[1] <= maxBounds[1] &&
                p[2] >= minBounds[2] && p[2] <= maxBounds[2]);
    }

    // Manhattan distance heuristic.
    int heuristic(const Coord3 &a, const Coord3 &b) const {
        return std::abs(a[0] - b[0]) +
               std::abs(a[1] - b[1]) +
               std::abs(a[2] - b[2]);
    }

    // Returns true if a given position is in a vector.
    bool inVector(const std::vector<Coord3> &vec, const Coord3 &p) const {
        return std::find(vec.begin(), vec.end(), p) != vec.end();
    }

    // Returns the cell “kind” at the given position (caching the result).
    CellInfo getKind(const Coord3 &p) {
        auto it = kindLut.find(p);
        if (it != kindLut.end())
            return it->second;
        CellInfo kind = getKindFunc(p[0], p[1], p[2]);
        kindLut[p] = kind;
        return kind;
    }

    // Returns true if every neighbor (in the six cardinal directions)
    // is “clear” (i.e. of type Air) — except if the neighbor equals start or goal.
    bool hasClearance(const Coord3 &pos, const Coord3 &start, const Coord3 &goal,
                      std::unordered_set<Coord3, Coord3Hash> ignore = {}) {
        // First, add start/goal neighbors to ignore.
        int x = pos[0], y = pos[1], z = pos[2];
        std::vector<Coord3> dirs = { {0, 0, 1}, {0, 0, -1},
                                     {1, 0, 0}, {-1, 0, 0},
                                     {0, 1, 0}, {0, -1, 0} };
        for (const auto &d : dirs) {
            Coord3 neighbor = { x + d[0], y + d[1], z + d[2] };
            if (ignore.find(neighbor) != ignore.end())
                continue;
            if (neighbor == start || neighbor == goal)
                ignore.insert(neighbor);
        }
        // Then check whether each neighbor is clear.
        for (const auto &d : dirs) {
            Coord3 neighbor = { x + d[0], y + d[1], z + d[2] };
            if (ignore.find(neighbor) != ignore.end())
                continue;
            if (!inBounds(neighbor)) {
                clearanceLut[pos] = false;
                return false;
            }
            CellInfo kind = getKind(neighbor);
            if (kind.type == CellType::Block) {
                clearanceLut[pos] = false;
                return false;
            }
            if (kind.type == CellType::Air) {
                bool flag = true;
                if (ignore.empty()) {
                    std::vector<Coord3> innerDirs = { {0, 0, 0}, {0, 0, 1},
                                                      {0, 0, -1}, {1, 0, 0},
                                                      {-1, 0, 0}, {0, 1, 0},
                                                      {0, -1, 0} };
                    for (const auto &d2 : innerDirs) {
                        Coord3 n2 = { neighbor[0] + d2[0],
                                      neighbor[1] + d2[1],
                                      neighbor[2] + d2[2] };
                        if (ignore.find(n2) != ignore.end())
                            continue;
                        if (n2 == start || n2 == goal) {
                            flag = true;
                            break;
                        }
                        CellInfo k2 = getKind(n2);
                        if (k2.type == CellType::Block) {
                            flag = false;
                        }
                        else if (k2.type == CellType::Integrated) {
                            auto &info = k2.integrated.value();
                            if (info.pid.first == start || info.pid.first == goal)
                                continue;
                            if (info.pid.second == start || info.pid.second == goal)
                                continue;
                            flag = false;
                        }
                    }
                    if (!flag) {
                        clearanceLut[pos] = false;
                        return false;
                    }
                }
            }
            else if (kind.type == CellType::Integrated) {
                auto &info = kind.integrated.value();
                // If state is NoneState and the current neighbor move is downward.
                if (info.state == IntegratedPathInfo::State::NoneState && d[2] == -1)
                    continue;
                if (info.pid.first == start || info.pid.first == goal)
                    continue;
                if (info.pid.second == start || info.pid.second == goal) {
                    clearanceLut[pos] = false;
                    return false;
                }
            }
        }
        clearanceLut[pos] = true;
        return true;
    }
};

// ---------------------------------------------------------------------
// Example usage (can be excluded via a preprocessor definition)
// ---------------------------------------------------------------------

#ifndef UNIT_TEST
int main() {
    // The getKind function.
    // For demonstration:
    //   - (2,2,0) returns an integrated path from (2,2,0)->(2,3,0)->(2,4,0)
    //   - (3,3,0) is blocked.
    // Otherwise, the cell is treated as Air.
    auto getKind = [](int x, int y, int z) -> CellInfo {
        if (x == 2 && y == 2 && z == 0) {
            IntegratedPathInfo ip;
            ip.pid = { {2, 2, 0}, {2, 4, 0} }; // endpoints: (2,2,0) and (2,4,0)
            ip.state = IntegratedPathInfo::State::TrueState;
            ip.nneb = {2, 3, 0};
            ip.path = { {2, 2, 0}, {2, 3, 0}, {2, 4, 0} };
            CellInfo ci;
            ci.type = CellType::Integrated;
            ci.integrated = ip;
            return ci;
        }
        if (x == 3 && y == 3 && z == 0) {
            CellInfo ci;
            ci.type = CellType::Block;
            return ci;
        }
        CellInfo ci;
        ci.type = CellType::Air;
        return ci;
    };

    // Define the bounding volume (from (0,0,0) to (10,10,10)).
    std::pair<Coord3, Coord3> bounds = { {0, 0, 0}, {10, 10, 10} };
    AStarSolver solver(bounds, getKind);

    Coord3 start = {0, 0, 0};
    Coord3 goal  = {5, 5, 0};

    try {
        auto path = solver.solve(start, goal);
        if (path.empty()) {
            std::cout << "Exceeded iteration limit without finding a solution." << std::endl;
        } else {
            std::cout << "Path found:" << std::endl;
            for (const auto &p : path) {
                std::cout << "(" << p[0] << ", " << p[1] << ", " << p[2] << ")" << std::endl;
            }
        }
    } catch (const OutOfBoundsError &e) {
        std::cout << "OutOfBoundsError: " << e.what() << std::endl;
    } catch (const NoPathFoundError &e) {
        std::cout << "NoPathFoundError: " << e.what() << std::endl;
    }
    return 0;
}
#endif
