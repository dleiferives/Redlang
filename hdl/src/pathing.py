import heapq
from typing import Tuple, List, Dict, Optional, Callable, Any


class OutOfBoundsError(Exception):
    pass


class NoPathFoundError(Exception):
    pass


class AStarSolver:
    def __init__(
        self,
        bounds: Tuple[Tuple[int, int, int], Tuple[int, int, int]],
        get_kind: Callable[[int, int, int], Any],
    ):
        """
        Initialize the solver with a bounding volume and a function to
        check the type of a cell.

        Parameters:
          bounds: A tuple of ((min_x, min_y, min_z), (max_x, max_y, max_z)).
          get_kind: A callable that, given (x, y, z), returns:
                    - None if the cell is empty (air).
                    - "full" if the cell is blocked.
                    - A list of positions representing an alternate integrated
                      path. In that case, the list should begin (or end) with the
                      current cell if this path is to be used as an entry point.
        """
        self.bounds = bounds
        self.get_kind_func = get_kind
        self.lut = {}
        self.invalid_path_lut = {}
        self.invalid_lut = {}
        self.clearance_lut = {}
        self.kind_lut = {}

        (self.min_x, self.min_y, self.min_z), (
            self.max_x,
            self.max_y,
            self.max_z,
        ) = bounds

        # Allowed moves:
        # Horizontal moves (same z-level)
        self.horizontal_moves = [
            (1, 0, 0),
            (-1, 0, 0),
            (0, 1, 0),
            (0, -1, 0),
        ]
        # Staircase moves upward (requires a horizontal component)
        self.staircase_moves_up = [
            (1, 0, 1),
            (-1, 0, 1),
            (0, 1, 1),
            (0, -1, 1),
        ]
        # Staircase moves downward
        self.staircase_moves_down = [
            (1, 0, -1),
            (-1, 0, -1),
            (0, 1, -1),
            (0, -1, -1),
        ]
        self.all_moves = (
            self.horizontal_moves + self.staircase_moves_up + self.staircase_moves_down
        )

    def get_kind(self, pos):
        if pos in self.kind_lut:
            return self.kind_lut[pos]
        kind = self.get_kind_func(*pos)
        self.kind_lut[pos] = kind
        return kind


    def in_bounds(self, pos: Tuple[int, int, int]) -> bool:
        """Check if the given position is within bounds."""
        x, y, z = pos
        return (
            self.min_x <= x <= self.max_x
            and self.min_y <= y <= self.max_y
            and self.min_z <= z <= self.max_z
        )

    def heuristic(self, a: Tuple[int, int, int], b: Tuple[int, int, int]) -> int:
        """Manhattan distance heuristic."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])

    def _has_clearance(
        self, pos: Tuple[int, int, int], start: Tuple[int, int, int],
            goal: Tuple[int, int, int], ignore=[]
    ) -> bool:
        """
        Check if the given cell at pos satisfies the clearance requirement.

        Clearance now means that every neighboring cell (including
        diagonals in all three dimensions, i.e. all 26 adjacent cells)
        must be air (get_kind returns None), unless that neighbor is the
        start or goal. This ensures the wiring cell is completely isolated,
        except in the direction where it connects to an endpoint.
        """
        if pos in self.clearance_lut:
            return self.clearance_lut[pos]
        x, y, z = pos


        for d in [[0, 0, 1],
                    [0, 0, -1],
                    [1, 0, 0],
                    [-1, 0, 0],
                    [0, 1, 0],
                    [0, -1, 0]
                    ]:
            dx, dy, dz = d
            neighbor = (x + dx, y + dy, z + dz)
            if(neighbor in ignore):
                continue
            # If the neighbor is an endpoint, allow any content.
            if neighbor == start or neighbor == goal:
                ignore.append(neighbor)
        for d in [[0, 0, 1],
                    [0, 0, -1],
                    [1, 0, 0],
                    [-1, 0, 0],
                    [0, 1, 0],
                    [0, -1, 0]
                    ]:
            dx, dy, dz = d
            neighbor = (x + dx, y + dy, z + dz)
            kind = self.get_kind(neighbor)
            if(neighbor in ignore):
                continue
            if dx == 0 and dy == 0 and dz == 0:
                continue  # Skip the cell itself.
            if not self.in_bounds(neighbor):
                # if(start == (9, 20, 34)):
                #     print(f"failed {neighbor} bounds")
                self.clearance_lut[pos] = False
                return False
            # The neighbor must be air.
            if kind == False:
                self.clearance_lut[pos] = False
                return False
            if kind is None:
                flag = True
                if len(ignore) == 0:
                    for d2 in [[0,0,0],
                            [0, 0, 1],
                                [0, 0, -1],
                                [1, 0, 0],
                                [-1, 0, 0],
                                [0, 1, 0],
                                [0, -1, 0]
                                ]:
                        dx2, dy2, dz2 = d2
                        n2 = (neighbor[0] + dx2, neighbor[1] + dy2, neighbor[2] + dz2)
                        if n2 in ignore:
                            continue
                        k2 = self.get_kind(n2)
                        if n2 == start or n2 == goal:
                            flag = True
                            break
                        if k2 == False:
                            flag = False
                        if k2 is None:
                            continue
                        else:
                            p2, s2, *_ = k2
                            if p2[0] == start or p2[0] == goal:
                                continue
                            if p2[1] == goal or p2[1] == start:
                                continue
                            flag = False

                    if flag == True:
                        continue
                    else:
                        self.clearance_lut[pos] = False
                        return False
                else:
                    continue
            if not (isinstance(kind, tuple)):
                self.clearance_lut[pos] = False
                return False
            else:
                pid, state, *_ = kind
                if state is None and dz == -1:
                    continue
                if pid[0] == start or pid[0] == goal:
                    continue
                if pid[1] == goal or pid[1] == start:
                    continue
                # if(start == (9, 20, 34)):
                #     print(f"failed {neighbor} path")
                self.clearance_lut[pos] = False
                return False
        self.clearance_lut[pos] = True
        return True

    def reset_path_caches(self):
        self.clearance_lut = {}
        self.kind_lut = {}
        self.invalid_path_lut = {}

    def reset_caches(self):
        self.invalid_lut = {}
        self.invalid_path_lut = {}
        self.clearance_lut = {}
        self.kind_lut = {}

    def solve(
        self, start: Tuple[int, int, int], goal: Tuple[int, int, int],max_iterations
    ) -> List[Tuple[int, int, int]]:
        """
        Perform A* search from start to goal.

        Requirements:
          - For any cell other than start or goal, get_kind must return None (air)
            and the cell must satisfy clearance conditions.
          - If get_kind returns a path for a cell (and that path begins or ends with
            start or goal), then that alternate (integrated) path is allowed.
          - The final solution is the complete route from the original start to the
            goal, including any jumps onto alternate integrated paths.

        Raises:
          OutOfBoundsError: if start or goal are out of bounds.
          NoPathFoundError: if no valid path exists.
        """
        self.reset_caches()
        if max_iterations == -1:
            iterations = 100000000000000
        else:
            iterations = max_iterations
        # print(iterations)
        if not self.in_bounds(start):
            raise OutOfBoundsError("Start position is out of bounds")
        if not self.in_bounds(goal):
            raise OutOfBoundsError("Goal position is out of bounds")

        # A* search initialization.
        open_set = []
        heapq.heappush(open_set, (self.heuristic(start, goal), 0, start))
        came_from: Dict[Tuple[int, int, int],
                        Optional[Tuple[int, int, int]]] = {start: None}
        cost_so_far: Dict[Tuple[int, int, int], int] = {start: 0}

        it_counter = 0
        while open_set:
            if(it_counter > iterations):
                return False
            it_counter += 1
            _, current_cost, current = heapq.heappop(open_set)

            if current == goal:
                # Reconstruct path by walking back from goal.
                path = []
                while current is not None:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            for dx, dy, dz in self.all_moves:
                if(it_counter > iterations):
                    return False
                it_counter += 1
                extra_paths =[]
                neighbor = (
                    current[0] + dx,
                    current[1] + dy,
                    current[2] + dz,
                )

                if neighbor in self.invalid_lut:
                    continue
                if neighbor in self.invalid_path_lut:
                    continue

                if not self.in_bounds(neighbor):
                    self.invalid_lut[neighbor] = True
                    continue

                # For endpoints, clearances are not enforced.
                if neighbor != start and neighbor != goal:
                    kind = self.get_kind(neighbor)
                    if kind == False:
                        continue

                    if kind is None:
                        # For wiring cells, the cell must be air and have full clearance.
                        if not self._has_clearance(neighbor, start, goal):
                            self.invalid_lut[neighbor] = True
                            continue
                        extra_paths.append((neighbor, 0))
                    else:
                        # If get_kind returns an integrated path, then only allow it if it
                        # begins or ends with the start or goal.
                        if not (isinstance(kind, tuple) and kind):
                            continue
                        npid, nstate, nneb, npath = kind
                        nstart, ngoal = npid
                        if ngoal == goal:
                            ncst = 0
                            if nstate == True:
                            # we have found our path to the goal
                                if nneb not in npath:
                                    self.invalid_path_lut[nneb] = True
                                    continue
                            elif nstate == False or nstate == 'top':
                                self.invalid_path_lut[nneb] = True
                                continue

                            else:
                                # we are one away and just have to produce
                                tdx, tdy, tdz = nneb
                                if nneb not in npath:
                                    self.invalid_path_lut[nneb] = True
                                    continue
                                else:
                                    ncst = 1;
                                    extra_paths.append((neighbor,0))

                            nindex = npath.index(nneb)
                            nnpath = npath[nindex:]
                            for nix, nstep in enumerate(nnpath):
                                extra_paths.append((nstep,nix + ncst))

                        elif nstart == start:
                            if nstate == True or nstate is None:
                                extra_paths.append((neighbor,0))

                    for alt_pos, extra_cost in extra_paths:
                        new_cost = cost_so_far[current] + 1 + extra_cost
                        if alt_pos not in cost_so_far or new_cost < cost_so_far[alt_pos]:
                            cost_so_far[alt_pos] = new_cost
                            priority = new_cost + self.heuristic(alt_pos, goal)
                            heapq.heappush(open_set, (priority, new_cost, alt_pos))
                            came_from[alt_pos] = current
                else:
                    new_cost = cost_so_far[current] + 1
                    if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (priority, new_cost, neighbor))
                        came_from[neighbor] = current

        raise NoPathFoundError(
            f"No valid path found from start to goal in current state. {start}->{goal}"
        )


# Example usage:
if __name__ == "__main__":
    # Example get_kind implementation.
    # This dummy function returns:
    #   - None if the cell is empty (air).
    #   - "full" if the cell is blocked.
    #   - An integrated path (list of positions) if one is present at a cell.
    #
    # For demonstration, suppose cell (2, 2, 0) has an integrated redstone wire path:
    # It returns a path from (2, 2, 0) to (2, 3, 0) to (2, 4, 0).
    def get_kind(x: int, y: int, z: int):
        if (x, y, z) == (2, 2, 0):
            return [(2, 2, 0), (2, 3, 0), (2, 4, 0)]
        # Block another cell for demonstration.
        if (x, y, z) == (3, 3, 0):
            return "full"
        return None

    # Define a bounding volume: ((min_x, min_y, min_z), (max_x, max_y, max_z)).
    bounds = ((0, 0, 0), (10, 10, 10))
    solver = AStarSolver(bounds, get_kind)

    start_position = (0, 0, 0)
    goal_position = (5, 5, 0)

    try:
        path = solver.solve(start_position, goal_position)
        print("Path found:")
        for pos in path:
            print(pos)
    except (OutOfBoundsError, NoPathFoundError) as err:
        print(f"Error: {err}")
