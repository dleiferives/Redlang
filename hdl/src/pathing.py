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
                    - None if the cell is empty.
                    - 'full' if the cell is blocked.
                    - A list of positions (path) if there is an integrated path present.
                      In that case, the first element of that list must be (x,y,z).
        """
        self.bounds = bounds
        self.get_kind = get_kind

        (self.min_x, self.min_y, self.min_z), (
            self.max_x,
            self.max_y,
            self.max_z,
        ) = bounds

        # Allowed moves:
        # Horizontal moves (same z level)
        self.horizontal_moves = [
            (1, 0, 0),
            (-1, 0, 0),
            (0, 1, 0),
            (0, -1, 0),
        ]
        # Staircase moves upward (requires a horizontal component)
        self.staircase_moves = [
            (1, 0, 1),
            (-1, 0, 1),
            (0, 1, 1),
            (0, -1, 1),
        ]
        self.all_moves = self.horizontal_moves + self.staircase_moves

    def in_bounds(self, pos: Tuple[int, int, int]) -> bool:
        """Check if pos is within bounds."""
        x, y, z = pos
        return (
            self.min_x <= x <= self.max_x
            and self.min_y <= y <= self.max_y
            and self.min_z <= z <= self.max_z
        )

    def heuristic(
        self, a: Tuple[int, int, int], b: Tuple[int, int, int]
    ) -> int:
        """Manhattan distance heuristic."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])

    def _has_clearance(self, pos: Tuple[int, int, int]) -> bool:
        """
        Check clearance around pos:
          - All 8 neighbors in the same horizontal plane (excluding itself) are empty.
          - The cells directly above (z+1) and two units above (z+2) are empty.
        """
        x, y, z = pos

        # Horizontal clearance on the same z.
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                neighbor = (x + dx, y + dy, z)
                if self.in_bounds(neighbor):
                    # For clearance, we require the neighbor be empty.
                    # Note: endpoints are exempt, but this function only applies to
                    # intermediate cells.
                    if self.get_kind(*neighbor) is not None:
                        # Not empty (could be 'full' or a path) => clearance fails.
                        return False
                else:
                    return False

        # Vertical clearance: a gap of 2 cells above.
        for dz in [1, 2]:
            above = (x, y, z + dz)
            if self.in_bounds(above):
                if self.get_kind(*above) is not None:
                    return False
            else:
                return False

        return True

    def solve(
        self, start: Tuple[int, int, int], goal: Tuple[int, int, int]
    ) -> List[Tuple[int, int, int]]:
        """
        Perform A* search from start to goal.

        Requirements:
          - For any cell other than the start or goal, get_kind must return None
            (empty) and the cell **must** have clearance conditions met.
          - If get_kind returns a path for a cell (and that path starts at the cell),
            then the algorithm is allowed to use any cell along that path as a valid
            starting point in place of that cell.
          - The solution returned will be the full path from the original start to
            the original goal, including any jumps onto the alternative paths.

        Raises:
          OutOfBoundsError: if start or goal are out-of-bounds.
          NoPathFoundError: if no valid path exists.
        """
        if not self.in_bounds(start):
            raise OutOfBoundsError("Start position is out of bounds")
        if not self.in_bounds(goal):
            raise OutOfBoundsError("Goal position is out of bounds")

        # For the start and goal, we don't check emptiness or clearance.
        open_set = []
        heapq.heappush(
            open_set, (self.heuristic(start, goal), 0, start)
        )
        came_from: Dict[Tuple[int, int, int],
                        Optional[Tuple[int, int, int]]] = {start: None}
        cost_so_far: Dict[Tuple[int, int, int], int] = {start: 0}

        while open_set:
            _, current_cost, current = heapq.heappop(open_set)

            if current == goal:
                # Reconstruct the full path.
                path = []
                while current is not None:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            for dx, dy, dz in self.all_moves:
                neighbor = (current[0] + dx,
                            current[1] + dy,
                            current[2] + dz)

                if not self.in_bounds(neighbor):
                    continue

                # For endpoints (start and goal), we allow non-empty.
                if neighbor != start and neighbor != goal:
                    kind = self.get_kind(*neighbor)
                    if kind == "full":
                        continue
                    # If the cell is empty (None), then use it.
                    if kind is None:
                        # Also check clearance.
                        if not self._has_clearance(neighbor):
                            continue
                        # This neighbor is valid.
                        extra_paths = [(neighbor, 0)]
                    else:
                        # kind is presumed to be a path (list of positions).
                        # Verify that the alternative path starts at neighbor.
                        if not (isinstance(kind, list) and kind and kind[0] == neighbor):
                            # If not, treat it as blocked.
                            continue
                        # For each position along the alternate route,
                        # we allow a "jump". The cost to reach one of these positions
                        # is the Manhattan distance along the alternative path (assumed to be
                        # the index in the list) plus 1 for moving to neighbor.
                        extra_paths = []
                        for index, alt_pos in enumerate(kind):
                            # Only allow a jump if the alternate position has clearance.
                            if alt_pos != start and alt_pos != goal:
                                if not self.in_bounds(alt_pos):
                                    continue
                                if not self._has_clearance(alt_pos):
                                    continue
                            extra_paths.append((alt_pos, index))
                    # Process each possible alternative endpoint.
                    for alt_pos, extra_cost in extra_paths:
                        total_cost = cost_so_far[current] + 1 + extra_cost
                        if alt_pos not in cost_so_far or total_cost < cost_so_far[alt_pos]:
                            cost_so_far[alt_pos] = total_cost
                            priority = total_cost + self.heuristic(alt_pos, goal)
                            heapq.heappush(open_set, (priority, total_cost, alt_pos))
                            # Record jump: our neighbor came from current. If we jumped,
                            # we record that the jump was via the alternative path.
                            came_from[alt_pos] = current
                else:
                    # For endpoints, no get_kind or clearance needed.
                    total_cost = cost_so_far[current] + 1
                    if neighbor not in cost_so_far or total_cost < cost_so_far[neighbor]:
                        cost_so_far[neighbor] = total_cost
                        priority = total_cost + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (priority, total_cost, neighbor))
                        came_from[neighbor] = current

        raise NoPathFoundError(
            "No valid path found from start to goal using the provided constraints."
        )


# Example usage:
if __name__ == "__main__":
    # Example get_kind implementation.
    # This dummy function returns:
    #  - None if the cell is "empty".
    #  - "full" if the cell is blocked.
    #  - A pre-defined alternate path if one is present at a cell.
    #
    # For demonstration purposes, suppose cell (2,2,0) has an integrated path:
    # It returns a path from (2,2,0) to (2,3,0) to (2,4,0).
    def get_kind(x: int, y: int, z: int):
        if (x, y, z) == (2, 2, 0):
            return [(2, 2, 0), (2, 3, 0), (2, 4, 0)]
        # For demonstration, you could block a cell:
        if (x, y, z) == (3, 3, 0):
            return "full"
        return None

    # Define bounding volume: ((min_x, min_y, min_z), (max_x, max_y, max_z))
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
