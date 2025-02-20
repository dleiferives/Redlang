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
        self.get_kind = get_kind

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
        goal: Tuple[int, int, int]
    ) -> bool:
        """
        Check if the given cell at pos satisfies the clearance requirement.

        Clearance now means that every neighboring cell (including
        diagonals in all three dimensions, i.e. all 26 adjacent cells)
        must be air (get_kind returns None), unless that neighbor is the
        start or goal. This ensures the wiring cell is completely isolated,
        except in the direction where it connects to an endpoint.
        """
        x, y, z = pos

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue  # Skip the cell itself.
                    neighbor = (x + dx, y + dy, z + dz)
                    # If the neighbor is an endpoint, allow any content.
                    if neighbor == start or neighbor == goal:
                        return True
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue  # Skip the cell itself.
                    if not self.in_bounds(neighbor):
                        return False
                    # The neighbor must be air.
                    if self.get_kind(*neighbor) is not None:
                        return False
        return True

    def solve(
        self, start: Tuple[int, int, int], goal: Tuple[int, int, int]
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

        while open_set:
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
                neighbor = (
                    current[0] + dx,
                    current[1] + dy,
                    current[2] + dz,
                )

                if not self.in_bounds(neighbor):
                    continue

                # For endpoints, clearances are not enforced.
                if neighbor != start and neighbor != goal:
                    kind = self.get_kind(*neighbor)
                    if kind == "full":
                        continue

                    if kind is None:
                        # For wiring cells, the cell must be air and have full clearance.
                        if not self._has_clearance(neighbor, start, goal):
                            continue
                        extra_paths = [(neighbor, 0)]
                    else:
                        # If get_kind returns an integrated path, then only allow it if it
                        # begins or ends with the start or goal.
                        if not (isinstance(kind, list) and kind):
                            continue
                        if (
                            kind[0] == start or kind[-1] == goal
                        ):
                            # Allow a jump along the integrated path, with the jump cost based
                            # on the index along the integrated path.
                            extra_paths = [(neighbor, 0)]
                            # for index, alt_pos in enumerate(kind):
                            #     # If an integrated cell is not an endpoint, assume its clearance
                            #     # is guaranteed by its integrated configuration.
                            #     if alt_pos != start and alt_pos != goal:
                            #         if not self.in_bounds(alt_pos):
                            #             continue
                            #     extra_paths.append((alt_pos, index))
                        else:
                            continue

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
            "No valid path found from start to goal using the provided constraints."
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
