import random
import math
import argparse

class Cell:
    def __init__(self, name):
        self.name = name
        self.width = 5
        self.height = 5
        self.x = 0  # Initial placement
        self.y = 0
        self.connections = []  # List of cell names this cell connects to
        self.occupied_locations = [] # List of (x, y) tuples occupied by the cell

    def __repr__(self):
        return f"Cell({self.name}, x={self.x}, y={self.y})"

    def distance(self, other_cell):
        """Calculates Manhattan distance between two cells."""
        return abs(self.x - other_cell.x) + abs(self.y - other_cell.y)

    def is_valid_placement(self, grid_width, grid_height):
        """Checks if the cell's placement is within the grid bounds."""
        return (0 <= self.x < grid_width - self.width + 1 and
                0 <= self.y < grid_height - self.height + 1)

    def overlaps(self, other_cell):
        """Checks if this cell overlaps with another cell."""
        return (self.x < other_cell.x + other_cell.width and
                self.x + self.width > other_cell.x and
                self.y < other_cell.y + other_cell.height and
                self.y + self.height > other_cell.y)

    def allocate_locations(self):
        """Allocates the locations occupied by the cell."""
        self.occupied_locations = []
        for x in range(self.x, self.x + self.width):
            for y in range(self.y, self.y + self.height):
                self.occupied_locations.append((x, y))

class PlacementAlgorithm:
    def __init__(self, cells, grid_width, grid_height, max_wire_length=10):
        """
        Initializes the placement algorithm.

        Args:
            cells (list of Cell): The list of cells to place.
            grid_width (int): The width of the placement grid.
            grid_height (int): The height of the placement grid.
            max_wire_length (int): The maximum allowed wire length.
        """
        self.cells = cells
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.max_wire_length = max_wire_length
        self.placed_cells = []  # Cells that have been placed
        self.unplaced_cells = cells[:]  # Cells that still need to be placed
        self.occupied_locations = set() # Set of (x, y) tuples occupied by cells or wires

    def place_cells(self, algorithm="random"):
        """
        Places the cells using the specified algorithm.

        Args:
            algorithm (str): The placement algorithm to use ("random", "greedy").
        """
        if algorithm == "random":
            self._random_placement()
        elif algorithm == "greedy":
            self._greedy_placement()
        else:
            print("Error: Invalid placement algorithm.")

    def _random_placement(self):
        """
        Places cells randomly, ensuring valid placements, wire length constraints,
        and no overlaps, and no wire locations.
        """
        random.shuffle(self.unplaced_cells)  # Randomize the order of placement

        for cell in self.unplaced_cells:
            placed = False
            for _ in range(100):  # Try a few random placements
                cell.x = random.randint(0, self.grid_width - cell.width)
                cell.y = random.randint(0, self.grid_height - cell.height)
                if cell.is_valid_placement(self.grid_width, self.grid_height):
                    # Check for overlaps with other cells and occupied locations
                    valid_placement = True
                    for x in range(cell.x, cell.x + cell.width):
                        for y in range(cell.y, cell.y + cell.height):
                            if (x, y) in self.occupied_locations:
                                valid_placement = False
                                break
                        if not valid_placement:
                            break

                    if valid_placement:
                        # Check wire length constraints
                        for connected_cell_name in cell.connections:
                            connected_cell = self._find_cell_by_name(connected_cell_name)
                            if connected_cell and connected_cell in self.placed_cells:
                                if cell.distance(connected_cell) > self.max_wire_length:
                                    valid_placement = False
                                    break

                    if valid_placement:
                        # Allocate locations
                        cell.allocate_locations()
                        for loc in cell.occupied_locations:
                            self.occupied_locations.add(loc)
                        self.placed_cells.append(cell)
                        self.unplaced_cells.remove(cell)
                        placed = True
                        break  # Placement successful
            if not placed:
                print(f"Warning: Could not place cell {cell.name} within wire length constraints and without overlaps.")

    def _greedy_placement(self):
        """
        Places cells greedily, prioritizing cells with more connections,
        attempting to minimize wire length, and avoiding overlaps and wire locations.
        """
        # Sort cells by the number of connections (descending)
        self.unplaced_cells.sort(key=lambda cell: len(cell.connections), reverse=True)

        for cell in self.unplaced_cells:
            best_x, best_y = -1, -1
            min_wire_length = float('inf')

            for x in range(0, self.grid_width - cell.width + 1):
                for y in range(0, self.grid_height - cell.height + 1):
                    cell.x, cell.y = x, y
                    if cell.is_valid_placement(self.grid_width, self.grid_height):
                        # Check for overlaps with other cells and occupied locations
                        valid_placement = True
                        for check_x in range(cell.x, cell.x + cell.width):
                            for check_y in range(cell.y, cell.y + cell.height):
                                if (check_x, check_y) in self.occupied_locations:
                                    valid_placement = False
                                    break
                            if not valid_placement:
                                break

                        if valid_placement:
                            total_wire_length = 0
                            for connected_cell_name in cell.connections:
                                connected_cell = self._find_cell_by_name(connected_cell_name)
                                if connected_cell and connected_cell in self.placed_cells:
                                    wire_length = cell.distance(connected_cell)
                                    if wire_length > self.max_wire_length:
                                        valid_placement = False
                                        break
                                    total_wire_length += wire_length

                            if valid_placement:
                                if total_wire_length < min_wire_length:
                                    min_wire_length = total_wire_length
                                    best_x, best_y = x, y

            if best_x != -1:
                cell.x, cell.y = best_x, best_y
                # Allocate locations
                cell.allocate_locations()
                for loc in cell.occupied_locations:
                    self.occupied_locations.add(loc)
                self.placed_cells.append(cell)
                self.unplaced_cells.remove(cell)
            else:
                print(f"Warning: Could not place cell {cell.name} within wire length constraints and without overlaps.")

    def _find_cell_by_name(self, cell_name):
        """Helper function to find a cell by its name."""
        for cell in self.cells:
            if cell.name == cell_name:
                return cell
        return None

    def print_placement(self):
        """Prints the placement of the cells."""
        for cell in self.cells:
            print(f"Cell {cell.name}: x={cell.x}, y={cell.y}")

    def calculate_wire_length(self):
        """Calculates the total wire length of the placement."""
        total_wire_length = 0
        for cell in self.cells:
            for connected_cell_name in cell.connections:
                connected_cell = self._find_cell_by_name(connected_cell_name)
                if connected_cell:
                    total_wire_length += cell.distance(connected_cell)
        return total_wire_length

    def display(self):
        """Displays the cell placement and wire routing using ASCII characters."""
        grid = [['.' for _ in range(self.grid_width)] for _ in range(self.grid_height)]

        # Place cells
        for cell in self.placed_cells:
            for x in range(cell.x, cell.x + cell.width):
                for y in range(cell.y, cell.y + cell.height):
                    grid[y][x] = cell.name[0]  # Use the first letter of the cell name

        # Draw wires (Manhattan routing)
        for cell in self.placed_cells:
            for connected_cell_name in cell.connections:
                connected_cell = self._find_cell_by_name(connected_cell_name)
                if connected_cell and connected_cell in self.placed_cells:
                    # Manhattan routing
                    x1, y1 = cell.x + cell.width // 2, cell.y + cell.height // 2
                    x2, y2 = connected_cell.x + connected_cell.width // 2, connected_cell.y + connected_cell.height // 2

                    # Route the wire
                    wire_segments = self._manhattan_route(x1, y1, x2, y2)
                    for segment in wire_segments:
                        if len(segment) == 2: # Horizontal or Vertical segment
                            for x in range(min(segment[0][0], segment[1][0]), max(segment[0][0], segment[1][0]) + 1):
                                for y in range(min(segment[0][1], segment[1][1]), max(segment[0][1], segment[1][1]) + 1):
                                    if 0 <= y < self.grid_height and 0 <= x < self.grid_width:
                                        grid[y][x] = '|' if segment[0][0] == segment[1][0] else '-'
                        elif len(segment) == 1: # Single point (shouldn't happen with manhattan)
                            if 0 <= segment[0][1] < self.grid_height and 0 <= segment[0][0] < self.grid_width:
                                grid[segment[0][1]][segment[0][0]] = 'X' # Mark the point

        # Print the grid
        print("-" * (self.grid_width + 2))
        for row in reversed(grid):  # Reverse to match typical coordinate systems
            print("|" + "".join(row) + "|")
        print("-" * (self.grid_width + 2))

    def _manhattan_route(self, x1, y1, x2, y2):
        """
        Generates a Manhattan route (horizontal and vertical segments)
        between two points.  Also adds the wire locations to occupied_locations.

        Args:
            x1, y1: Starting coordinates.
            x2, y2: Ending coordinates.

        Returns:
            list of tuples: A list of line segments, where each segment is a tuple of (x, y) coordinates.
        """
        segments = []
        # Add wire locations to occupied_locations
        if x1 == x2:  # Vertical segment
            for y in range(min(y1, y2), max(y1, y2) + 1):
                if (x1, y) not in self.occupied_locations:
                    self.occupied_locations.add((x1, y))
            segments.append([(x1, y1), (x2, y2)])
        elif y1 == y2:  # Horizontal segment
            for x in range(min(x1, x2), max(x1, x2) + 1):
                if (x, y1) not in self.occupied_locations:
                    self.occupied_locations.add((x, y1))
            segments.append([(x1, y1), (x2, y2)])
        else:  # Manhattan route (two segments)
            # Route 1: Horizontal then Vertical
            for x in range(min(x1, x2), max(x1, x2) + 1):
                if (x, y1) not in self.occupied_locations:
                    self.occupied_locations.add((x, y1))
            segments.append([(x1, y1), (x2, y1)]) # Horizontal segment
            for y in range(min(y1, y2), max(y1, y2) + 1):
                if (x2, y) not in self.occupied_locations:
                    self.occupied_locations.add((x2, y))
            segments.append([(x2, y1), (x2, y2)]) # Vertical segment
        return segments
