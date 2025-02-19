import random
import math
import heapq
from typing import List, Tuple, Optional
import json
from yosys_parser import YosysJSONParser  # Uncomment if available

import nbtlib
from nbtlib import Compound, File
from nbtlib.tag import Int, IntArray, List
from typing import List, Tuple, Optional

from typing import List, Tuple
import nbtlib
from nbtlib import Compound, File

def export_state_to_litematica_nbt(
    cells: List, routes: List[List[Tuple[int, int, int]]], filename: str
):
    """
    Exports the current 3D state (cells and wire routes) as an NBT schematic
    using the following structure:

      version: 1
      size: [width, height, length]  (in schematic coordinates)
      pallet: list of compounds, each with a "Name" (e.g. {"Name": "minecraft:air"})
      blocks: list of compounds; each compound defines:
          - pos: [x, y, z] position, in schematic coordinates
          - state: an integer index referencing the block in the pallet

    Voxel assignment (in simulation coordinates):
      - Voxels with a routed wire are "minecraft:red_wool"
      - Voxels occupied by a cell (if not overwritten by a wire) are
        "minecraft:white_wool"
      - All other voxels are "minecraft:air"

    In this simulation we use (x, y, z) with z as up.
    For the schematic we remap:
        schematic X = sim_x - min_x
        schematic Y = sim_z - min_z   (vertical)
        schematic Z = sim_y - min_y
    """
    # Determine overall bounding box in simulation space.
    xs, ys, zs = [], [], []
    for cell in cells:
        xs.extend([cell.x, cell.x + cell.width - 1])
        ys.extend([cell.y, cell.y + cell.depth - 1])
        zs.extend([cell.z, cell.z + cell.height - 1])
    for route in routes:
        for (x, y, z) in route:
            xs.append(x)
            ys.append(y)
            zs.append(z)
    grid_min_x, grid_max_x = min(xs), max(xs)
    grid_min_y, grid_max_y = min(ys), max(ys)
    grid_min_z, grid_max_z = min(zs), max(zs)

    # Schematic dimensions (remapped):
    # schematic X: from simulation x, schematic Y: from simulation z (vertical),
    # schematic Z: from simulation y.
    width  = grid_max_x - grid_min_x + 1  # schematic X dimension
    length = grid_max_y - grid_min_y + 1   # schematic Z dimension
    height = grid_max_z - grid_min_z + 1   # schematic Y dimension

    # Allocate a 3D grid (using [X][Y][Z] indexing)
    # Using the mapping:
    #   sx = sim_x - grid_min_x
    #   sy = sim_z - grid_min_z  (vertical in schematic: Y)
    #   sz = sim_y - grid_min_y
    grid = [
        [
            ["minecraft:air" for _ in range(length)]
            for _ in range(height)
        ]
        for _ in range(width)
    ]

    # Mark cell volumes as white wool.
    for cell in cells:
        for dx in range(cell.width):
            for dy in range(cell.depth):
                for dz in range(cell.height):
                    sim_x = cell.x + dx
                    sim_y = cell.y + dy
                    sim_z = cell.z + dz
                    sx = sim_x - grid_min_x
                    sy = sim_z - grid_min_z  # simulation z -> schematic Y
                    sz = sim_y - grid_min_y  # simulation y -> schematic Z
                    grid[sx][sy][sz] = "minecraft:white_wool"

    # Mark wire route voxels as red wool (overriding any cell fill).
    for route in routes:
        for (x, y, z) in route:
            sx = x - grid_min_x
            sy = z - grid_min_z  # simulation z becomes schematic Y
            sz = y - grid_min_y  # simulation y becomes schematic Z
            grid[sx][sy][sz] = "minecraft:red_wool"

    # Define a static palette mapping.
    # The palette maps block name -> index.
    palette_mapping = {
        "minecraft:air": 0,
        "minecraft:white_wool": 1,
        "minecraft:red_wool": 2
    }

    # Build block compounds.
    blocks = []
    # Iterate over every voxel in the 3D schematic grid. The index order here
    # will be used explicitly as the block's pos.
    for sx in range(width):
        for sy in range(height):
            for sz in range(length):
                block_type = grid[sx][sy][sz]
                state_index = palette_mapping[block_type]
                block_compound = Compound({
                    "pos": nbtlib.tag.List([
                        nbtlib.tag.Int(sx),
                        nbtlib.tag.Int(sy),
                        nbtlib.tag.Int(sz)
                    ]),
                    "state": nbtlib.tag.Int(state_index)
                })
                blocks.append(block_compound)

    # Build pallet: the palette is now a list of compounds containing a "Name".
    pallet = nbtlib.tag.List([
        Compound({"Name": nbtlib.tag.String("minecraft:air")}),
        Compound({"Name": nbtlib.tag.String("minecraft:white_wool")}),
        Compound({"Name": nbtlib.tag.String("minecraft:red_wool")})
    ])

    # Build the overall schematic compound using the new format.
    schematic = Compound({
        "version": nbtlib.tag.Int(1),
        "size": nbtlib.tag.List([
            nbtlib.tag.Int(width),
            nbtlib.tag.Int(height),
            nbtlib.tag.Int(length)
        ]),
        "entities": nbtlib.tag.List([]),
        "palette": pallet,
        "blocks": nbtlib.tag.List(blocks)
    })

    # Create and save the NBT file.
    nbt_file = File(schematic)
    nbt_file.save(filename)
    print(f"Exported NBT schematic to {filename}")


# -------------------------------------------------------------------
# Cell and Overlap (3D)
# -------------------------------------------------------------------
class Cell:
    def __init__(self, name: str, width: int, depth: int, height: int,
                 x: int, y: int, z: int):
        """
        A 3D cell with a position (x,y,z) and dimensions:
          width (x-dimension), depth (y-dimension), height (z-dimension)
        """
        self.name = name
        self.width = width
        self.depth = depth
        self.height = height
        self.x = x  # x coordinate of the "origin" (lower x,y,z corner)
        self.y = y
        self.z = z

    def __repr__(self):
        return (f"{self.name}@({self.x},{self.y},{self.z}) "
                f"[{self.width}x{self.depth}x{self.height}]")

    def get_port(self) -> Tuple[int, int, int]:
        """
        Returns the coordinate of the connection port.
        Here we choose the “top-center” port: center in x and y and just
        above the top face of the cell (i.e. z - 1).
        """
        return (self.x + self.width // 2,
                self.y + self.depth // 2,
                self.z - 1)


def compute_overlap(cell1: Cell, cell2: Cell) -> int:
    """
    Computes the overlapping volume between two cells.
    (A higher overlap means more penalization.)
    """
    overlap_x = max(
        0, min(cell1.x + cell1.width, cell2.x + cell2.width) -
           max(cell1.x, cell2.x)
    )
    overlap_y = max(
        0, min(cell1.y + cell1.depth, cell2.y + cell2.depth) -
           max(cell1.y, cell2.y)
    )
    overlap_z = max(
        0, min(cell1.z + cell1.height, cell2.z + cell2.height) -
           max(cell1.z, cell2.z)
    )
    return overlap_x * overlap_y * overlap_z


# -------------------------------------------------------------------
# A* Grid Routing (for wires in 3 dimensions)
# -------------------------------------------------------------------
def manhattan_distance_points_3d(p1: Tuple[int, int, int],
                                 p2: Tuple[int, int, int]) -> int:
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1]) + abs(p1[2] - p2[2])


def compute_grid_bounds_3d(cells: List[Cell], margin: int = 10
                           ) -> Tuple[int, int, int, int, int, int]:
    """
    Computes a 3D bounding box for the grid based on cell placements,
    with an extra margin. Returns (min_x, max_x, min_y, max_y, min_z, max_z)
    """
    min_x = min(cell.x for cell in cells) - margin
    min_y = min(cell.y for cell in cells) - margin
    min_z = min(cell.z for cell in cells) - margin
    max_x = max(cell.x + cell.width - 1 for cell in cells) + margin
    max_y = max(cell.y + cell.depth - 1 for cell in cells) + margin
    max_z = max(cell.z + cell.height - 1 for cell in cells) + margin
    return (min_x, max_x, min_y, max_y, min_z, max_z)


def is_blocked_3d(x: int, y: int, z: int, cells: List[Cell]) -> bool:
    """
    Returns True if the point (x,y,z) is inside any cell’s volume.
    (Wires are not allowed to run through any cell's internal volume.)
    """
    for cell in cells:
        if (cell.x <= x < cell.x + cell.width and
            cell.y <= y < cell.y + cell.depth and
            cell.z <= z < cell.z + cell.height):
            return True
    return False


def get_neighbors_3d(point: Tuple[int, int, int],
                     cells: List[Cell],
                     bounds: Tuple[int, int, int, int, int, int]
                     ) -> List[Tuple[int, int, int]]:
    (x, y, z) = point
    (min_x, max_x, min_y, max_y, min_z, max_z) = bounds
    # 6 axis-aligned directions (no diagonals)
    directions = [(1, 0, 0), (-1, 0, 0),
                  (0, 1, 0), (0, -1, 0),
                  (0, 0, 1), (0, 0, -1)]
    neighbors = []
    for dx, dy, dz in directions:
        nx, ny, nz = x + dx, y + dy, z + dz
        if nx < min_x or nx > max_x or ny < min_y or ny > max_y or \
           nz < min_z or nz > max_z:
            continue
        if is_blocked_3d(nx, ny, nz, cells):
            continue
        neighbors.append((nx, ny, nz))
    return neighbors


def reconstruct_path(came_from: dict,
                     current: Tuple[int, int, int]) -> List[Tuple[int, int, int]]:
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.append(current)
    total_path.reverse()
    return total_path


def astar_3d(start: Tuple[int, int, int],
             goal: Tuple[int, int, int],
             cells: List[Cell],
             bounds: Tuple[int, int, int, int, int, int]
             ) -> Optional[List[Tuple[int, int, int]]]:
    """
    Finds a path from start to goal using A* in 3D.
    The obstacles are the cells (if a grid point is inside a cell’s volume,
    it cannot be used in the path).
    """
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: manhattan_distance_points_3d(start, goal)}

    while open_set:
        current = heapq.heappop(open_set)[1]
        if current == goal:
            return reconstruct_path(came_from, current)
        for neighbor in get_neighbors_3d(current, cells, bounds):
            tentative_g = g_score[current] + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + manhattan_distance_points_3d(
                    neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return None


# -------------------------------------------------------------------
# Wiring Cost: Route all wires using A* in 3D and penalize overlaps.
# -------------------------------------------------------------------
def compute_wiring_routes_3d(cells: List[Cell],
                             wires: List[Tuple[Cell, Cell]]
                             ) -> Tuple[Optional[List[List[Tuple[int, int, int]]]], float]:
    """
    For each "wire" (a connection between two cells) compute an A* route from the
    port of one cell to the port of the other.

    The cost on wires is the sum of route lengths. In addition, if two wires share
    grid points, a penalty is added.
    Returns a list of routes (one per wire) and the total wiring cost.
    If any connection fails to route, a heavy penalty is incurred.
    """
    routes = []
    total_cost = 0.0
    overlap_penalty = 1000  # penalty for each extra usage of a grid cell
    bounds = compute_grid_bounds_3d(cells, margin=20)

    for cell1, cell2 in wires:
        start = cell1.get_port()
        goal = cell2.get_port()
        route = astar_3d(start, goal, cells, bounds)
        if route is None:
            return None, 1e6
        routes.append(route)
        total_cost += len(route)

    cell_usage = {}
    for route in routes:
        for point in route:
            cell_usage[point] = cell_usage.get(point, 0) + 1
    for count in cell_usage.values():
        if count > 1:
            total_cost += overlap_penalty * (count - 1)
    return routes, total_cost


# -------------------------------------------------------------------
# Overall Cost Function: cell overlaps + wiring cost in 3D
# -------------------------------------------------------------------
def compute_cell_overlap_cost(cells: List[Cell]) -> float:
    cost = 0.0
    overlap_penalty = 10000
    for i in range(len(cells)):
        for j in range(i + 1, len(cells)):
            overlap = compute_overlap(cells[i], cells[j])
            if overlap > 0:
                cost += overlap_penalty * overlap
    return cost


def compute_total_cost_3d(cells: List[Cell],
                          wires: List[Tuple[Cell, Cell]]) -> float:
    cell_cost = compute_cell_overlap_cost(cells)
    _, wiring_cost = compute_wiring_routes_3d(cells, wires)
    return cell_cost + wiring_cost


# -------------------------------------------------------------------
# Simulated Annealing: Move cells in 3D and update routing.
# -------------------------------------------------------------------

# --- MODIFIED 3D SIMULATED ANNEALING, EXPORTING EVERY 100 ITERATIONS ---
def simulated_annealing_3d(
    cells: List,
    wires: List[Tuple],
    max_iter: int = 10000,
    start_temp: float = 1000.0,
    cooling_rate: float = 0.995,
    export_interval: int = 100
) -> Tuple[Optional[List[Tuple[int, int, int]]], float]:
    """
    Performs simulated annealing to adjust cell positions in 3D.
    Every 'export_interval' iterations the current state is exported as an NBT
    schematic using `export_state_to_litematica_nbt`.

    Returns a tuple (best_state, best_cost) where best_state is a list of cell
    coordinates.
    """
    current_cost = compute_total_cost_3d(cells, wires)
    best_cost = current_cost
    best_state = [(cell.x, cell.y, cell.z) for cell in cells]

    temperature = start_temp

    for i in range(max_iter):
        print(f"Iteration: {i}")
        if temperature < 1e-3:
            break

        # Choose a random cell and propose a move.
        cell = random.choice(cells)
        old_x, old_y, old_z = cell.x, cell.y, cell.z
        move_x = random.randint(-3, 3)
        move_y = random.randint(-3, 3)
        move_z = random.randint(-3, 3)
        cell.x += move_x
        cell.y += move_y
        cell.z += move_z

        new_cost = compute_total_cost_3d(cells, wires)
        if (new_cost < current_cost or
            math.exp((current_cost - new_cost) / temperature) > random.random()):
            current_cost = new_cost
            if new_cost < best_cost:
                best_cost = new_cost
                best_state = [(c.x, c.y, c.z) for c in cells]
        else:
            # Revert move if not accepted.
            cell.x, cell.y, cell.z = old_x, old_y, old_z

        temperature *= cooling_rate

        # Every 'export_interval' iterations, export the current state.
        if i == 10 or i % export_interval == 0:
            routes, _ = compute_wiring_routes_3d(cells, wires)
            if routes is not None:
                filename = f"schematic_iter_{i}.nbt"
                export_state_to_litematica_nbt(cells, routes, filename)

    # A very low total cost indicates a valid placement.
    threshold = 1e5
    if best_cost < threshold:
        return best_state, best_cost
    else:
        return None, best_cost


# -------------------------------------------------------------------
# Rendering the 3D placement: Show each layer (z slice) separately.
# -------------------------------------------------------------------
def render_placement_3d(cells: List[Cell],
                        routes: List[List[Tuple[int, int, int]]]):
    """
    Renders the placement of cells and wires onto a 3D ASCII grid.
    Each z-level (layer) is printed separately.

    Priority: cell > port marker > wire > empty.
    """
    # Determine overall bounding box from cells and routes.
    xs, ys, zs = [], [], []
    for cell in cells:
        xs.extend([cell.x, cell.x + cell.width - 1])
        ys.extend([cell.y, cell.y + cell.depth - 1])
        zs.extend([cell.z, cell.z + cell.height - 1])
    for route in routes:
        for (x, y, z) in route:
            xs.append(x)
            ys.append(y)
            zs.append(z)
    grid_min_x, grid_max_x = min(xs), max(xs)
    grid_min_y, grid_max_y = min(ys), max(ys)
    grid_min_z, grid_max_z = min(zs), max(zs)

    width = grid_max_x - grid_min_x + 1
    depth = grid_max_y - grid_min_y + 1

    # Create a 3D grid (dict keyed by z-level)
    layers = {}
    for z in range(grid_min_z, grid_max_z + 1):
        layers[z] = [[" " for _ in range(width)] for _ in range(depth)]

    # 1. Render all wire routes (using "*" as the symbol)
    for route in routes:
        for (x, y, z) in route:
            gx = x - grid_min_x
            gy = y - grid_min_y
            if layers[z][gy][gx] == " ":
                layers[z][gy][gx] = "*"

    # 2. Render cells (using first letter of name, or '#' if unavailable)
    for cell in cells:
        symbol = cell.name[0] if cell.name else "#"
        for dz in range(cell.height):
            for dy in range(cell.depth):
                for dx in range(cell.width):
                    cx = cell.x + dx - grid_min_x
                    cy = cell.y + dy - grid_min_y
                    cz = cell.z + dz
                    layers[cz][cy][cx] = symbol

    # 3. Mark the ports of the cells (with 'P') if not overwritten by a cell.
    for cell in cells:
        px, py, pz = cell.get_port()
        gx = px - grid_min_x
        gy = py - grid_min_y
        if layers[pz][gy][gx] in (" ", "*"):
            layers[pz][gy][gx] = "P"

    # 4. Print each layer
    for z in range(grid_min_z, grid_max_z + 1):
        print(f"Layer z = {z}")
        for row in layers[z]:
            print("".join(row))
        print()


# -------------------------------------------------------------------
# Conversion from modules (if using Yosys) into cells and wires (3D)
# -------------------------------------------------------------------
def convert_module_to_cells_and_wires(module: dict) -> Tuple[
        List[Cell], List[Tuple[Cell, Cell]]]:
    """
    Converts a parsed Yosys module into lists of placement cells and wires.
    In a 3D setting, we assign an initial z coordinate (e.g. 0) and a default
    height. You may customize the default sizes.
    """
    # Default sizes for different cell types.
    default_sizes = {
        "NOT": (3, 3, 2),    # width, depth, height
        "NAND": (3, 3, 3),
        "NOR": (3, 3, 3),
        "DFF": (3, 3, 3),
        "PORT": (1, 1, 1),   # minimal for ports.
    }

    cells = []
    net_to_endpoints = {}

    # 1. Convert all internal cells.
    for cell_name, cell_data in module.get("cells", {}).items():
        cell_type = cell_data.get("type", "UNKNOWN")
        width, depth, height = default_sizes.get(cell_type, (3, 3, 3))
        cell_obj = Cell(cell_name, width, depth, height, 0, 0, 0)
        cells.append(cell_obj)
        for port_name, nets in cell_data.get("connections", {}).items():
            for net in nets:
                net_to_endpoints.setdefault(net, []).append((cell_obj, port_name))

    # 2. Convert module ports into PORT cells.
    for port_name, port_data in module.get("ports", {}).items():
        for net in port_data.get("bits", []):
            port_cell = Cell(f"PORT_{port_name}",
                             default_sizes["PORT"][0],
                             default_sizes["PORT"][1],
                             default_sizes["PORT"][2],
                             0, 0, 0)
            cells.append(port_cell)
            net_to_endpoints.setdefault(net, []).append((port_cell, port_name))

    # 3. Build wire connections.
    wires = []
    for net, endpoints in net_to_endpoints.items():
        if len(endpoints) < 2:
            continue
        driver = endpoints[0][0]
        for endpoint in endpoints[1:]:
            sink = endpoint[0]
            if driver != sink:
                wires.append((driver, sink))

    return cells, wires


# -------------------------------------------------------------------
# Example / Demo
# -------------------------------------------------------------------
def main():
    # Create some sample 3D cells.
    # For example, assume we have a 3x3x2 board slice.
    cells = [
        Cell("A", 3, 3, 2, 0, 0, 1),
        Cell("B", 3, 3, 2, 8, 0, 1),
        Cell("C", 3, 3, 2, 0, 8, 1),
        Cell("D", 3, 3, 2, 8, 8, 1)
    ]

    # Create wires (each tuple is a connection between two cells).
    wires = [
        (cells[0], cells[1]),
        (cells[0], cells[2]),
        (cells[1], cells[3]),
        (cells[2], cells[3])
    ]

    # Try to find a placement and routing in 3D.
    state, cost = simulated_annealing_3d(cells, wires)
    if state is not None:
        print("Found a valid placement:")
        for i, cell in enumerate(cells):
            print(f"  {cell.name} at {state[i]}")
        routes, wire_cost = compute_wiring_routes_3d(cells, wires)
        if routes is not None:
            for i, route in enumerate(routes):
                print(f"Wire {i} route: {route}")
        render_placement_3d(cells, routes)
    else:
        print("No valid placement found. Best cost was:", cost)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Convert Yosys JSON to 3D placement cells and wires"
    )
    parser.add_argument("json_file", nargs="?",
                        help="Path to the Yosys JSON file (optional)")
    args = parser.parse_args()

    if args.json_file:
        # Use the provided YosysJSONParser to load the JSON.
        # Note: Uncomment the following lines if you have yosys_parser available.
        yosys_parser = YosysJSONParser(args.json_file)
        counter_module = yosys_parser.get_module("counter")
        if counter_module is None:
            print("Module 'counter' not found in the provided JSON.")
        else:
            cells, wires = convert_module_to_cells_and_wires(counter_module)
            print("Extracted Cells:")
            for cell in cells:
                print(f"  {cell}")
            print("\nExtracted Wires:")
            for source, target in wires:
                print(f"  {source.name} -> {target.name}")
            state, cost = simulated_annealing_3d(cells, wires)
            if state is not None:
                print("Found a valid placement:")
                for i, cell in enumerate(cells):
                    print(f"  {cell.name} at {state[i]}")
                routes, wire_cost = compute_wiring_routes_3d(cells, wires)
                if routes is not None:
                    for i, route in enumerate(routes):
                        print(f"Wire {i} route: {route}")
                render_placement_3d(cells, routes)
            else:
                print("No valid placement found. Best cost was:", cost)
        #
        # For now, if a JSON file is provided, we will print a message.
        print("JSON file input not implemented in this demo version.")
    else:
        main()
