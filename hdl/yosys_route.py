import argparse
from yosys_parser import YosysJSONParser  # Assuming yosys_parser.py is in the same directory
from placement import Cell, PlacementAlgorithm  # Assuming placement.py is in the same directory

def route_design(json_file, grid_width, grid_height, max_wire_length, algorithm="greedy"):
    """
    Routes a design based on a Yosys JSON file.

    Args:
        json_file (str): Path to the Yosys JSON file.
        grid_width (int): Width of the placement grid.
        grid_height (int): Height of the placement grid.
        max_wire_length (int): Maximum wire length.
        algorithm (str): Placement algorithm ("random" or "greedy").
    """
    parser = YosysJSONParser(json_file)

    if not parser.json_data:
        print("Error: Could not load Yosys JSON file.  Exiting.")
        return

    # Extract cell information and connections from the Yosys JSON
    cells = []
    # Assuming the top-level module is the one we want to place
    top_module_name = next(iter(parser.modules))  # Get the first module name
    top_module = parser.get_module(top_module_name)

    if not top_module:
        print(f"Error: Could not find top module '{top_module_name}' in the JSON file. Exiting.")
        return

    cell_name_map = {} # Map Yosys cell names to our Cell objects
    for cell_name, cell_data in top_module["cells"].items():
        if cell_data.get("type") not in ("DFF", "NOT", "NAND", "NOR"): # Filter out unwanted cells
            continue
        new_cell = Cell(cell_name)
        cells.append(new_cell)
        cell_name_map[cell_name] = new_cell

    # Build connections based on netnames and cell connections
    for cell_name, cell_data in top_module["cells"].items():
        if cell_data.get("type") not in ("DFF", "NOT", "NAND", "NOR"):
            continue
        for connection_port, net_indices in cell_data.get("connections", {}).items():
            for net_index in net_indices:
                for netname, netname_data in top_module["netnames"].items():
                    if net_index in netname_data.get("bits", []):
                        # Find cells connected to this net
                        for other_cell_name, other_cell_data in top_module["cells"].items():
                            if other_cell_name == cell_name:
                                continue # Don't connect a cell to itself
                            if other_cell_data.get("type") not in ("DFF", "NOT", "NAND", "NOR"):
                                continue
                            for other_connection_port, other_net_indices in other_cell_data.get("connections", {}).items():
                                if net_index in other_net_indices:
                                    # Connect the cells
                                    if cell_name in cell_name_map and other_cell_name in cell_name_map:
                                        cell_name_map[cell_name].connections.append(other_cell_name)

    # Create and run the placement algorithm
    placement = PlacementAlgorithm(cells, grid_width, grid_height, max_wire_length)
    placement.place_cells(algorithm=algorithm)
    placement.display()

    # Print the placement results
    placement.print_placement()
    total_wire_length = placement.calculate_wire_length()
    print(f"\nTotal Wire Length: {total_wire_length}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Yosys-based Placement and Routing")
    parser.add_argument("json_file", help="Path to the Yosys JSON file")
    parser.add_argument("--grid_width", type=int, default=50, help="Width of the placement grid")
    parser.add_argument("--grid_height", type=int, default=50, help="Height of the placement grid")
    parser.add_argument("--max_wire_length", type=int, default=10, help="Maximum wire length")
    parser.add_argument("--algorithm", choices=["random", "greedy"], default="greedy", help="Placement algorithm")
    args = parser.parse_args()

    route_design(args.json_file, args.grid_width, args.grid_height, args.max_wire_length, args.algorithm)
