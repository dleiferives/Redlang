from parser import CELL_KINDS, Layout, Cell
from nbt_manager import NBTGenerator
import json
import argparse
import os
import sys

def main():
    parser = argparse.ArgumentParser(description="Minecraft Yosys layout generator")
    parser.add_argument("json_file", help="Path to the Yosys JSON file")
    args = parser.parse_args()
    json_data = None
    try:
        with open(args.json_file, 'r') as f:
            json_data = json.load(f)
    except FileNotFoundError:
        print(f"Error: File not found at '{args.json_file}'")
        sys.exit(1);
    except json.JSONDecodeError:
        print(f"Error: Invalid JSON format in '{args.json_file}'")
        sys.exit(1)

    layout = Layout(cells={},ports={})
    layout.parse_json(json_data)
    layout.generate_start_layout();
    nbt_gen = NBTGenerator(layout);
    nbt_gen.generate_file(args.json_file+"_layout.nbt")
    layout.generate_paths();
    nbt_gen = NBTGenerator(layout);
    nbt_gen.generate_file(args.json_file+"_layout.nbt")


if __name__ ==  '__main__':
    main()
