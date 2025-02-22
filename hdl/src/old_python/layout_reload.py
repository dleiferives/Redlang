from parser import CELL_KINDS, Layout, Cell
from nbt_manager import NBTGenerator
import argparse
import json
import sys
import os

def main():
    parser = argparse.ArgumentParser(description="Minecraft Yosys layout generator")
    parser.add_argument("pkl_file", help="Path to the Yosys JSON file")
    args = parser.parse_args()
    layout = Layout.deserialize_layout(args.pkl_file);
    layout.generate();
    nbt_gen = NBTGenerator(layout);
    nbt_gen.generate_file(args.pkl_file+"_layout.nbt")


if __name__ ==  '__main__':
    main()
