from typing import List, Tuple, Optional
import nbtlib
from nbtlib import Compound, File
from nbtlib.tag import Int, IntArray, List

from parser import Layout, Cell, CELL_KINDS
import sys

## TODO @(dleiferives,db9ba75f-58a1-47f4-a517-143152e5dae4): Add cell placement
## stuff/ layout design or smnthn ~#

class NBTGenerator:
    def __init__(self, layout: Layout):
        self.layout = layout

    def generate_file(self,filename:str, layout=None):
        if(layout == None):
            layout = self.layout
        if(layout == None):
            print("No layout to generate")
            return


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

        width  = layout.volume[0]
        length = layout.volume[1]
        height = layout.volume[2]

        grid = [
                [
                ["minecraft:air" for _ in range(length)]
                for _ in range(height)
                ]
                for _ in range(width)
        ]

        cells = layout.generated_cells;
        # Mark cell volumes as white wool.
        for cell in cells:
                c_x = cell.pos[0]
                c_y = cell.pos[1]
                c_z = cell.pos[2]
                c_width = cell.pos[3]
                c_depth = cell.pos[4]
                c_height = cell.pos[5]
                for dx in range(c_width):
                        for dy in range(c_depth):
                                for dz in range(c_height):
                                        sim_x = c_x + dx
                                        sim_y = c_y + dy
                                        sim_z = c_z + dz
                                        grid[sim_x][sim_y][sim_z] = "minecraft:white_wool"
        wires = layout.gen_wires
        for wire in wires:
            for step in wire:
                tx = step[0]
                ty = step[1]
                tz = step[2]
                grid[tx][ty][tz] = "minecraft:red_wool"

        # # Mark wire route voxels as red wool (overriding any cell fill).
        # for route in routes:
        #         for (x, y, z) in route:
        #                 sx = x - grid_min_x
        #                 sy = z - grid_min_z  # simulation z becomes schematic Y
        #                 sz = y - grid_min_y  # simulation y becomes schematic Z
        #                 grid[sx][sy][sz] = "minecraft:red_wool"

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
                                        nbtlib.tag.Int(sz),
                                        nbtlib.tag.Int(sy)
                                ]),
                                "state": nbtlib.tag.Int(state_index)
                                })
                                blocks.append(block_compound)
## TODO @(dleiferives,6095c635-a651-47b6-9cf3-fc4f8fcaf1f6): add ports as red wool
## next ~#

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
