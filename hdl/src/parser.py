import json
import random
import sys
import argparse
import pickle
from datetime import datetime
import os

## TODO @(dleiferives,4b111f97-d7e1-484f-8a5c-6d4680635be1): add rotations ~#

CELL_KINDS= {
    ## TODO @(dleiferives,7e67c6a6-6973-4872-ab9d-5e46375a07e9): add generationg
    ## of port cells automatically after parsing ports ~#
    "PORT_IN": {
        "x":1,
        "y":1,
        "z":2,
        "outputs": {
            "OUT": {
                "x": 0,
                "y": 0,
                "z": 1
            }
        }
    },
    "PORT_OUT": {
        "x":1,
        "y":1,
        "z":2,
        "inputs": {
            "IN": {
                "x": 0,
                "y": 0,
                "z": 1
            }
        }
    },
    "NAND": {
        "x":3,
        "y":3,
        "z":3,
        "inputs": {
            "A": {
                "x": 0,
                "y": 0,
                "z": 1
            },
            "B": {
                "x": 2,
                "y": 0,
                "z": 1
            }
        },
        "outputs": {
            "Y": {
                "x": 1,
                "y": 2,
                "z": 1
            }
        }
    },
    "NOR": {
        "x":3,
        "y":3,
        "z":3,
        "inputs": {
            "A": {
                "x": 0,
                "y": 0,
                "z": 1
            },
            "B": {
                "x": 2,
                "y": 0,
                "z": 1
            }
        },
        "outputs": {
            "Y": {
                "x": 1,
                "y": 2,
                "z": 1
            }
        }
    },
    "BUF": {
        "x":1,
        "y":1,
        "z":2,
        "inputs": {
            "A": {
                "x": 0,
                "y": 0,
                "z": 1
            }
        },
        "outputs": {
            "Y": {
                "x": 2,
                "y": 0,
                "z": 1
            }
        }
    },
    "NOT": {
        "x":3,
        "y":1,
        "z":2,
        "inputs": {
            "A": {
                "x": 0,
                "y": 0,
                "z": 1
            }
        },
        "outputs": {
            "Y": {
                "x": 2,
                "y": 0,
                "z": 1
            }
        }
    },
    "DFF": {
        "x":3,
        "y":3,
        "z":3,
        "inputs": {
            "D": {
                "x": 0,
                "y": 0,
                "z": 1
            },
            "C": {
                "x": 2,
                "y": 0,
                "z": 1
            }
        },
        "outputs": {
            "Q": {
                "x": 1,
                "y": 2,
                "z": 1
            }
        }
    },

}


class Layout:
    def __init__(self,cells, ports, volume=[100,100,100]):
        # cells is a dict of names -> things
        self.cells = cells
        self.ports = ports
        self.volume = volume
        self.wires = {}
        self.wires_lut = {}
        # "bit": [(NAME, MEMBER NAME)]
        temp = []
        for x in range(volume[0]*volume[1]*volume[2]):
            temp.append(None);
        self.grid = temp
        self.generated_cells = []

    def deserialize_layout(filename):
        with open(filename, 'rb') as file:
            loaded_layout = pickle.load(file)
        return loaded_layout

    def serialize(self):
        current_utc = datetime.utcnow()
        date_string = current_utc.strftime("%Y%m%d_%H%M%S")

        filename = f"layout_{str(self.volume[1])}x{str(self.volume[1])}x{str(self.volume[1])}_{date_string}.pkl"

        with open(filename, 'wb') as file:
            pickle.dump(self, file)

        return filename

    def generate(self):
        self.serialize();
        # start with ports
        for port, body in self.ports.items():
            kind = "PORT_IN" if body['direction'] == 'input' else 'PORT_OUT'
            cell = Cell(self,port,kind)
            self.generated_cells.append(cell)
            if( not self.fill_volume(cell) ):
                print(f"Could not fill cell {cell.name} volume {cell.pos}")



        # then do cells
        for cell, body in self.cells.items():
            cell = Cell(self,cell,body['type'])
            self.generated_cells.append(cell)
            if( not self.fill_volume(cell) ):
                print(f"Could not fill cell {cell.name} volume {cell.pos}")

        ## TODO @(dleiferives,e48135de-37d3-47a8-9d9c-03b1bad26537): then do wires ~#
        return 0;


    def parse_ports(self,ports):
        if ports is None:
            return
        for port, body in ports.items():
            self.ports[port] = body
            if port not in self.wires:
                self.wires[port] = { 'connections': {}, 'bits': []}
            direction = 'OUT' if body['direction'] == 'input' else 'IN'
            self.wires[port]['connections'][direction] = body['bits'][0]
            self.wires[port]['bits'].append(body['bits'][0])
            if str(body['bits'][0]) not in self.wires_lut:
                self.wires_lut[str(body['bits'][0])] = []
            self.wires_lut[str(body['bits'][0])].append((port, direction));

    def parse_cells(self,cells):
        if cells is None:
            return
        for cell, body in cells.items():
            self.cells[cell] = body
            if cell not in self.wires:
                self.wires[cell] = {'connections': {}, 'bits': []}
            for con_name, con_val in body['connections'].items():
                self.wires[cell]['connections'][con_name] = con_val[0];
                self.wires[cell]['bits'].append(con_val[0])
                if str(con_val[0]) not in self.wires_lut:
                    self.wires_lut[str(con_val[0])] = []
                self.wires_lut[str(con_val[0])].append((cell, con_name));


    def parse_json(self,contents):
        if "modules" not in contents:
            exit("No modules");
        contents = contents['modules']
        ## TODO @(dleiferives,823a4f84-53ff-4602-8ff9-f93688e68854): make this not
        ## just be the first entry ~#
        for key in contents.keys():
            contents = contents[key]
            break
        self.parse_ports(contents['ports']);
        self.parse_cells(contents['cells'])

    def get_index(self,x,y,z):
        return z*self.volume[0]*self.volume[1] + y*self.volume[0] + x;

    def fill_volume(self, cell):
        x = cell.pos[0]
        y = cell.pos[1]
        z = cell.pos[2]
        width = cell.pos[3]
        depth = cell.pos[4]
        height = cell.pos[5]
        if(width + x >= self.volume[0]):
            return False

        if(depth + y >= self.volume[1]):
            return False

        if(height + z >= self.volume[2]):
            return False

        for dx in range(width):
            for dy in range(depth):
                for dz in range(height):
                    idx = self.get_index(dx+x,dy+y,dz+z);
                    if self.grid[idx] != None:
                        return False

        for dx in range(width):
            for dy in range(depth):
                for dz in range(height):
                    idx = self.get_index(dx+x,dy+y,dz+z);
                    self.grid[idx] = cell.name

        return True
    def volume_empty(self, width, height, depth, x, y, z):
        #/ TODO @(dleiferives,2568b396-654f-493a-ae21-e6215aee77a3): optimize this ~#
        if(width + x >= self.volume[0]):
            return False

        if(depth + y >= self.volume[1]):
            return False

        if(height + z >= self.volume[2]):
            return False

        for dx in range(width):
            for dy in range(depth):
                for dz in range(height):
                    idx = self.get_index(dx+x,dy+y,dz+z);
                    if self.grid[idx] != None:
                        return False

        return True


    def new_cell_pos(self,kind,trials=1000,every_pos=False):
        if kind not in CELL_KINDS:
            print(f"kind {kind} not found in kinds {CELL_KINDS.keys()}");
            sys.exit(1)

        kind = CELL_KINDS[kind];
        # oversize the volume so there is air gap
        width = kind['x'] + 2;
        height = kind['z'] + 2;
        depth = kind['y'] + 2;

        for _ in range(trials):
            # random position
            rpos_x = random.randint(0, self.volume[0] - width)
            rpos_y = random.randint(0, self.volume[1] - height)
            rpos_z = random.randint(0, self.volume[2] - depth)
            if (self.volume_empty(width,height,depth,rpos_x,rpos_y,rpos_z) == True):
                # center cell and reduce vol
                return [rpos_x+1, rpos_y+1, rpos_z+1, width-2, depth-2, height-2]

        # no pos found in n random trials. checking every position
        if not every_pos:
            return None

        for x in range(self.volume[0]):
            for y in range(self.volume[1]):
                for z in range(self.volume[2]):
                    if (self.volume_empty(width,height,depth,x,y,z) == True):
                        # center cell and reduce vol
                        return [x+1, y+1, z+1, width-2, depth-2, height-2]

        return None

    def get_cell_ports(self, cell_name):
        if cell_name not in self.wires:
            sys.exit("cell name not found in wires!")

        result = {}
        wire = self.wires[cell_name]
        for con_name, bit in wire['connections'].items():
            other_bit = []
            lut = self.wires_lut[str(bit)]
            for entry in lut:
                if entry[0] != cell_name:
                    other_bit.append(entry)
            result[con_name] = other_bit

        return result



class Cell:
    # Layout is layout parent
    # kind is one of the cell cinds
    # name is the name from the json eg "$abc$129$auto$blifparse.cc:396:parse_blif$137"
    # ports is {
    #   "<port name ie A>": [(<cell connecting to name>, <cell connecting to port name connecting to>)]
    # }
    # pos is [x, y, z, width, depth, height]
    def __init__(self,layout,name,kind,ports=None,pos=None):
        self.layout = layout
        self.kind = kind;
        self.name = name
        if pos == None:
            pos = layout.new_cell_pos(kind)
        if pos == None:
            print(f"Could not find position for cell {kind},{id},{pos},{ports},{connections}")
            sys.exit(1)
        self.pos = pos

        if ports == None:
            ports = layout.get_cell_ports(name)
        if ports == None:
            ports = {}
            print(f"WARNING: no ports for cell {name}")
