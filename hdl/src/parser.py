import json
import random
import sys
import argparse
import pickle
from datetime import datetime
import os
from pathing import AStarSolver
import copy

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
                "x": 2,
                "y": 1,
                "z": 1
            },
            "B": {
                "x": 1,
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
                "x": 2,
                "y": 1,
                "z": 1
            },
            "B": {
                "x": 1,
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

# should make deep copy of layoutgenome first
# ie making new cells
# then do the mutations
class LayoutGenome:
    def __init__(self, cell_list, cell_lut, volume=[100, 100, 100],seed=420):
        self.cell_list = cell_list  # a list of Cell objects (with their positions)
        self.cell_lut = cell_lut    # a lookup for cells (by name)
        self.volume = volume
        self.fitness = None
        self.seed = seed
        if seed is None:
            random.seed()
        else:
            random.seed(seed);

    def mutate_movement(self, mutation_rate=0.1):
        """
        For each cell in the genome, slightly shift its position (if allowed)
        """
        layout = self.to_layout()
        for idx, cell in enumerate(self.cell_list):
            if random.random() < mutation_rate:
                # small delta chosen from -1, 0, or 1
                delta = [random.choice([-1, 0, 1]) for _ in range(3)]
                new_pos = cell.pos.copy()
                new_pos[0] += delta[0]
                new_pos[1] += delta[1]
                new_pos[2] += delta[2]

                # Ensure the new position does not violate volume bounds.
                if (
                    new_pos[0] < 0
                    or new_pos[1] < 0
                    or new_pos[2] < 0
                    or new_pos[0] + new_pos[3] > self.volume[0]
                    or new_pos[1] + new_pos[4] > self.volume[1]
                    or new_pos[2] + new_pos[5] > self.volume[2]
                ):
                    continue
                cell.pos = new_pos

    def random_cell_pos(self, cell):
        """
        Generate a completely new, random valid position for the given cell.
        Note: The effective cell dimensions are taken from CELL_KINDS with a 1-cell
        buffer on all sides.
        """
        if cell.kind not in CELL_KINDS:
            print(f"Kind {cell.kind} not found in CELL_KINDS!")
            sys.exit(1)
        dims = CELL_KINDS[cell.kind]
        ext_w = dims["x"] + 6  # extra space for air gap
        ext_d = dims["y"] + 6
        ext_h = dims["z"] + 6
        x = random.randint(0, self.volume[0] - ext_w)
        y = random.randint(0, self.volume[1] - ext_d)
        z = random.randint(0, self.volume[2] - ext_h)
        # Return the inner cell position (offset by 1) plus the cell dimensions.
        return [x + 3, y + 3, z + 3, dims["x"], dims["y"], dims["z"]]

    def mutate_rand_pos(self, mutation_rate=0.05):
        """
        Pick a random cell and set its position completely at random.
        """
        cell = random.choice(self.cell_list)
        if random.random() < mutation_rate:
            new_pos = self.random_cell_pos(cell)
            if new_pos:
                cell.pos = new_pos

    @staticmethod
    def crossbreed(genome1, genome2):
        """
        Create a new genome by combining two parent genomes.
        (Cells are assumed to be in the same order in both genomes.)
        """
        if len(genome1.cell_list) != len(genome2.cell_list):
            print("Genomes have different number of cells, cannot crossbreed")
            sys.exit(1)
        new_cell_list = []
        new_cell_lut = {}
        for cell1, cell2 in zip(genome1.cell_list, genome2.cell_list):
            # choose randomly from one of the parents
            chosen = cell1 if random.random() < 0.5 else cell2
            new_cell = copy.deepcopy(chosen)
            new_cell_list.append(new_cell)
            new_cell_lut[new_cell.name] = new_cell
        return LayoutGenome(new_cell_list, new_cell_lut, volume=genome1.volume)

    def to_layout(self):
        """
        Convert the genome into a full Layout instance. 'cells_config' and 'ports'
        are dictionaries parsed from JSON and used by Layout. If a cell is stored in
        the genome, a new Cell instance is created (with the genome’s cell position).
        """
        new_layout = Layout(cells={}, ports={}, volume=self.volume,seed=self.seed)
        new_layout.volume = self.volume
        new_layout.generated_cells = []
        new_layout.cells_lut = {}
        for cell in self.cell_list:
            new_cell = Cell(new_layout, cell.name, cell.kind,
                            ports=cell.ports, pos=cell.pos)
            new_layout.generated_cells.append(new_cell)
            new_layout.cells_lut[new_cell.name] = new_cell
        new_layout.rebuild_grid();
        new_layout.set_astar();
        return new_layout

    def compute_fitness(self):
        """
        Build a full layout from the genome and then generate wiring paths.
        If wiring fails (generate_paths returns -1), assign a heavy penalty.
        Otherwise, use the (negative) total wiring length as the fitness
        (lower wiring length is better).
        """
        res_length = -1
        max_len = -1
        try:
            layout_tmp = self.to_layout()
            if layout_tmp.overlap > 0:
                return -100 * layout_tmp.overlap
            res_length, max_len = layout_tmp.generate_paths()
        finally:
            if res_length == -1:
                return -1e6
        # Fitness is defined so that a lower total wiring length gives a
        # higher fitness.
        return -res_length


class Layout:
    def __init__(self,cells={}, ports={}, volume=[50,50,50],seed=420):
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
        self.gen_wires = []
        self.cells_lut = {}
        self.overlap = 0
        self.a_star = None;
        self.seed = seed
        if (seed is None):
            random.seed()
        else:
            random.seed(self.seed)



    def to_genome(self,seed=420):
        """
        Create and return a deep copy of this layout's genome.
        This copies the generated_cells and rebuilds the cell lookup
        dictionary, so that modifications to the genome do not affect
        the original layout.
        """
        # Create a deep copy of the generated cells list
        cells_copy = copy.deepcopy(self.generated_cells)

        # Rebuild the lookup dictionary using the copied cells.
        cell_lut_copy = {cell.name: cell for cell in cells_copy}
        if seed is not None:
            return LayoutGenome(cells_copy, cell_lut_copy, volume=self.volume,seed=seed)
        else:
            return LayoutGenome(cells_copy, cell_lut_copy, volume=self.volume,seed=self.seed)

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

    def set_astar(self):
        self.a_star = AStarSolver(((0,0,0),(self.volume[0],self.volume[1],self.volume[1])),self.get_kind)

    def generate_start_layout(self):
        # self.serialize();
        self.a_star = AStarSolver(((0,0,0),(self.volume[0],self.volume[1],self.volume[1])),self.get_kind)
        # start with ports
        for port, body in self.ports.items():
            cell_name = port
            kind = "PORT_IN" if body['direction'] == 'input' else 'PORT_OUT'
            cell = Cell(self,port,kind)
            self.generated_cells.append(cell)
            self.cells_lut[cell_name] = cell
            if( not self.fill_volume(cell) ):
                print(f"Could not fill cell {cell.name} volume {cell.pos}")



        # then do cells
        for cell, body in self.cells.items():
            cell_name = cell
            cell = Cell(self,cell,body['type'])
            self.generated_cells.append(cell)
            self.cells_lut[cell_name] = cell
            if( not self.fill_volume(cell) ):
                print(f"Could not fill cell {cell.name} volume {cell.pos}")

    def rebuild_grid(self):
        overlap = 0
        self.grid = [None for e in range(self.volume[0]*self.volume[1]*self.volume[1])]
        for cell in self.generated_cells:
            overlap += self.fill_volume_compute_overlap(cell)
        self.overlap = overlap


    # used for fitness
    def generate_paths(self):
        """
        Generate output paths from the generated cells and process them.
        Only return after a fixed number of iterations (or if no failed paths).
        """
        output_paths = self._generate_output_paths()
        routed_paths = self._solve_generated_paths(output_paths)
        if routed_paths is None:
            return -1, -1

        path_len = 0
        num_paths = 0
        for p in routed_paths:
            if len(p) != 0:
                num_paths += 1
                path_len += len(p)
        return path_len, num_paths

    def _generate_output_paths(self):
        """
        Iterate over all generated cells to build the list of output paths.
        Each output path is a tuple: ((startx, starty, startz), (endx, endy, endz)).
        """
        output_paths = []
        for cell in self.generated_cells:
            kind = CELL_KINDS[cell.kind]
            if "outputs" not in kind:
                continue
            for out_name, out_rpos in kind["outputs"].items():
                startx = out_rpos["x"] + cell.pos[0]
                starty = out_rpos["y"] + cell.pos[1]
                startz = out_rpos["z"] + cell.pos[2]
                # Use the output name to get all the destination ports
                dests = cell.ports[out_name]
                for dest in dests:
                    dname = dest[0]
                    dport = dest[1]
                    dcell = self.cells_lut[dname]
                    dkind = CELL_KINDS[dcell.kind]
                    if dport not in dkind["inputs"]:
                        print(f"wire from output {cell.name} does not go to input at {dname}")
                    else:
                        end_pos = dkind["inputs"][dport]
                        endx = end_pos["x"] + dcell.pos[0]
                        endy = end_pos["y"] + dcell.pos[1]
                        endz = end_pos["z"] + dcell.pos[2]
                        output_paths.append(
                            ((startx, starty, startz), (endx, endy, endz))
                        )
        print(f"{len(output_paths)} to solve")
        return output_paths

    def _solve_generated_paths(self, output_paths):
        """
        Attempt to solve paths in multiple iterations.
        In each iteration:
        - Save the current grid and gen_wires state.
        - Call _attempt_paths() which returns successful and failed paths along
            with the total length and the maximum length found in this attempt.
        - For every successful path that is longer than a minimal length, generate
            an extension path that will be attempted in the next iteration.
        - Combine the failed paths and extensions to form the set of paths for the
            next iteration.

        The loop terminates if no paths remain or after a maximum number of iterations.
        Returns a tuple of (overall_total_length, overall_max_path_length).
        """
        max_iterations = 5  # or any iteration limit you desire
        iteration = 0
        result_paths = []
        failed_paths = [0]
        saved_grid = [i for i in self.grid]

        base_saved_grid = [i for i in self.grid]
        base_saved_wires = [i for i in self.gen_wires]
        failed_tracker = {}
        input_paths = [ p for p in output_paths]
        fail_repeat_reset_threshold = 2
        while len(failed_paths) > 0:
            if iteration >= max_iterations:
                return None

            print(f"--- Iteration {iteration+1}: "
                f"{len(output_paths)} paths to attempt ---")

            # Save states for potential rollback or debugging.
            self.grid = [i for i in saved_grid]
            self.gen_wires = []
            self.a_star.get_kind = self.get_kind


            # Attempt to solve all given paths.
            successful_paths, failed_paths = (
                self._attempt_paths(output_paths)
            )

            if (len(failed_paths) > 0):
                if iteration == 0:
                    iteration += 1

                    self.grid = [i for i in saved_grid]
                    self.gen_wires = []
                    self.a_star.get_kind = self.get_kind
                    output_paths = []
                    for f in failed_paths:
                        output_paths.append(f)
                    for s in successful_paths:
                        output_paths.append((s[0],s[1]))


                self.grid = [i for i in saved_grid]
                self.gen_wires = []
                self.a_star.get_kind = self.get_kind

                # For every successful path, try to extend it a little further.
                new_extensions = []
                new_results = {}
                for path in successful_paths:
                    # For example, if the solved path is long, take a segment extension.
                    if path[0] not in new_results:
                        new_results[path[0]] = { 'paths':[], 'path': None, 'new_start': None, 'new_goal': None }
                    new_results[path[0]]['paths'].append(path)
                    new_results[path[0]]['path'] = path[:2]
                    new_results[path[0]]['new_start'] = path[2]
                    new_results[path[0]]['new_goal'] = path[-1]

                for entry in new_results.values():
                    result_paths.append(entry['path'])
                    self.fill_path(entry['path'],2)
                    for epath in entry['paths']:
                        # Using the third node onward as an "extension" to solve further.
                        new_extensions.append((entry['path'][0], epath[-1]))


                # Combine failed paths and extensions for the next iteration.
                output_paths = []
                continue_me = False
                for p in failed_paths:
                    output_paths.append(p)
                    if p not in failed_tracker:
                        failed_tracker[p] = 1
                    else:
                        failed_tracker[p] += 1

                    if failed_tracker[p] > fail_repeat_reset_threshold:
                        # we are going to reset
                        self.grid = [i for i in base_saved_grid]
                        saved_grid = [i for i in base_saved_grid]
                        self.gen_wires = []
                        failed_paths = [0]
                        failed_sort = sorted(failed_tracker.items, reverse=True)
                        output_paths = []
                        for s in failed_sort:
                            if s[0] in input_paths:
                                output_paths.append(s[0])
                        for s in input_paths:
                            if s not in output_paths:
                                output_paths.append(s)
                        failed_tracker = {}
                        continue_me = True
                        break

                if continue_me:
                    continue

                for p in new_extensions:
                    output_paths.append(p)
                saved_grid = [i for i in self.grid]
            else:
                for p in successful_paths:
                    result_paths.append(p)
                    pass

            iteration += 1
        self.gen_wires = [w for w in result_paths]
        return result_paths


    def _attempt_paths(self, paths, limit=100000,verbose=False):
        """
        Attempt to solve a list of paths using A* with a given search limit.
        For each path:
        - Try the 'fast' instance of A* (using limit).
        - If that fails, try the reverse direction.
        - If still unresolved, try with no limit (limit=-1).
        - Add successfully solved paths to the list and fill the path on the grid.
        - For paths that remain unsolved, add them to the failed_paths list.

        Returns:
        successful_paths (list of solved paths),
        failed_paths (list of unresolved paths),
        total_length (cumulative length of all solved paths),
        max_length (length of the longest solved path).
        """
        successful_paths = []
        failed_paths = []

        for idx, path in enumerate(paths):
            if verbose:
                print(f"Attempting path {idx+1}/{len(paths)}: "
                    f"{path[0]} -> {path[1]}")
            solved = None

            try:
                solved = self.a_star.solve(path[0], path[1], limit)
                if solved is False:
                    solved = self.a_star.solve(path[1], path[0], limit)
                    if isinstance(solved, list):
                        solved.reverse()
            except Exception as e:
                failed_paths.append(path)
                print(f"Fast attempt {limit} searches: {e}")
            else:

                if solved and solved is not False:
                    successful_paths.append(solved)
                    self.gen_wires.append(solved)
                    self.fill_path(solved, -1)
                else:
                    # Try a backup attempt with no limit.
                    try:
                        solved = self.a_star.solve(path[0], path[1], -1)
                    except Exception as e:
                        print(f"Full attempt failed: {e}")
                        solved = False

                    if solved and solved is not False:
                        successful_paths.append(solved)
                        self.gen_wires.append(solved)
                        self.fill_path(solved, -1)
                    else:
                        failed_paths.append(path)
                        print(f"Path {idx+1} failed all attempts.")

        return successful_paths, failed_paths


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

    def fill_path(self, path, n,my_grid=True):
        if n == -1:
            n = 1000000
        path_id = (path[0], path[-1]), path
        _, *r_path, _2 = path
        ctr = 0
        for step in r_path:
            if (ctr > n):
                break
            x = step[0]
            y = step[1]
            z = step[2]
            ctr += 1
            for d in [[0, 0, 0],
                      [0, 0, 1],
                      [0, 0, -1],
                      [1, 0, 0],
                      [-1, 0, 0],
                      [0, 1, 0],
                      [0, -1, 0]
                      ]:
                dx, dy, dz = d
                filled = self.get_kind(x+dx,y+dx,z+dz)
                if isinstance(filled, tuple):
                    fp, fstate, _, fpp = filled
                    if fstate == False or fstate == 'top':
                        if d == [0,0,0]:
                            print(f"tried to place wire in illegal position")

                    elif fstate == 'top':
                        if dz == -1:
                            tpid = path_id[0], False, (x,y,z), path_id[1]
                            self.grid[self.get_index(x+dx,y+dy,z+dz)] = tpid

                        if d == [0,0,0]:
                            tpid = path_id[0], True, (x,y,z), path_id[1]
                            self.grid[self.get_index(x+dx,y+dy,z+dz)] = tpid
                    else:
                        if d == [0,0,0] and fstate != True:
                            tpid = path_id[0], True, (x,y,z), path_id[1]
                            self.grid[self.get_index(x+dx,y+dy,z+dz)] = tpid


                    # if(filled != path_id):
                        # print("tried to overlay two paths!")
                    continue
                elif filled is None:
                    tpid = path_id
                    if (d == [0,0,0]):
                        tpid = path_id[0], True, (x,y,z), path_id[1]
                    elif dz == -1:
                        tpid = path_id[0], False, (x,y,z), path_id[1]
                    elif dz == 0:
                        tpid = path_id[0], None, (x,y,z), path_id[1]
                    elif dz == 1:
                        tpid = path_id[0], 'top', (x,y,z), path_id[1]
                    else:
                        print("impossible state reached in parsing")
                        sys.exit("aa");
                    self.grid[self.get_index(x+dx,y+dy,z+dz)] = tpid

                else:
                    pass
        if my_grid:
            return None
        else:
            return grid
                    # print(f"path trying to place at {x} {y} {z} object {self.grid[self.get_index(x,y,z)]} present")
    def fill_volume_compute_overlap(self, cell):
        x = cell.pos[0]
        y = cell.pos[1]
        z = cell.pos[2]
        width = cell.pos[3]
        depth = cell.pos[4]
        height = cell.pos[5]
        error_counter = 0

        if(width + x >= self.volume[0]):
            diff = width + x - self.volume[0]
            error_counter += depth*height*diff
            width = width - diff -1 if diff -1 > 0 else 0

        if(depth + y >= self.volume[1]):
            diff = depth + y - self.volume[1]
            error_counter += width*height*diff
            depth= depth - diff -1 if diff -1 > 0 else 0

        if(height + z >= self.volume[2]):
            diff = height + z - self.volume[2]
            error_counter += width*depth*diff
            height = height - diff -1 if diff -1 > 0 else 0

        for dx in range(width):
            for dy in range(depth):
                for dz in range(height):
                    idx = self.get_index(dx+x,dy+y,dz+z);
                    if self.grid[idx] != None:
                        error_counter += 1

        for dx in range(width):
            for dy in range(depth):
                for dz in range(height):
                    idx = self.get_index(dx+x,dy+y,dz+z);
                    self.grid[idx] = str(cell.name)

        return error_counter


    def fill_volume(self, cell):
        x = cell.pos[0]
        y = cell.pos[1]
        z = cell.pos[2]
        width = cell.pos[3]
        depth = cell.pos[4]
        height = cell.pos[5]
        error_counter = 0
        if(width + x >= self.volume[0]):
            error_counter += 1000
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
                    self.grid[idx] = str(cell.name)
                    if self.grid[idx] == None:
                        self.grid[idx] = True

        return True

    def get_kind(self, x, y, z):
        if(1 + x >= self.volume[0]):
            return False

        if(1 + y >= self.volume[1]):
            return False

        if(1 + z >= self.volume[2]):
            return False

        idx = self.get_index(x,y,z);
        if self.grid[idx] != None:
            if isinstance(self.grid[idx], tuple):
                return self.grid[idx]
            else:
                return False
        else:
            return None


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
        width = kind['x'] + 6;
        height = kind['z'] + 6;
        depth = kind['y'] + 6;

        for _ in range(trials):
            # random position
            rpos_x = random.randint(0, self.volume[0] - width)
            rpos_y = random.randint(0, self.volume[1] - height)
            rpos_z = random.randint(0, self.volume[2] - depth)
            if (self.volume_empty(width,height,depth,rpos_x,rpos_y,rpos_z) == True):
                # center cell and reduce vol
                return [rpos_x+3, rpos_y+3, rpos_z+3, width-6, depth-6, height-6]

        # no pos found in n random trials. checking every position
        if not every_pos:
            return None

        for x in range(self.volume[0]):
            for y in range(self.volume[1]):
                for z in range(self.volume[2]):
                    if (self.volume_empty(width,height,depth,x,y,z) == True):
                        # center cell and reduce vol
                        return [x+3, y+3, z+3, width-6, depth-6, height-6]

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
        self.ports = ports
