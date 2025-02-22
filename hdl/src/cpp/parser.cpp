// main.cpp
#include <algorithm>
#include <array>
#include <chrono>
#include <ctime>
#include <exception>
#include <fstream>
#include <functional>
#include <iostream>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>
#include <optional>

// Include our refactored A* solver header.
#include "astar_solver.h"

// Global verbosity flag.
bool verbose = false;

// ---------------------------------------------------------------------
// Data structures for positions and cell kinds
// ---------------------------------------------------------------------

// Structure describing a cell’s placement and dimensions.
struct Position {
    int x, y, z;  // starting coordinates
    int w, d, h;  // dimensions: width, depth, height
};

// We use Coord3 (a std::array<int,3>) as the coordinate type in both our A* solver
// (see astar_solver.cpp) and throughout the layout code.
using Coord3 = std::array<int, 3>;

// A cell “kind” definition.
struct CellKind {
    int x;
    int y;
    int z;
    // Definitions for I/O ports.
    std::unordered_map<std::string, std::array<int, 3>> inputs;
    std::unordered_map<std::string, std::array<int, 3>> outputs;
};

// Global mapping of cell kinds.
std::unordered_map<std::string, CellKind> CELL_KINDS = {
    { "PORT_IN", { 1, 1, 2, {},
                   {{"OUT", {0, 0, 1}}} } },
    { "PORT_OUT", { 1, 1, 2, {{"IN", {0, 0, 1}}},
                    {} } },
    { "NAND", { 3, 3, 3, {{"A", {2, 1, 1}}, {"B", {1, 0, 1}}},
                {{"Y", {1, 2, 1}}} } },
    { "NOR", { 3, 3, 3, {{"A", {2, 1, 1}}, {"B", {1, 0, 1}}},
               {{"Y", {1, 2, 1}}} } },
    { "BUF", { 1, 1, 2, {{"A", {0, 0, 1}}},
               {{"Y", {2, 0, 1}}} } },
    { "NOT", { 3, 1, 2, {{"A", {0, 0, 1}}},
               {{"Y", {2, 0, 1}}} } },
    { "DFF", { 3, 3, 3, {{"D", {0, 0, 1}}, {"C", {2, 0, 1}}},
               {{"Q", {1, 2, 1}}} } }
};

// ---------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------

class Layout;
class Cell;
class LayoutGenome;

// ---------------------------------------------------------------------
// Cell class
// ---------------------------------------------------------------------

class Cell {
public:
    Layout* layout;
    std::string kind;
    std::string name;
    // Ports: mapping a port name (like "A") to a list of (destination cell, destination port) pairs.
    std::unordered_map<std::string, std::vector<std::pair<std::string, std::string>>> ports;
    Position pos;

    Cell(Layout* layout, const std::string &name, const std::string &kind,
         const std::unordered_map<std::string, std::vector<std::pair<std::string, std::string>>>& ports = {},
         const Position* pos_ptr = nullptr);
    Cell(const Cell &other)
        : layout(other.layout), kind(other.kind), name(other.name),
          ports(other.ports), pos(other.pos) {}
};

Cell::Cell(Layout* layout, const std::string &name, const std::string &kind,
           const std::unordered_map<std::string, std::vector<std::pair<std::string, std::string>>>& ports,
           const Position* pos_ptr)
    : layout(layout), name(name), kind(kind), ports(ports)
{
    if (pos_ptr == nullptr) {
        // Ask the layout for a valid random position.
        pos = layout->new_cell_pos(kind);
    } else {
        pos = *pos_ptr;
    }
    // If no port info was provided, try getting it from the layout.
    if (this->ports.empty()) {
        this->ports = layout->get_cell_ports(name);
        if (this->ports.empty() && verbose)
            std::cout << "WARNING: no ports for cell " << name << std::endl;
    }
}

// ---------------------------------------------------------------------
// Layout class
// ---------------------------------------------------------------------

class Layout {
public:
    std::unordered_map<std::string, std::unordered_map<std::string, std::string>> cells;
    std::unordered_map<std::string, std::unordered_map<std::string, std::string>> ports;
    std::array<int, 3> volume;
    int seed;

    // Our grid is stored as a 1D vector of strings (empty string means unoccupied).
    std::vector<std::string> grid;

    // Generated (placed) cells.
    std::vector<Cell*> generated_cells;
    std::unordered_map<std::string, Cell*> cells_lut;
    int overlap;

    // Wiring paths.
    std::vector<std::vector<Coord3>> gen_wires;

    // Pointer to our A* solver (from astar_solver.cpp).
    AStarSolver* a_star;

    // Random number generator.
    std::mt19937 rng;

    Layout(const std::array<int, 3>& vol = {50, 50, 50}, int seed = 420);
    ~Layout();

    LayoutGenome* to_genome(int seed = 420);
    std::string serialize();
    static Layout* deserialize_layout(const std::string &filename);

    // -----------------------------------------------------------------
    // Updated to use our new A* solver.
    // set_astar sets up the A* solver using a lambda that calls our get_kind.
    void set_astar();

    void generate_start_layout();
    void rebuild_grid();
    std::pair<int,int> generate_paths();

    std::vector<std::pair<Coord3, Coord3>> _generate_output_paths();
    std::pair<std::vector<std::vector<Coord3>>, std::vector<std::pair<Coord3, Coord3>>>
        _attempt_paths(const std::vector<std::pair<Coord3, Coord3>> &paths, int limit = 100000, bool verbose_flag = false);
    std::vector<std::vector<Coord3>> _solve_generated_paths(const std::vector<std::pair<Coord3, Coord3>> &output_paths, bool large_log = false);

    void parse_ports(const std::unordered_map<std::string, std::unordered_map<std::string, std::string>> &ports_config);
    void parse_cells(const std::unordered_map<std::string, std::unordered_map<std::string, std::string>> &cells_config);
    void parse_json() {}

    int get_index(int x, int y, int z) const;

    // For wiring, we fill the cells along a path—this is a stub.
    void fill_path(const std::vector<Coord3>& path, int n, bool my_grid = true);

    int fill_volume_compute_overlap(Cell* cell);
    bool fill_volume(Cell* cell);

    // -----------------------------------------------------------------
    // Updated get_kind: returns a CellInfo using A* solver conventions.
    CellInfo get_kind(int x, int y, int z);

    bool volume_empty(int width, int depth, int height, int x, int y, int z);

    Position new_cell_pos(const std::string &kind, int trials = 1000, bool every_pos = false);

    std::unordered_map<std::string, std::vector<std::pair<std::string, std::string>>> get_cell_ports(const std::string &cell_name);
};

Layout::Layout(const std::array<int, 3>& vol, int seed)
    : volume(vol), seed(seed), overlap(0), a_star(nullptr)
{
    rng.seed(seed);
    grid.resize(volume[0] * volume[1] * volume[2], "");
}

Layout::~Layout() {
    for (auto cell : generated_cells)
        delete cell;
    if (a_star)
        delete a_star;
}

int Layout::get_index(int x, int y, int z) const {
    return z * volume[0] * volume[1] + y * volume[0] + x;
}

// Here we implement get_kind to return a CellInfo value.
// For simplicity, if the grid cell is not empty we treat it as Block, otherwise Air.
CellInfo Layout::get_kind(int x, int y, int z) {
    CellInfo ci;
    if (x < 0 || x >= volume[0] || y < 0 || y >= volume[1] || z < 0 || z >= volume[2]) {
        ci.type = CellType::Block;
        return ci;
    }
    int idx = get_index(x, y, z);
    if (grid[idx].empty())
        ci.type = CellType::Air;
    else
        ci.type = CellType::Block;
    return ci;
}

// new_cell_pos: choose a random valid position (using a 1–cell buffer and cell dimensions from CELL_KINDS).
Position Layout::new_cell_pos(const std::string &kind) {
    if (CELL_KINDS.find(kind) == CELL_KINDS.end()) {
        if (verbose)
            std::cout << "Kind " << kind << " not found in CELL_KINDS!" << std::endl;
        std::exit(1);
    }
    CellKind dims = CELL_KINDS[kind];
    int ext_w = dims.x + 6;
    int ext_d = dims.y + 6;
    int ext_h = dims.z + 6;
    std::uniform_int_distribution<int> dist_x(0, volume[0] - ext_w);
    std::uniform_int_distribution<int> dist_y(0, volume[1] - ext_d);
    std::uniform_int_distribution<int> dist_z(0, volume[2] - ext_h);
    int x = dist_x(rng);
    int y = dist_y(rng);
    int z = dist_z(rng);
    Position pos;
    pos.x = x + 3;
    pos.y = y + 3;
    pos.z = z + 3;
    pos.w = dims.x;
    pos.d = dims.y;
    pos.h = dims.z;
    return pos;
}

bool Layout::fill_volume(Cell* cell) {
    int x = cell->pos.x, y = cell->pos.y, z = cell->pos.z;
    int width = cell->pos.w, depth = cell->pos.d, height = cell->pos.h;
    if (x + width >= volume[0] || y + depth >= volume[1] || z + height >= volume[2])
        return false;
    for (int dx = 0; dx < width; dx++) {
        for (int dy = 0; dy < depth; dy++) {
            for (int dz = 0; dz < height; dz++) {
                int idx = get_index(x + dx, y + dy, z + dz);
                if (!grid[idx].empty())
                    return false;
            }
        }
    }
    for (int dx = 0; dx < width; dx++) {
        for (int dy = 0; dy < depth; dy++) {
            for (int dz = 0; dz < height; dz++) {
                int idx = get_index(x + dx, y + dy, z + dz);
                grid[idx] = cell->name;
            }
        }
    }
    return true;
}

int Layout::fill_volume_compute_overlap(Cell* cell) {
    int overlapCount = 0;
    int x = cell->pos.x, y = cell->pos.y, z = cell->pos.z;
    int width = cell->pos.w, depth = cell->pos.d, height = cell->pos.h;
    for (int dx = 0; dx < width; dx++) {
        for (int dy = 0; dy < depth; dy++) {
            for (int dz = 0; dz < height; dz++) {
                int idx = get_index(x + dx, y + dy, z + dz);
                if (!grid[idx].empty())
                    overlapCount++;
            }
        }
    }
    for (int dx = 0; dx < width; dx++) {
        for (int dy = 0; dy < depth; dy++) {
            for (int dz = 0; dz < height; dz++) {
                int idx = get_index(x + dx, y + dy, z + dz);
                grid[idx] = cell->name;
            }
        }
    }
    return overlapCount;
}

std::unordered_map<std::string, std::vector<std::pair<std::string, std::string>>>
Layout::get_cell_ports(const std::string &cell_name) {
    // For simplicity, we return an empty map here.
    return {};
}

void Layout::parse_ports(const std::unordered_map<std::string, std::unordered_map<std::string, std::string>> &ports_config) {
    ports = ports_config;
}

void Layout::parse_cells(const std::unordered_map<std::string, std::unordered_map<std::string, std::string>> &cells_config) {
    cells = cells_config;
}

void Layout::rebuild_grid() {
    overlap = 0;
    grid.assign(volume[0] * volume[1] * volume[2], "");
    for (auto cell : generated_cells) {
        overlap += fill_volume_compute_overlap(cell);
    }
}

void Layout::fill_path(const std::vector<Coord3>& path, int n, bool my_grid) {
    int count = 0;
    for (const auto &p : path) {
        if (n != -1 && count >= n)
            break;
        int idx = get_index(p[0], p[1], p[2]);
        grid[idx] = "wire";
        count++;
    }
}

std::vector<std::pair<Coord3, Coord3>> Layout::_generate_output_paths() {
    std::vector<std::pair<Coord3, Coord3>> output_paths;
    for (auto cell : generated_cells) {
        auto it = CELL_KINDS.find(cell->kind);
        if (it == CELL_KINDS.end())
            continue;
        CellKind ck = it->second;
        if (ck.outputs.empty())
            continue;
        for (auto &out_pair : ck.outputs) {
            Coord3 start = { cell->pos.x + out_pair.second[0],
                             cell->pos.y + out_pair.second[1],
                             cell->pos.z + out_pair.second[2] };
            Coord3 end = start;
            end[0] += 5; // dummy offset for demonstration.
            output_paths.push_back({ start, end });
        }
    }
    return output_paths;
}

std::pair<std::vector<std::vector<Coord3>>, std::vector<std::pair<Coord3, Coord3>>>
Layout::_attempt_paths(const std::vector<std::pair<Coord3, Coord3>> &paths, int limit, bool verbose_flag) {
    std::vector<std::vector<Coord3>> successful;
    std::vector<std::pair<Coord3, Coord3>> failed;
    for (size_t i = 0; i < paths.size(); i++) {
        try {
            auto path = a_star->solve(paths[i].first, paths[i].second, limit);
            if (!path.empty()) {
                successful.push_back(path);
                fill_path(path, -1);
            } else {
                failed.push_back(paths[i]);
            }
        } catch (...) {
            failed.push_back(paths[i]);
        }
    }
    return { successful, failed };
}

std::vector<std::vector<Coord3>>
Layout::_solve_generated_paths(const std::vector<std::pair<Coord3, Coord3>> &output_paths, bool large_log) {
    int max_iterations = 5;
    int iteration = 0;
    std::vector<std::vector<Coord3>> result;
    auto currentPaths = output_paths;
    while (iteration < max_iterations && !currentPaths.empty()) {
        auto savedGrid = grid;
        gen_wires.clear();
        a_star->resetCaches();
        auto attempt = _attempt_paths(currentPaths, 100000, false);
        if (!attempt.second.empty()) {
            for (auto &p : attempt.first) {
                result.push_back(p);
                fill_path(p, 2);
            }
            currentPaths = attempt.second;
            grid = savedGrid;
        } else {
            for (auto &p : attempt.first)
                result.push_back(p);
            break;
        }
        iteration++;
    }
    gen_wires = result;
    return result;
}

std::pair<int,int> Layout::generate_paths() {
    auto output_paths = _generate_output_paths();
    auto routed_paths = _solve_generated_paths(output_paths);
    if (routed_paths.empty())
        return { -1, -1 };
    int path_len = 0;
    int num_paths = 0;
    for (auto &p : routed_paths) {
        if (!p.empty()) {
            num_paths++;
            path_len += p.size();
        }
    }
    return { path_len, num_paths };
}

std::string Layout::serialize() {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    char dateStr[100];
    std::strftime(dateStr, sizeof(dateStr), "%Y%m%d_%H%M%S", std::gmtime(&t));
    std::string filename = "layout_" + std::to_string(volume[0]) + "x" +
                           std::to_string(volume[1]) + "x" +
                           std::to_string(volume[2]) + "_" + dateStr + ".bin";
    std::ofstream file(filename, std::ios::binary);
    file.close();
    return filename;
}

Layout* Layout::deserialize_layout(const std::string &filename) {
    // Stub.
    return new Layout();
}

// ---------------------------------------------------------------------
// Here we update set_astar to use the AStarSolver defined in astar_solver.cpp.
// We create a bounding box (using Coord3) and pass a lambda that calls our get_kind.
void Layout::set_astar() {
    Coord3 min = { 0, 0, 0 };
    Coord3 max = { volume[0] - 1, volume[1] - 1, volume[2] - 1 };
    if (a_star)
        delete a_star;
    a_star = new AStarSolver({ min, max }, [this](int x, int y, int z) -> CellInfo {
        return this->get_kind(x, y, z);
    });
}

// For demonstration, we simply place ports first and then cells.
void Layout::generate_start_layout() {
    // Place ports.
    for (auto &p : ports) {
        std::string cell_name = p.first;
        std::string direction = p.second.at("direction");
        std::string kind = (direction == "input") ? "PORT_IN" : "PORT_OUT";
        Cell* cell = new Cell(this, cell_name, kind);
        generated_cells.push_back(cell);
        cells_lut[cell_name] = cell;
        if (!fill_volume(cell)) {
            if (verbose)
                std::cout << "Could not fill cell " << cell->name << std::endl;
        }
    }
    // Place the rest of the cells.
    for (auto &c : cells) {
        std::string cell_name = c.first;
        std::string cell_type = c.second.at("type");
        Cell* cell = new Cell(this, cell_name, cell_type);
        generated_cells.push_back(cell);
        cells_lut[cell_name] = cell;
        if (!fill_volume(cell)) {
            if (verbose)
                std::cout << "Could not fill cell " << cell->name << std::endl;
        }
    }
}

// ---------------------------------------------------------------------
// LayoutGenome: a container for a layout’s cell configuration, with mutation/crossover.
// ---------------------------------------------------------------------

class LayoutGenome {
public:
    std::vector<Cell*> cell_list;
    std::unordered_map<std::string, Cell*> cell_lut;
    std::array<int, 3> volume;
    double fitness;
    int seed;
    std::mt19937 rng;

    LayoutGenome(const std::vector<Cell*>& cell_list,
                 const std::unordered_map<std::string, Cell*>& cell_lut,
                 const std::array<int, 3>& volume, int seed = 420)
        : cell_list(cell_list), cell_lut(cell_lut), volume(volume), seed(seed) {
        rng.seed(seed);
    }
    void mutate_movement(double mutation_rate = 0.1) {
        for (auto cell : cell_list) {
            std::uniform_real_distribution<double> dist(0.0, 1.0);
            if (dist(rng) < mutation_rate) {
                Position new_pos = cell->pos;
                std::uniform_int_distribution<int> d(-1, 1);
                new_pos.x += d(rng);
                new_pos.y += d(rng);
                new_pos.z += d(rng);
                if (new_pos.x < 0 || new_pos.y < 0 || new_pos.z < 0 ||
                    new_pos.x + new_pos.w > volume[0] ||
                    new_pos.y + new_pos.d > volume[1] ||
                    new_pos.z + new_pos.h > volume[2])
                    continue;
                cell->pos = new_pos;
            }
        }
    }
    Position random_cell_pos(Cell* cell) {
        if (CELL_KINDS.find(cell->kind) == CELL_KINDS.end()) {
            if (verbose)
                std::cout << "Kind " << cell->kind << " not found in CELL_KINDS!" << std::endl;
            std::exit(1);
        }
        CellKind dims = CELL_KINDS[cell->kind];
        int ext_w = dims.x + 6;
        int ext_d = dims.y + 6;
        int ext_h = dims.z + 6;
        std::uniform_int_distribution<int> dist_x(0, volume[0] - ext_w);
        std::uniform_int_distribution<int> dist_y(0, volume[1] - ext_d);
        std::uniform_int_distribution<int> dist_z(0, volume[2] - ext_h);
        int x = dist_x(rng), y = dist_y(rng), z = dist_z(rng);
        Position pos;
        pos.x = x + 3;
        pos.y = y + 3;
        pos.z = z + 3;
        pos.w = dims.x;
        pos.d = dims.y;
        pos.h = dims.z;
        return pos;
    }
    void mutate_rand_pos(double mutation_rate = 0.05) {
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        if (cell_list.empty())
            return;
        std::uniform_int_distribution<size_t> dist_index(0, cell_list.size()-1);
        Cell* cell = cell_list[dist_index(rng)];
        if (dist(rng) < mutation_rate) {
            Position new_pos = random_cell_pos(cell);
            if (new_pos.w != -1)
                cell->pos = new_pos;
        }
    }
    static LayoutGenome* crossbreed(const LayoutGenome* genome1, const LayoutGenome* genome2) {
        if (genome1->cell_list.size() != genome2->cell_list.size()) {
            if (verbose)
                std::cout << "Genomes have different number of cells, cannot crossbreed" << std::endl;
            std::exit(1);
        }
        std::vector<Cell*> new_cell_list;
        std::unordered_map<std::string, Cell*> new_cell_lut;
        std::mt19937 local_rng(genome1->seed);
        for (size_t i = 0; i < genome1->cell_list.size(); i++) {
            Cell* cell1 = genome1->cell_list[i];
            Cell* cell2 = genome2->cell_list[i];
            std::uniform_real_distribution<double> dist(0.0, 1.0);
            Cell* chosen = (dist(local_rng) < 0.5) ? cell1 : cell2;
            Cell* new_cell = new Cell(*chosen);
            new_cell_list.push_back(new_cell);
            new_cell_lut[new_cell->name] = new_cell;
        }
        return new LayoutGenome(new_cell_list, new_cell_lut, genome1->volume, genome1->seed);
    }
    Layout* to_layout() {
        Layout* new_layout = new Layout(volume, seed);
        new_layout->generated_cells.clear();
        new_layout->cells_lut.clear();
        for (auto cell : cell_list) {
            Cell* new_cell = new Cell(new_layout, cell->name, cell->kind, cell->ports, &cell->pos);
            new_layout->generated_cells.push_back(new_cell);
            new_layout->cells_lut[new_cell->name] = new_cell;
        }
        new_layout->rebuild_grid();
        new_layout->set_astar();
        return new_layout;
    }
    double compute_fitness() {
        int res_length = -1;
        int max_len = -1;
        Layout* layout_tmp = to_layout();
        if (layout_tmp->overlap > 0) {
            double fit = -100 * layout_tmp->overlap;
            delete layout_tmp;
            return fit;
        }
        auto [plen, num_paths] = layout_tmp->generate_paths();
        res_length = plen;
        max_len = num_paths;
        delete layout_tmp;
        if (res_length == -1)
            return -1e6;
        return -res_length;
    }
};

LayoutGenome* Layout::to_genome(int seed) {
    std::vector<Cell*> cells_copy;
    std::unordered_map<std::string, Cell*> cell_lut_copy;
    for (auto cell : generated_cells) {
        Cell* new_cell = new Cell(*cell);
        cells_copy.push_back(new_cell);
        cell_lut_copy[new_cell->name] = new_cell;
    }
    return new LayoutGenome(cells_copy, cell_lut_copy, volume, seed);
}

// ---------------------------------------------------------------------
// Main function
// ---------------------------------------------------------------------

int main() {
    Layout layout({50, 50, 50}, 420);

    // Stub configuration for ports and cells.
    layout.ports = {
        {"port1", {{"direction", "input"}, {"bit", "100"}}},
        {"port2", {{"direction", "output"}, {"bit", "200"}}}
    };
    layout.cells = {
        {"cell1", {{"type", "NAND"}}},
        {"cell2", {{"type", "NOR"}}}
    };

    layout.parse_ports(layout.ports);
    layout.parse_cells(layout.cells);
    layout.generate_start_layout();
    layout.rebuild_grid();

    auto [path_len, num_paths] = layout.generate_paths();
    std::cout << "Total wiring length: " << path_len
              << ", Number of paths: " << num_paths << std::endl;

    LayoutGenome* genome = layout.to_genome(420);
    double fitness = genome->compute_fitness();
    std::cout << "Genome fitness: " << fitness << std::endl;

    delete genome;
    return 0;
}
