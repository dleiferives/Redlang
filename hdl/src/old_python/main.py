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
    population = []
    for n in range(10):
        population.append(layout.to_genome(seed=n))

    base_fitness = population[0].compute_fitness()
    for n in range(100):
        for idx, gene in enumerate(population):
            gene.mutate_movement();
            gene.mutate_rand_pos();
            population[idx] = gene
    max_fitness = base_fitness
    max_fgene = population[0]
    fitness_list = []
    for idx, gene in enumerate(population):
        fitness = gene.compute_fitness();
        if(max_fitness < fitness):
            max_fgene = gene
            max_fitness = fitness
        fitness_list.append(fitness)


    print(f" base fitness: {base_fitness}")
    for idx, fitness in enumerate(fitness_list):
        print(f"Pop {str(idx)} fitness: {fitness}")

    print(f" Best fitness: {max_fitness}")

    nbt_gen = NBTGenerator(max_fgene.to_layout());
    nbt_gen.generate_file(args.json_file+"_layout_most_fit.nbt")



if __name__ ==  '__main__':
    main()
