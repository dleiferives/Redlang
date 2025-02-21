from parser import Layout, Cell, CELL_KINDS, LayoutGenome
import json
import os
import sys
# Layout Genome:
#  Cells Position (x,y,z)
#
#
# Changes:
#  Allow valid movement x y z as a more common mutation
#  Completely random position as a rare mutation
#  Allow crossbreeding as a rare mutation
#
# Fitness:
#  - Average path length 0.8
#  - Max path length 0.2

class GeneticHandler:

    
    def __init__(self, json_data, size, random=True, seed=420):
        seed = None if random == True else seed 
        self.seed = seed
        population = []
        for _ in range(size):
            _layout = Layout(cells={},ports={},seed=seed)
            _layout.parse_json(json_data)
            _layout.generate_start_layout();
            population.append(_layout.to_genome(seed=seed))
        self.population = population
        self.fitness = [None for _ in range(size)]

    def mutate(self):
        for idx, gene in enumerate(self.population):
            gene.mutate_movement();
            gene.mutate_rand_pos();
            self.population[idx] = gene

    def crossbreed(self):




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
    max_fitness = base_fitness
    max_fgene = population[0]
    fitness_list = []
    for idx, gene in enumerate(population):
        fitness = gene.compute_fitness();
        if(max_fitness < fitness):
            max_fgene = gene
            max_fitness = fitness
        fitness_list.append(fitness)



