from parser import Layout, Cell, CELL_KINDS, LayoutGenome
import json
import os
import sys
import pickle
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
#  - Number paths

class GeneticHandler:
    def __init__(self, json_data, size, random=True, seed=420):
        seed = None if random == True else seed 
        self.seed = seed
        self.size = size
        population = []
        self.epocs = 0
        for _ in range(size):
            _layout = Layout(cells={},ports={},seed=seed)
            _layout.parse_json(json_data)
            _layout.generate_start_layout();
            population.append(_layout.to_genome(seed=seed))
        self.population = population
        self.fitness = [None for _ in range(size)]

    def run_epoc(self,save=False):
        self.epocs += 1
        self.evaluate(save=save)
        self.cull()
        self.crossbreed()
        self.mutate()


    # sorts populatoin by fitness, also stores fitness into self.fitness
    # assumes fitness is same size as population
    def evaluate(self,save=False):
        for idx, gene in enumerate(self.population):
            print(f" On gene {idx}/{self.size}")
            self.fitness[idx] = gene.compute_fitness()

        pair = list(zip(self.fitness, range(len(self.fitness)), self.population))
        pair.sort(key=lambda x: -x[0])
        sorted_fitness, _, sorted_population = zip(*pair)
        
        self.fitness = list(sorted_fitness)
        self.population = list(sorted_population)
        if save:
            save = False
            self.save(f"GeneticHandler_{str(self.seed)}_{self.size}_{self.epocs}.pkl")

    # randomizes list.
    # params are percentages of what you want
    def cull(self, upper=0.1, random=0.1, bottom=0.01):
        upper = (len(self.population) * upper) // 1
        random = (len(self.population) * random) // 1
        bottom = (len(self.population) * bottom) // 1

        if upper > 0:
            upper = self.population[:upper]
        else:
            upper = []
        if bottom > 0:
            bottom = self.population[-bottom:]
        else:
            bottom = []

        if random > 0:
            random = random.sample(self.population,random)
        else:
            random = []
        result = upper + random + bottom
        self.population = random.shuffle(result)
        self.fitness = [None for _ in range(len(result))]

    def mutate(self):
        for idx, gene in enumerate(self.population):
            gene.mutate_movement();
            gene.mutate_rand_pos();
            self.population[idx] = gene

    # expects culling prior to breeding
    def crossbreed(self, size=None):
        if size is None:
            size = self.size

        while self.population < size:
            # pick two parents at random
            p1, p2 = random.sample(self.population,2)
            self.population.append(LayoutGenome.crossbreed(p1,p2))

        self.fitness = [None for _ in range(self.size)]

    def save(self, filepath):
        with open(filepath, "wb") as f:
            pickle.dump(self, f)
        print(f"GeneticHandler saved to {filepath}")

    @classmethod
    def load(cls, filepath):
        with open(filepath, "rb") as f:
            instance = pickle.load(f)
        print(f"GeneticHandler loaded from {filepath}")
        return instance


if __name__ == '__main__':
    flag = True
    if flag:
        handler = GeneticHandler.load("GeneticHandler_None_100_0.pkl")
        for f in handler.fitness:
            print(f"{f}")

    if flag == False:
        json_file = "../../synth.json"
        json_data = {}
        try:
            with open(json_file, 'r') as f:
                json_data = json.load(f)
        except FileNotFoundError:
            print(f"Error: File not found at '{json_file}'")
            sys.exit(1);
        except json.JSONDecodeError:
            print(f"Error: Invalid JSON format in '{json_file}'")
            sys.exit(1)

        handler = GeneticHandler(json_data,100)
        handler.evaluate(save=True)
