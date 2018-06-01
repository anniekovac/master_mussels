import numpy
import random
import salesman
from operator import attrgetter


points_dict = dict()


class Guess(object):
    def __init__(self, gene):
        """
        :param genes: list of integers
        :param fitness: float
        :param MSE: float
        """
        self.gene = gene
        self.fitness = None
        self.points = None


def get_fitness(guess_instance):
    """
    Calculating fitness function for set of genes given with guess_instance.
    Fitness function for now is just total distance between
    :param guess_instance: instance of Guess class
    :return: float (fitness)
    """
    return (1/ 1 + salesman.total_distance(guess_instance.points))


def evaluation(system):
    """
    This function should calculate fitness for every gene.
    :param system:
    :return:
    """
    pass


def mutation(guess, mutation_probability=0.01):
    """
    Mutation of one guess.
    :param guess: Guess instance
    :param mutation_probability: float (probability for a mutation to happen)
    :return: Guess instance (new mutated guess)
    """
    gene = guess.gene
    for index, num in enumerate(gene):
        if numpy.random.uniform(low=0, high=1) <= mutation_probability:
            gene[index] = numpy.random.uniform(low=-4, high=4)
    return guess.gene


def crossover(guess1, guess2, mut_prob):
    """
    This crossover implements discreete recombination
    of genes.
    :param guess1: Guess class instance
    :param guess2: Guess class instance
    :return: Guess class instance
    """
    mother = guess1.gene
    father = guess2.gene
    gene_length = len(mother)

    index1 = random.randint(1, gene_length - 2)
    index2 = random.randint(1, gene_length - 2)
    if index1 > index2: index1, index2 = index2, index1

    child1 = [None] * gene_length
    child2 = [None] * gene_length

    # fill in numbers between crossover points with genes from mother and father
    child1[index1:index2] = mother[index1:index2]
    child2[index1:index2] = father[index1:index2]

    # fill in further bits with numbers from opposite parent
    for i in xrange(gene_length):
        # if this place in the gene is empty
        if child1[i] is None:
            # if that number is not already written in child
            if father[i] not in child1:
                child1[i] = father[i]

        if child2[i] is None:
            if mother[i] not in child2:
                child2[i] = mother[i]

    # fill in further bits with numbers tracked via pairing
    for i in xrange(gene_length):
        # if this place in the gene is empty
        if child1[i] is None:
            # if that number is not already written in child
            if mother[i] not in child1:
                child1[i] = mother[i]

        if child2[i] is None:
            if father[i] not in child2:
                child2[i] = father[i]

    for i in range(gene_length):
        if sorted(child1) == list(range(1, gene_length + 1)):
            break


    print(mother)
    print(father)
    print("\n")

    print(child1)
    print(child2)

    child1 = father[:index1] + mother[index1:index2] + father[index2:]
    child2 = mother[:index1] + father[index1:index2] + mother[index2:]

    child1_instance = Guess(child1)
    child1_instance.points = [points_dict[i] for i in child1]

    child2_instance = Guess(child2)
    child2_instance.points = [points_dict[i] for i in child2]



    return (child1_instance, child2_instance)


def generate_population(points, n):
    """
    Generating first population. Genes are random variations of the
    order of passing through points.
    :param n: number of guesses in population
    :return: list of Guess instances
    """
    size_of_gene = len(points)
    population = []
    for guess in range(n):
        genes = []
        genes = range(1, size_of_gene+1)
        random.shuffle(genes)
        guess_instance = Guess(genes)
        instance_points = [points_dict[i] for i in genes]
        guess_instance.points = instance_points
        fitness = get_fitness(guess_instance)
        guess_instance.fitness = fitness
        population.append(guess_instance)
    #print([item.gene for item in population])
    return population


def roulette_wheel(population):
    """
    Implemented roulette wheel selection.
    :param population: list of Guess instances
    :return: Guess instance
    """
    choices = {chromosome: chromosome.fitness for chromosome in population}
    max = sum(choices.values())
    pick = random.uniform(0, max)
    current = 0
    for key, value in choices.items():
        current += value
        if current > pick:
            return key


def k_gen_algorithm(points, n, iterations, mut_prob, mse_exit_criteria=0.01, elitism=True):
    population = generate_population(points, n)
    i = iterations
    while i:
        i -= 1
        new_population = []
        if elitism:
            greatest_fitness = max([item.fitness for item in population])
            new_population.append([guess for guess in population if guess.fitness == greatest_fitness][0])
        while len(new_population) < n:
            first_parent = roulette_wheel(population)
            second_parent = roulette_wheel(population)
            #print(first_parent.gene)
            #print(second_parent.gene)
            child = crossover(first_parent, second_parent, mut_prob)
            #print(child.gene)
            #break
            fitness = get_fitness(child)
            child.fitness = fitness
            new_population.append(child)
        population = new_population
        max_fitness = max([item.fitness for item in population])
        best = max(population, key=attrgetter('fitness'))
        if i % 100 == 0:
            print(salesman.total_distance(best.points))
            #print([item.gene for item in population])
            #print("iteration: {}, max fitness: {}".format(iterations - i, max_fitness))
            #break
    return max_fitness


def main(points):
    n = 35  # POPULATION SIZE
    mut_prob = 0.01  # MUTATION PROBABILITY
    number_of_iterations = 2500  # NUMBER OF ITERATIONS

    global points_dict
    for i in range(0, len(points)):
        points_dict[i+1] = points[i]

    best = k_gen_algorithm(points, n, number_of_iterations, mut_prob, mse_exit_criteria=0.005)
    print("Best guess is: {}".format(best.gene))


if __name__ == "__main__":
    main()
