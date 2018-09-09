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
        :param points: list of floats (points)
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
    return 1/(1 + salesman.total_distance(guess_instance.points))


def evaluation(system):
    """
    This function should calculate fitness for every gene.
    :param system:
    :return:
    """
    pass


def mutation(gene, mutation_probability=0.05):
    """
    Mutation of one guess.
    :param gene: list
    :param mutation_probability: float (probability for a mutation to happen)
    :return: Guess instance (new mutated guess)
    """
    for index, num in enumerate(gene):
        if numpy.random.uniform(low=0, high=1) <= mutation_probability:

            index1 = random.randint(0, len(gene)-1)
            index2 = random.randint(0, len(gene)-1)
            a = gene[index1]
            b = gene[index2]
            gene[index1] = b
            gene[index2] = a

    return gene


def position_based_crossover(guess1, guess2, mut_prob):
    """
    This function implements position based crossover.
    :param guess1: Guess instance
    :param guess2: Guess instance
    :param mut_prob: float
    :return: Guess instance
    """
    mother = guess1
    father = guess2
    # mother = guess1.gene
    # father = guess2.gene
    geneLen = len(mother)
    child = [None] * geneLen
    for i, gene in enumerate(child):
        if random.random(0, 1) <= 0.5:
            child[i] = mother[i]
        else:
            child[i] = father[i]
    # mutChild = mutation(child)
    # guessChild = Guess(mutChild)
    # guessChild.points = [points_dict[i] for i in mutChild]
    # return guessChild
    

def printPositionBased():
    gene1 = [1, 2, 3, 4, 5, 6, 7, 8]
    gene2 = [3, 8, 7, 2, 6, 5, 1, 4]
    position_based_crossover(gene1, gene2, 0.01)
    print("mother: {}".format(gene1))
    print("father: {}".format(gene2))
    print("child: ")


def cycle_crossover(guess1, guess2, mut_prob):
    """
    This function implements cycle crossover.
    :param guess1: Guess instance
    :param guess2: Guess instance
    :param mut_prob: float
    :return: Guess instance
    """
    mother = guess1.gene
    father = guess2.gene
    # mother = guess1
    # father = guess2
    geneLen = len(mother)
    child = [None] * geneLen

    while (None in child):
        try:
            iMother = child.index(None)
        except ValueError:
            break

        while (child[iMother] is None):
            child[iMother] = mother[iMother]
            iMother = father.index(mother[iMother])

        try:
            iFather = child.index(None)
        except ValueError:
            break

        while (child[iFather] is None):
            child[iFather] = father[iFather]
            iFather = mother.index(father[iFather])

    mutChild = mutation(child)
    guessChild = Guess(mutChild)
    # guessChild.fitness = get_fitness(guessChild)
    guessChild.points = [points_dict[i] for i in mutChild]
    return guessChild


def printCycle():
    m = [1, 2, 3, 4, 5, 6, 7, 8]
    f = [3, 8, 7, 2, 6, 5, 1, 4]
    c = cycle_crossover(m, f, 0.5)
    print("m: ", m)
    print("f: ", f)
    print(c)


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
    #print_points(population)
    return population


def print_points(population):
    for guess in population:
        print(guess.points)


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


def k_gen_algorithm(points, n, iterations, mut_prob, elitism=False):
    population = generate_population(points, n)
    # for item in population:
    #     print(item.gene)
    i = iterations
    while i:
        i -= 1
        new_population = []
        if elitism:
            greatest_fitness = max([item.fitness for item in population])
            new_population.append([guess for guess in population if guess.fitness == greatest_fitness][0])
        while len(new_population) <= n:
            first_parent = roulette_wheel(population)
            second_parent = roulette_wheel(population)
            child = cycle_crossover(first_parent, second_parent, mut_prob)
            fitness = get_fitness(child)
            child.fitness = fitness
            new_population.append(child)
        population = new_population
        # if i == 1000:
        #     print_points(population)
        #     return
        max_fitness = max([item.fitness for item in population])
        best = max(population, key=attrgetter('fitness'))
        # if i % 100 == 0:
        #     print(i, ": ", best.points)
    print(max_fitness)
    return best


def main(points):
    n = 25  # POPULATION SIZE
    mut_prob = 0.01  # MUTATION PROBABILITY
    number_of_iterations = 12500  # NUMBER OF ITERATION7
    global points_dict
    for i in range(0, len(points)):
        points_dict[i+1] = points[i]

    best = k_gen_algorithm(points, n, number_of_iterations, mut_prob)
    print("Best guess is: {}".format(best.gene))
    print("Best length is: {}".format(salesman.total_distance(best.points)))
    return (best, points_dict)


if __name__ == "__main__":
    #main()
    #printCycle()
    print(mutation([1, 2, 3, 4, 5], 1.0))
