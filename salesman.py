import doctest
from itertools import permutations
from topology import Topology
from data_structures import aMussel


def distance(point1, point2):
    """
    Returns the Euclidean distance of two points in the Cartesian Plane.
    """
    return ((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2) ** 0.5


def total_distance(points):
    """
    Returns the length of the path passing throught
    all the points in the given order.
    """
    return sum([distance(point, points[index + 1]) for index, point in enumerate(points[:-1])])


def travelling_salesman(points, start=None):
    """
    Finds the shortest route to visit all the cities by bruteforce.
    Time complexity is O(N!), so never use on long lists.

    >>> travelling_salesman([[0,0],[10,0],[6,0]])
    ([0, 0], [6, 0], [10, 0])
    >>> travelling_salesman([[0,0],[6,0],[2,3],[3,7],[0.5,9],[3,5],[9,1]])
    ([0, 0], [6, 0], [9, 1], [2, 3], [3, 5], [3, 7], [0.5, 9])
    """
    if start is None:
        start = points[0]
    return min([perm for perm in permutations(points) if perm[0] == start], key=total_distance)


def go_to_nearest(system, start=None):
    """
    :arg system: instance of Topology class
    :arg start: list (coordinates of a mussel from which algortihm is supposed to start from)
    :return path: list of list (list of coordinates, in which order algortihm goes through mussels)
    """
    points = [list(mussel.coordinates) for mussel in system.mussels]
    if start is None:
        start = points[0]
    for x in system.mussels:
        if list(x.coordinates) == start:
            x.order_of_passing = 1
            break
    must_visit = points
    path = [start]
    counter = 1
    must_visit.remove(start)
    for x in system.mussels:
        if x.coordinates == start:
            x.order_of_passing = counter
            break
    while must_visit:
        counter += 1
        nearest = min(must_visit, key=lambda x: distance(path[-1], x))
        path.append(nearest)
        for x in system.mussels:
            if list(x.coordinates) == nearest:
                x.order_of_passing = counter
                break
        must_visit.remove(nearest)
    return total_distance(path)


def main():
    doctest.testmod()
    points = [[0, 0], [1, 5.7], [2, 3], [3, 7],
              [0.5, 9], [3, 5], [9, 1], [10, 5]]
    print("""The minimum distance to visit all the following points: {}
starting at {} is {}.

The optimized algoritmh yields a path long {}.""".format(
        tuple(points),
        points[0],
        total_distance(travelling_salesman(points)),
        total_distance(optimized_travelling_salesman(points))))


if __name__ == "__main__":
    main()