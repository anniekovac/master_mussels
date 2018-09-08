from data_structures import aPad, aMussel
import random, numpy, os
import util
import time


class Topology(object):
    """
    This class will define number of aMussels and
    aPads on the surface, their coordinated, and it will
    be able to plot them.
    """

    def __init__(self):
        self.all_agents = []
        self.mussels = []
        self.pads = []
        self.deltat = None

    def plot_topology(self, save=False, annotate_energy=True, order_of_passing=False):
        """
        This function is used for 2D plotting
        topology. Mussels are marked with one colour,
        Pads are marked with another.
        :param save: boolean (if you want to save a figure - saving with name of this second so it's unique)
        :param annotate_energy: boolean (if you want to write energy levels of agents on plots)
        """
        import matplotlib.pyplot as plt
        self.mussels.sort(key=lambda x: x.order_of_passing) #, reverse=True)
        mussels_x = [item.coordinates[0] for item in self.mussels]
        mussels_y = [item.coordinates[1] for item in self.mussels]
        area = numpy.pi * (15 * 1) ** 2  # 0 to 15 point radii

        fig, ax = plt.subplots()
        ax.scatter(mussels_x, mussels_y, s=area, alpha=0.5, c="r", label="Mussels")
        if order_of_passing:
            ax.plot(mussels_x, mussels_y)

        pads_x = [item.coordinates[0] for item in self.pads]
        pads_y = [item.coordinates[1] for item in self.pads]
        area = numpy.pi * (15 * 1) ** 2
        ax.scatter(pads_x, pads_y, s=area, alpha=0.5, c="b", label="Pads")
        plt.legend()

        for (i, mussel) in enumerate(self.mussels):
            if annotate_energy:
                ax.annotate(str(mussel.energy), (mussels_x[i] + 0.05, mussels_y[i] + 0.05))
            if order_of_passing:
                ax.annotate(str(mussel.order_of_passing), (mussels_x[i] - 0.08, mussels_y[i] - 0.08))

        for (i, apad) in enumerate(self.pads):
            ax.annotate(str(apad.energy), (pads_x[i] + 0.05, pads_y[i] + 0.05))

        plt.grid()
        if save:
            plt.savefig(str(time.time()).replace(".", "")+".jpg")
        else:
            plt.show()


if __name__ == '__main__':
    topology = util.parser(filename=os.path.join(os.getcwd(), "init_files", "evolutionary_init.txt"))
    #topology.plot_topology(order_of_passing=True)
