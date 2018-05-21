import topology
import data_structures
import os
import matplotlib.pyplot as plt
import numpy


"""
trenutno ne mogu importati msg koji sam stvorila - otkriti zasto je to tako.
znaci treba:
- moci publishati na taj message
- napraviti da se cita s tog message
- sastaviti "plan rada" musule
- pokusati simulirati taj plan rada
- napraviti grafove za taj plan rada - kretanje musule
                                     - promjena energije musule
                                     - promjena energije vise musula + lopoca
                                     - gif koji prikazuje kretanje musula
"""

class Annotate(object):
    def __init__(self, text, coordinates):
        """
        Simple class used for annotating text. 
        @text : str (working mode which starts in this time)
        @coordinates : tuple of floats (time, energy) - where the text is placed
        """
        self.text = text
        self.coordinates = coordinates


def parser(filename=None):
    """
    Function created for parsing init file.
    In init file are written agents and their
    initial states of energy, as well as deltat (this
    is discrete time : how often energies are updated).

    :param: filename : str (name of the init file)
    :return: topology.Topology() class instance
    """
    if filename is None:
        filename = os.path.join(os.getcwd(), "..", "init_files", "init.txt")
    with open(filename, "r") as init:
        agents = []
        for line in init:
            line.rstrip(os.linesep)
            if "deltat" in line:
                deltat = float(line.split(":")[-1])
            else:
                agent, data = line.split(":")
                energy, x, y = data.split(",")
                energy = float(energy)
                x = float(x)
                y = float(y)
                if agent == "amussel":
                    new_mussel = data_structures.aMussel(energy)
                    new_mussel.coordinates = (x, y)
                    agents.append(new_mussel)
                elif agent == "apad":
                    new_apad = data_structures.aPad(energy)
                    new_apad.coordinates = (x, y)
                    agents.append(new_apad)

    system = topology.Topology()
    system.all_agents = agents
    system.mussels = [agent for agent in agents if isinstance(agent, data_structures.aMussel)]
    system.pads = [agent for agent in agents if isinstance(agent, data_structures.aPad)]
    system.deltat = deltat

    return system


def convert_time(time):
    """
    Converting time: every second of "our time"
    is represented as 30 minutes of simulated time.
    @time : array (list)
    :return: array (list)
    """
    new_time = []
    first_second = time[0]
    for sec in time:
        new_time.append((sec-first_second) * 0.5)
    return new_time   


def plot_coordinates(xs, ys, title="How certain agent moved"):
    """
    This function should plot coordinates of movement of an agent.

    :param xs: list of floats (x coordinates of an agent)
    :param ys:  list of floats (y coordinates of an agent)
    :param title: string
    """
    fig, ax = plt.subplots()
    ax.scatter(numpy.array(xs), numpy.array(ys), alpha=0.5, c="b")
    plt.grid()
    plt.xlabel("X coordinates")
    plt.ylabel("Y coordinates")
    plt.show()


def plot_energy(energy_list, time, title="Energy of certain agent", labels=[], annotate=[]):
    """
    PLotting energy of an agent. For now, one second in "our time" is
    represented as 30 minutes of simulated time.
    :param energy_list: array (can be list of lists, if you want to plot energy of multiple agents)
    :param time: array
    """
    # turning seconds into half an hour
    time = convert_time(time)

    if any(isinstance(el, list) for el in energy_list):  # if there are multiple agents' energy
        if not labels:  # and labels are not defined
            for idx, item in enumerate(energy_list):  # define labels by agents' indexes
                labels.append("Agent {}".format(idx))
    else:
        energy_list = [energy_list]  # if there aren't multiple agents' energy

    for idx, energy in enumerate(energy_list):
        # plotting
        plt.plot(numpy.array(time), numpy.array(energy), linewidth=2.0, label=labels[idx])
        ax = plt.subplot(111)
        ax.set_xlim(0, time[-1])
        #ax.set_ylim(min(energy), max(energy)+0.5)
        ax.set_ylim(0, 100)

        if annotate:  # if annotate argument exists in function calling
            for ann_instance in annotate:  # for every instance of annotate
                # ann_instance includes:
                # ann_instance.coordinates = (time, energy)
                # ann_instance.text = working_mode which begins in time
                ann_instance.coordinates = ((convert_time([ann_instance.coordinates[0]][0]) - first_second)*0.5, ann_instance.coordinates[1])  # converting seconds
                plt.annotate(ann_instance.text, xy=ann_instance.coordinates)

        plt.title(title)
        plt.xlabel("Time (hours)")
        plt.ylabel("Energy (percent)")

    plt.legend()
    plt.grid()
    plt.show()
