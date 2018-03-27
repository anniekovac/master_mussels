import topology
import data_structures
import os
import matplotlib.pyplot as plt
import numpy


class Annotate(object):
    def __init__(self, text, coordinates):
        self.text = text
        self.coordinates = coordinates


def parser():
    """
    Function created for parsing init file.
    In init file are written agents and their
    initial states of energy, as well as deltat (this
    is discrete time : how often energies are updated).

    :return: topology.Topology() class instance
    """
    with open("init.txt", "r") as init:
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


def plot_energy(energy_list, time, title="Energy of certain agent", labels=[], annotate=[]):
    """
    PLotting energy of an agent. For now, one second in "our time" is
    represented as 30 minutes of simulated time.
    :param energy_list: array (can be list of lists, if you want to plot energy of multiple agents)
    :param time: array
    """
    # turning seconds into half an hour
    new_time = []
    first_second = time[0]
    for sec in time:
        new_time.append((sec-first_second) * 0.5)
    time = new_time

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

        #if annotate:
        for ann_instance in annotate:
            ann_instance.coordinates = (ann_instance.coordinates[0] - first_second, ann_instance.coordinates[1])
            text_coor = ann_instance.coordinates
            print(ann_instance.coordinates)
            print(text_coor)
            plt.annotate(ann_instance.text, xy=ann_instance.coordinates) #, xytext=text_coor),
                            #arrowprops=dict(facecolor='black', shrink=0.05),)
        plt.title(title)
        plt.xlabel("Time (hours)")
        plt.ylabel("Energy (percent)")

    plt.legend()
    plt.grid()
    plt.show()
