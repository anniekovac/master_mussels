import os
import rospy
import data_structures
import topology


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


# TODO : think of the smarter init structure
# TODO : add efficiency of charging in some way (percentage?)
# TODO : changes in code neccessary for one aPad charging multiple aMussels
# TODO : add a scenario where one aPad charges multiple aMussels
if __name__ == '__main__':
    system = parser()
    system.plot_topology()
    frequency = 1.0/system.deltat
    rospy.init_node('system', anonymous=True)
    r = rospy.Rate(frequency)  # 1 Hz
    while not rospy.is_shutdown():
        for apad in system.pads:
            if apad.working_mode:
                print("Energy before update: {}".format(apad.energy))
                apad.update_energy(deltat=system.deltat, working_mode=apad.working_mode[0])
                print("Energy after update: {}".format(apad.energy))
        r.sleep()
