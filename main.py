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
            agent, energy = line.split(":")
            energy = float(energy)
            if agent == "amussel":
                agents.append(data_structures.aMussel(energy))
            elif agent == "apad":
                agents.append(data_structures.aPad(energy))
            elif agent == "deltat":
                deltat = energy

    system = topology.Topology()
    system.all_agents = agents
    system.mussels = [agent for agent in agents if isinstance(agent, data_structures.aMussel)]
    system.pads = [agent for agent in agents if isinstance(agent, data_structures.aPad)]
    system.deltat = deltat

    return system


# TODO : add coordinates in init
# TODO : think of the smarter init structure
# TODO : main function where update_energy is called
if __name__ == '__main__':
    system = parser()
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