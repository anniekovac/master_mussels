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
            agent, energy = line.split(":")
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


if __name__ == '__main__':
    system = parser()
    rospy.init_node('system', anonymous=True)
    r = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        print("da")
        r.sleep()