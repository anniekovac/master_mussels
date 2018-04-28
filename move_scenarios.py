import data_structures
import rospy
from util import plot_energy, Annotate, parser
from std_msgs.msg import String


def move_scenario():
    """
    One simple scenario: one aPad is added
    and one aMussel. aPad has the energy of 22.
    aPad is charging, therefore, gaining energy.
    """
    deltat = 1
    topology = parser(filename="init_topo_exmple.txt")
    apad = topology.all_agents[0]
    r = rospy.Rate(1/deltat)  # 1 Hz
    while not rospy.is_shutdown():
        #apad.update_energy(deltat)
        apad.move(deltat)
        print(apad.coordinates)
        r.sleep()


if __name__ == '__main__':
    rospy.init_node('topology', anonymous=True)
    time = rospy.Time().now()
    try:
        move_scenario()
    except rospy.ROSInterruptException:
        pass
