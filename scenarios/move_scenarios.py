import data_structures
import rospy, os
from util import plot_energy, Annotate, parser
from std_msgs.msg import String


def move_scenario():
    """
    One apad is added and it should move somewhere.
    Figure out how times and speed will be calculated. 1 second
    of our time is 30 minutes in simulation, but speed of apad is 0.5 meters per second.
    How will we calculate everything??
    """
    deltat = 1
    topology = parser(filename=os.path.join("..", "init_files", "init_topo_exmple.txt"))
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
