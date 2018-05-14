import rospy, os, sys
from util import plot_energy, Annotate, parser, plot_coordinates
from std_msgs.msg import String
import rospkg
rospack = rospkg.RosPack()
# get the file path for rospy_tutorials
path = rospack.get_path('controller')
msg_file = "WorkingModes.msg"
path_to_msg = os.path.join(path, "msg") #, msg_file)
print(path_to_msg)
sys.path.append(path_to_msg)
import WorkingModes
import imp
foo = imp.load_source(path_to_msg, "WorkingModes.msg")
from foo import WorkingModes


def move_scenario():
    """
    One apad is added and it should move somewhere.
    Figure out how times and speed will be calculated. 1 second
    of our time is 30 minutes in simulation, but speed of apad is 0.5 meters per second.
    How will we calculate everything??
    """
    #deltat = 1  # this is 30 minutes
    #deltat = 0.5  # this is 15 minutes
    deltat = 0.25  # this is 7.5 minutes
    topology = parser(filename=os.path.join("..", "init_files", "init_topo_exmple.txt"))
    apad = topology.all_agents[0]
    r = rospy.Rate(1/deltat)  # 1 Hz
    xs = []
    ys = []
    while not rospy.is_shutdown():
        apad.move(deltat)
        x, y = apad.coordinates
        xs.append(x)
        ys.append(y)
        r.sleep()
        if len(xs) > 100:
            break
    plot_coordinates(xs, ys)


if __name__ == '__main__':
    rospy.init_node('topology', anonymous=True)
    time = rospy.Time().now()
    try:
        move_scenario()
    except rospy.ROSInterruptException:
        pass
