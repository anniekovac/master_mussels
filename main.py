import rospy
from util import parser




# TODO : think of the smarter init structure
# TODO : add efficiency of charging in some way (percentage?)
# TODO : changes in code neccessary for one aPad charging multiple aMussels
# TODO : add a scenario where one aPad charges multiple aMussels
# TODO : create some kind of visual representation (how to distinguish when mussels
# todo : beneath water surface and on the surface, etc
if __name__ == '__main__':
    system = parser()
    # system.plot_topology()
    frequency = 1.0/system.deltat
    rospy.init_node('system', anonymous=True)
    r = rospy.Rate(frequency)  # 1 Hz
    while not rospy.is_shutdown():
        for apad in system.pads:
            if apad.working_mode:
                print("Energy before update: {}".format(apad.energy))
                apad.update_energy(deltat=system.deltat) #, working_mode=apad.working_mode[0])
                print("Energy after update: {}".format(apad.energy))
        r.sleep()
