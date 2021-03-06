import data_structures
import rospy
from util import plot_energy, Annotate, parser
from std_msgs.msg import String
from controller.msg import WorkingModes
from salesman import go_to_nearest, distance, total_distance
import evolutionary


def scenario1():
    """
    One simple scenario: one aPad is added
    and one aMussel. aPad has the energy of 22.
    aPad is charging, therefore, gaining energy.
    """
    deltat = 1
    apad = data_structures.aPad(22)
    apad.working_mode.append("charging_self")
    energy = []
    time = []
    r = rospy.Rate(1/deltat)  # 1 Hz
    while not rospy.is_shutdown():
        energy.append(apad.energy)
        time.append(rospy.get_rostime().secs)
        apad.update_energy(deltat)
        print(apad.energy)
        if len(energy) > 100:
            break
        r.sleep()
    plot_energy(energy, time, "Energy of an apad in scenario 1 - just charging", labels=["aPad"])


def scenario_mussel_charging():
    """
    There is one mussel in topology.
    """
    deltat = 1
    r = rospy.Rate(1/deltat)

    amussel = data_structures.aMussel(2)
    modes = [("charging", 60)]

    energy = []
    time = []
    for mode, mode_time in modes:
        start_mode = rospy.get_rostime().secs
        while not rospy.is_shutdown():
            if rospy.get_rostime().secs - start_mode >= mode_time:
                break
            amussel.working_mode = [mode]
            energy.append(amussel.energy)
            time.append(rospy.get_rostime().secs)
            amussel.update_energy(deltat)
            print(amussel.energy)
            r.sleep()
    plot_energy(energy, time, "Energy of amussel in scenario 2", labels=["Mussel"])


def scenario2():
    """
    There is one mussel in topology.
    """
    deltat = 1
    r = rospy.Rate(1/deltat)

    amussel = data_structures.aMussel(98)
    modes = [("camera", 15), ("sleep", 6), ("camera", 8), ("charging", 6), ("normal", 6)]

    energy = []
    time = []
    annotate = []
    for mode, mode_time in modes:
        start_mode = rospy.get_rostime().secs
        annotate.append(Annotate(mode, (start_mode, amussel.energy)))
        while not rospy.is_shutdown():
            if rospy.get_rostime().secs - start_mode >= mode_time:
                break
            amussel.working_mode = [mode]
            energy.append(amussel.energy)
            time.append(rospy.get_rostime().secs)
            amussel.update_energy(deltat)
            print("Mussel energy: {}".format(amussel.energy))
            r.sleep()
    plot_energy(energy, time, "Energy of amussel in scenario 2", labels=["Mussel"], annotate=annotate)


def scenario3():
    """
    There is one mussel and one apad in topology.
    aPad is charging aMussel, while also charging himself.
    """
    deltat = 1
    r = rospy.Rate(1/deltat)

    # initializing agents
    amussel = data_structures.aMussel(27)
    apad = data_structures.aPad(98)

    # initializing modes of work and how long they last
    mussel_modes = [("charging", 5)]
    apad_modes = [(["charging_mussels"], 5)]

    # initializing empty arrays for plot
    mussel_energy = []
    apad_energy = []
    time = []
    for apad_mode, mussel_mode in zip(apad_modes, mussel_modes):
        start_mode = rospy.get_rostime().secs
        while not rospy.is_shutdown():
            if rospy.get_rostime().secs - start_mode >= apad_mode[1]:  # if "mode time" passed - if charging is finished
                break

            # setting agents working mode
            amussel.working_mode = [mussel_mode[0]]
            apad.working_mode = apad_mode[0]

            # adding energies to curves
            mussel_energy.append(amussel.energy)
            apad_energy.append(apad.energy)
            time.append(rospy.get_rostime().secs)

            # updating energy of agents
            amussel.update_energy(deltat)
            apad.update_energy(deltat)

            r.sleep()

    # plot
    plot_energy([mussel_energy, apad_energy], time, "Energy of apad and amussel in scenario 3", labels=["Mussel", "Pad"])


def scenario4():
    """
    There is two mussels and one apad in topology.
    aPad is charging 2 aMussels.
    """
    deltat = 1
    r = rospy.Rate(1/deltat)

    # initializing agents
    amussel1 = data_structures.aMussel(27)
    amussel2 = data_structures.aMussel(55)
    apad = data_structures.aPad(98)

    # initializing modes of work and how long they last
    mussel1_modes = [("charging", 14)]
    mussel2_modes = [("charging", 14)]
    apad_modes = [(["charging_mussels"], 14)]
    apad.mussels_charging = [amussel1, amussel2]

    # initializing empty arrays for plot
    mussel1_energy = []
    mussel2_energy = []
    apad_energy = []
    time = []
    for apad_mode, mussel1_mode, mussel2_mode in zip(apad_modes, mussel1_modes, mussel2_modes):
        start_mode = rospy.get_rostime().secs
        while not rospy.is_shutdown():
            if rospy.get_rostime().secs - start_mode >= apad_mode[1]:  # if "mode time" passed - if charging is finished
                break

            # setting agents working mode
            amussel1.working_mode = [mussel1_mode[0]]
            amussel2.working_mode = [mussel2_mode[0]]
            apad.working_mode = apad_mode[0]

            # adding energies to curves
            mussel1_energy.append(amussel1.energy)
            mussel2_energy.append(amussel2.energy)
            apad_energy.append(apad.energy)
            time.append(rospy.get_rostime().secs)

            # updating energy of agents
            amussel1.update_energy(deltat)
            amussel2.update_energy(deltat)
            apad.update_energy(deltat)
            print("Mussel1 energy: {}".format(amussel1.energy))
            print("Mussel2 energy: {}".format(amussel2.energy))
            print("aPad energy: {}".format(apad.energy))
            # time check - this cannot last forever :'(

            r.sleep()

    # plot
    plot_energy([mussel1_energy, mussel2_energy, apad_energy], time, "Energy of apad and amussel in scenario 3", labels=["Mussel1", "Mussel2", "Pad"])


def publish_modes():
     pub = rospy.Publisher('working_mode', WorkingModes, queue_size=10)
     rospy.init_node('talker', anonymous=True)
     rate = rospy.Rate(10) # 10hz
     while not rospy.is_shutdown():
         for mode, time in [("sleep", 5), ("normal", 5), ("camera", 5)]:
             new_msg = WorkingModes()
             new_msg.working_mode = mode
             new_msg.working_mode_time = time
             pub.publish(new_msg)
             #listener()
             rate.sleep()

def orderPointsByPassing(system):
    points = len(system.mussels)*[None]
    for mussel in system.mussels:
        points[mussel.order_of_passing-1] = mussel.coordinates
    return points


def scenarioEvolutionary():
    system = parser(filename="evolutionary_init.txt")
    mussel_coordinates = [list(item.coordinates) for item in system.mussels]
    bestGene, points_dict = evolutionary.main(mussel_coordinates, system.pads[0].coordinates)
    switchDict = dict()
    for key, item in points_dict.items():
        switchDict[tuple(item)] = key
    # for i, mussel in enumerate(system.mussels):
    points = bestGene.points
    for i, mussel in enumerate(system.mussels):
        system.mussels[i].order_of_passing = points.index(list(mussel.coordinates))
    totalDistance = total_distance(points, system.pads[0].coordinates)
    system.plot_topology(annotate_energy=False, order_of_passing=True, title="Way of prioritizing: Evolutionary algorithm; Total Distance: {}".format(totalDistance))


def scenarioGoToNearestRandom():
    system = parser(filename="evolutionary_init.txt")
    mussel_coordinates = [list(item.coordinates) for item in system.mussels]
    #start = min(mussel_coordinates, key=lambda x: distance(list(system.pads[0].coordinates), x))
    #print("Total distance is {}".format(go_to_nearest(system, start)))
    points = go_to_nearest(system)
    totalDistance = total_distance(points)
    system.plot_topology(annotate_energy=False, order_of_passing=True, title="Way of prioritizing: Go to nearest point with random first point; Total Distance: {}".format(totalDistance))


def scenarioGoToNearest():
    system = parser(filename="evolutionary_init.txt")
    mussel_coordinates = [list(item.coordinates) for item in system.mussels]
    start = min(mussel_coordinates, key=lambda x: distance(list(system.pads[0].coordinates), x))
    points = go_to_nearest(system, start)
    totalDistance = total_distance(points)
    system.plot_topology(annotate_energy=False, order_of_passing=True, title="Way of prioritizing: Go to nearest point with nearest first point; Total Distance: {}".format(totalDistance))


def callback(data):
    print("callback")
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def listener():
    #rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("working_mode", WorkingModes, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    scenarioEvolutionary()
    #scenarioGoToNearestRandom()
    #scenarioGoToNearest()
    # rospy.init_node('topology', anonymous=True)
    # try:
    #     #publish_modes()
    #     #listener()
    #     scenario5()
    # except rospy.ROSInterruptException:
    #     pass