import data_structures
import rospy
from util import plot_energy, Annotate


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
            print(amussel.energy)
            if len(energy) > 100:
                break
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
    There is one mussel and one apad in topology.
    aPad is charging aMussel, while also charging himself.
    """
    deltat = 1
    r = rospy.Rate(1/deltat)

    # initializing agents
    amussel = data_structures.aMussel(27)
    apad = data_structures.aPad(98)

    # initializing modes of work and how long they last
    mussel_modes = [("charging", 14), ("sleep", 7), ("motors", 1), ("normal", 7)]
    apad_modes = [(["charging_self", "charging_mussels"], 14), (["charging_self"], 8), ([""])]

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

            # time check - this cannot last forever :'(

            r.sleep()

    # plot
    plot_energy([mussel_energy, apad_energy], time, "Energy of apad and amussel in scenario 3", labels=["Mussel", "Pad"])


if __name__ == '__main__':
    rospy.init_node('topology', anonymous=True)
    time = rospy.Time().now()
    scenario2()