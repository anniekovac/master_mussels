import data_structures
import rospy
import matplotlib.pyplot as plt
import pylab
import numpy


def plot_energy(energy, time, title="Energy of certain agent"):
    """
    PLotting energy of an agent. For now, one second in "our time" is
    represented as 30 minutes of simulated time.
    :param energy: array
    :param time: array
    """
    # turning seconds into half an hour
    new_time = []
    first_second = time[0]
    for sec in time:
        new_time.append((sec-first_second) * 0.5)
    time = new_time

    # plotting
    plt.plot(numpy.array(time), numpy.array(energy), linewidth=2.0)
    ax = plt.subplot(111)
    ax.set_xlim(0, time[-1])
    ax.set_ylim(0, 110)
    plt.title(title)
    plt.xlabel("Time (hours)")
    plt.ylabel("Energy (percent)")
    plt.grid()
    plt.show()


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
    plot_energy(energy, time, "Energy of an apad in scenario 1 - just charging")


def scenario2():
    """
    There is one mussel in topology.
    """
    deltat = 1
    r = rospy.Rate(1/deltat)

    amussel = data_structures.aMussel(98)
    modes = [("camera", 7), ("sleep", 2), ("charging", 6), ("camera", 8), ("normal", 6)]

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
            if len(energy) > 100:
                break
            r.sleep()
    plot_energy(energy, time, "Energy of amussel in scenario 2")


if __name__ == '__main__':
    rospy.init_node('arm_to_pos', anonymous=True)
    time = rospy.Time().now()
    scenario2()