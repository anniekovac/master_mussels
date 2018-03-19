import data_structures
import rospy
import matplotlib.pyplot as plt
import pylab
import numpy


def plot_energy(energy_list, time, title="Energy of certain agent", labels=[]):
    """
    PLotting energy of an agent. For now, one second in "our time" is
    represented as 30 minutes of simulated time.
    :param energy_list: array (can be list of lists, if you want to plot energy of multiple agents)
    :param time: array
    """
    # turning seconds into half an hour
    new_time = []
    first_second = time[0]
    for sec in time:
        new_time.append((sec-first_second) * 0.5)
    time = new_time

    if any(isinstance(el, list) for el in energy_list):  # if there are multiple agents' energy
        if not labels:  # and labels are not defined
            for idx, item in enumerate(energy_list):  # define labels by agents' indexes
                labels.append("Agent {}".format(idx))
    else:
        energy_list = [energy_list]  # if there aren't multiple agents' energy

    for idx, energy in enumerate(energy_list):
        # plotting
        plt.plot(numpy.array(time), numpy.array(energy), linewidth=2.0, label=labels[idx])
        ax = plt.subplot(111)
        ax.set_xlim(0, time[-1])
        ax.set_ylim(0, 110)
        plt.title(title)
        plt.xlabel("Time (hours)")
        plt.ylabel("Energy (percent)")

    plt.legend()
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


def scenario3():
    """
    There is one mussel and one apad in topology.
    """
    deltat = 1
    r = rospy.Rate(1/deltat)

    # initializing agents
    amussel = data_structures.aMussel(27)
    apad = data_structures.aPad(98)

    # initializing modes of work and how long they last
    mussel_modes = [("charging", 14)]
    apad_modes = [(["charging_self", "charging_mussels"], 14)]

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
            if len(time) > 10:
                break

            r.sleep()

    # plot
    plot_energy([mussel_energy, apad_energy], time, "Energy of apad and amussel in scenario 3", labels=["Mussel", "Pad"])


if __name__ == '__main__':
    rospy.init_node('arm_to_pos', anonymous=True)
    time = rospy.Time().now()
    scenario3()