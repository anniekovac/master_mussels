import data_structures
import rospy
import matplotlib.pyplot as plt
import pylab
import numpy


def plot_energy(energy, time, title="Energy of certain agent"):
    """
    PLotting energy of an agent.
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
    energy = []
    time = []
    r = rospy.Rate(1/deltat)  # 1 Hz
    while not rospy.is_shutdown():
        energy.append(apad.energy)
        time.append(rospy.get_rostime().secs)
        apad.update_energy(deltat, working_mode="charging_self")
        print(apad.energy)
        if len(energy) > 100:
            break
        r.sleep()
    plot_energy(energy, time, "Energy of an apad in scenario 1 - just charging")


if __name__ == '__main__':
    rospy.init_node('arm_to_pos', anonymous=True)
    time = rospy.Time().now()
    scenario1()