import data_structures
import rospy


def scenario1():
    """
    One simple scenario: one aPad is added
    and one aMussel. aMussel has the energy of
    23 and aPad has the energy of 98.
    aPad adds aMussel to charging. WIth this act,
    aPad loses energy and aMussel gains energy.
    :return:
    """
    mussel = data_structures.aMussel(23)
    apad = data_structures.aPad(98)
    apad.add_mussel_to_charge(mussel)


if __name__ == '__main__':
    rospy.init_node('arm_to_pos', anonymous=True)
    time = rospy.Time().now()
    scenario1()