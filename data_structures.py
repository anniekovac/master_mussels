import rospy

mussel_working_modes = ["sleep", "normal", "camera", "motors", "charging"]
apad_working_modes = ["motors", "charging_self", "charging_mussels", "sleep"]

mussel_mode_percentages = dict()
apad_mode_percentages = dict()

charge_till = 90


class EnergyBase(object):
    def __init__(self, energy):
        self.energy = energy

    def charge(self, charge_duration, charge_till_percentage = False):
        """
        This function should simulate how
        object will be charged.
        :param start_energy: int (percentage of objects energy)
        :return: None
        """
        start = rospy.get_rostime().secs
        if charge_till:
            while self.energy < charge_till:
                self.energy += 0.00001
        else:
            finish_time = start + charge_duration
            while rospy.get_rostime().secs < finish_time:
                self.energy += 0.00001

    def losing_energy(self, discharge_duration):
        """
        This function should simulate how an object
        losses energy (for example, aPad is losing energy
        by charging mussels, but also by moving)
        :param discharge_duration: int (duration of discharge in seconds)
        """
        start = rospy.get_rostime().secs
        finish_time = start + discharge_duration
        while rospy.get_rostime().secs < finish_time:
            self.energy -= 0.00001


class aMussel(EnergyBase):
    def __init__(self, energy, working_mode=["sleep"]):
        """
        Working mode has to be a list because there could be multiple working
        modes in every moment.
        :param energy: int (percentage of energy that mussels has in the moment)
        :param working_mode: string (one of 5 modes of working described above)
        :param coordinates: tuple (x, y coordinate of a mussel)
        :param set_of_events: list (list of events that should happend in a row ["sleep", "charge", ...])
        :param on_surface: boolean (if mussel is on surface, True, else False)
        """
        super(aMussel, self).__init__(energy)
        self.working_mode = [working_mode]
        self.coordinates = (None, None)
        self.set_of_events = []
        if "charge" in self.working_mode:
            self.on_surface = True
        else:
            self.on_surface = None  # because we don't know if mussel is on surface yet


class aPad(EnergyBase):
    def __init__(self, energy, working_mode=["charging"]):
        super(aPad, self).__init__(energy)
        self.mussels_charging = []
        self.working_mode = working_mode
        self.coordinates = (None, None)

    def add_mussel_to_charge(self, mussel_instance):
        """
        Function that adds mussel_instance
        :param mussel_instance:
        :return:
        """
        print("Energy of mussel before charging: {}".format(mussel_instance.energy))
        self.mussels_charging.append(mussel_instance)
        mussel_instance.charge(5, charge_till_percentage=True)
        print("Energy of mussel after charging: {}".format(mussel_instance.energy))


if __name__ == '__main__':
    my_mussel = aMussel(50)
    my_pad = aPad(10)
    rospy.init_node('arm_to_pos', anonymous=True)
    time = rospy.Time().now()
    my_pad.add_mussel_to_charge(my_mussel)

