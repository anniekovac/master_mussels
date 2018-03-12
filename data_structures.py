import rospy

mussel_working_modes = ["sleep", "normal", "camera", "motors", "charging"]
apad_working_modes = ["motors", "charging_self", "charging_mussels", "sleep"]

# how much energy mussel loses during certain activity (percentage/half an hour)
mussel_mode_percentages = {"sleep": -1,
                           "normal": -3,
                           "camera": -5,
                           "motors": -10,
                           "charging": 1}

# how much energy mussel loses during certain activity
apad_mode_percentages = {"sleep": -1,
                         "charging_mussels": -2,  # for each mussel
                         "charging_self": 1,
                         "motors": -7}

charge_till = 90


class EnergyBase(object):
    def __init__(self, energy):
        self.energy = energy

    def update_energy(self):
        """
        This function should calculate change of energy
        during certain deltaT and certain working state of
        aMussels and aPad.
        """
        pass

    def charge(self, charge_duration, charge_till_percentage = False):
        """
        This function should simulate how
        object will be charged.
        :param charge_duration: int (for how many seconds mussel will be charged)
        :param charge_till_percentage: boolean (if you want to charge the mussel till certain percentage)
        """
        start = rospy.get_rostime().secs
        if charge_till:
            while self.energy < charge_till:
                self.energy += 0.00001
            charge_duration = rospy.get_rostime().secs - start
            return charge_duration 
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
        print("aPad energy before discharge: {}".format(self.energy))
        start = rospy.get_rostime().secs
        finish_time = start + discharge_duration
        while rospy.get_rostime().secs < finish_time:
            self.energy -= 0.00001
        print("aPad energy after discharge: {}".format(self.energy))


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
        charge_dur = mussel_instance.charge(5, charge_till_percentage=True)
        if charge_dur:
            self.losing_energy(charge_dur)
        print("Energy of mussel after charging: {}".format(mussel_instance.energy))


if __name__ == '__main__':
    my_mussel = aMussel(50)
    my_pad = aPad(10)
    rospy.init_node('system', anonymous=True)
    time = rospy.Time().now()
    my_pad.add_mussel_to_charge(my_mussel)

