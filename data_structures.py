import rospy

charge_till = 90


class EnergyBase(object):
    def __init__(self, energy):
        self.energy = energy
        # please, keep in mind that self.working_modes property is a LIST!
        self.working_modes = []
        self.mode_percentages = dict()

    def update_energy(self, deltat):
        """
        This function should calculate change of energy
        during certain deltaT and certain working state of
        aMussels and aPad.
        """
        if self.energy == 100.0:
            return
        for working_mode in self.working_mode:
            self.energy = self.energy + deltat * self.mode_percentages[working_mode]


class aMussel(EnergyBase):
    def __init__(self, energy, working_mode=None):
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
        self.working_modes = ["sleep", "normal", "camera", "motors", "charging"]

        # how much energy mussel loses (or gains) during certain activity (percentage/half an hour)
        self.mode_percentages = {"sleep": -1,
                                   "normal": -3,
                                   "camera": -5,
                                   "motors": -10,
                                   "charging": 1}

        self.working_mode = [working_mode]
        self.coordinates = (None, None)
        self.set_of_events = []
        if "charge" in self.working_mode:
            self.on_surface = True
        else:
            self.on_surface = None  # because we don't know if mussel is on surface yet


class aPad(EnergyBase):
    def __init__(self, energy, working_mode="charging_self"):
        super(aPad, self).__init__(energy)
        self.working_modes = ["motors", "charging_self", "charging_mussels", "sleep"]

        # how much energy aPad loses (or gains) during certain activity
        self.mode_percentages =  {"sleep": -1,
                                 "charging_mussels": -2,  # for each mussel
                                 "charging_self": 1,
                                 "motors": -7}

        self.mussels_charging = []
        self.working_mode = [working_mode]
        self.coordinates = (None, None)

    def add_mussel_to_charge(self, mussel_instance):
        """
        Function that adds mussel_instance
        :param mussel_instance:
        :return:
        """
        print("Energy of mussel before charging: {}".format(mussel_instance.energy))
        self.mussels_charging.append(mussel_instance)
        self.working_mode.append("charging_mussel")
        print("Energy of mussel after charging: {}".format(mussel_instance.energy))


if __name__ == '__main__':
    my_mussel = aMussel(50)
    my_pad = aPad(10)
    rospy.init_node('system', anonymous=True)
    time = rospy.Time().now()
    my_pad.add_mussel_to_charge(my_mussel)

