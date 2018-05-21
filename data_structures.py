import rospy

charge_till = 90


class EnergyBase(object):
    def __init__(self, energy):
        self.energy = energy
        # please, keep in mind that self.working_modes property is a LIST!
        self.working_modes = []
        self.mode_percentages = dict()

    def move(self, deltat, speed=None, direction=None):
        """
        This function should calculate new coordinates that
        agent has after time deltat.
        Default speed defined here is also the max speed of the
        apad (0.5 meters per second = 1.8 km/h = 0.9 km/half an hour).
        Lets take 0.25 meters per second. = 0.9 km/h = 0.45 km/half an hour
        :param deltat: float
        """
        # s = v*t
        if speed == None:
            #speed = 900
            speed = 450
        deltas = speed * deltat
        self.coordinates = (self.coordinates[0] + deltas, self.coordinates[1] + deltas)


    def update_energy(self, deltat):
        """
        This function should calculate change of energy
        during certain deltaT and certain working state of
        aMussels and aPad. If energy is already 100 percent, 
        or if it would be larger than 100 percent or smaller than 0
        percent by adding change
        of energy that is calculated, energy doesn't change.
        """
        if self.energy == 100.0:
            return
        change_of_energy = 0
        for working_mode in self.working_mode:
            if working_mode == "charging_mussels":  # if aPad is charging mussels
                for mussel in self.mussels_charging:  # for each mussel that is being charged
                    change_of_energy += deltat * self.mode_percentages[working_mode]  # add change of energy
            else:
                change_of_energy += deltat * self.mode_percentages[working_mode]
        if (self.energy + change_of_energy > 100.0) or (self.energy + change_of_energy < 0):
            return
        else:
            self.energy = self.energy + change_of_energy


# total energy = 5000 mAh
# active mode = -100mAh
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
        self.mode_percentages = {"sleep": -0.3,
                                   "normal": -1,
                                   "camera": -4,
                                   "motors": -15,
                                   "charging": 2}

        self.working_mode = [working_mode]
        self.coordinates = (None, None)
        self.set_of_events = []
        self.order_of_passing = None
        if "charge" in self.working_mode:
            self.on_surface = True
        else:
            self.on_surface = None  # because we don't know if mussel is on surface yet


class aPad(EnergyBase):
    def __init__(self, energy, working_mode="charging_self"):
        super(aPad, self).__init__(energy)
        self.working_modes = ["motors", "charging_self", "charging_mussels", "sleep"]

        # how much energy aPad loses (or gains) during certain activity
        self.mode_percentages =  {"sleep": -0.3,
                                 "charging_mussels": -2,  # for each apad
                                 "charging_self": 3,
                                 "motors": -4}

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

