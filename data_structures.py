mussel_working_modes = ["sleep", "normal", "camera", "motors", "charging"]
apad_working_modes = ["motors", "charging_self", "charging_mussels", "sleep"]


class EnergyBase(object):
	def __init__(self, energy):
		self.energy = energy

	def charge(self, start_energy):
		"""
		This function should simulate how
		object will be charged.
		:param start_energy: int (percentage of objects energy)
		:return: None
		"""
		pass

class aMussel(EnergyBase):
	def __init__(self, energy, working_mode=["sleep"]):
		"""
		Working mode has to be a list because there could be multiple working
		modes in every moment.
		:param energy: int (percentage of energy that mussels has in the moment) 
		:param working_mode: string (one of 5 modes of working described above)
		"""
		super(aMussel, self).__init__(energy)
		self.working_mode = [working_mode]


class aPad(EnergyBase):
	def __init__(self, energy, working_mode=["charging"]):
		super(aPad, self).__init__(energy)
		self.mussels_charging = []
		self.working_mode = working_mode


if __name__ == '__main__':
	mussels = []
	for i in range(5):
		mussels.append(aMussel(50))

	my_pad = aPad(10)
