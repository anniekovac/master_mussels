import simpy
import time
import random
import numpy

mussel_working_modes = ["sleep", "normal", "camera", "motors", "charging"]
apad_working_modes = ["motors", "charging_self", "charging_mussels", "sleep"]

env = simpy.rt.RealtimeEnvironment(factor=1)



class EnergyBase(object):
	def __init__(self, energy):
		self.energy = energy

	def charge(self, charge_duration):
		"""
		This function should simulate how
		object will be charged.
		:param start_energy: int (percentage of objects energy)
		:return: None
		"""
		start = time.perf_counter()
		yield env.timeout(charge_duration)
		end = time.perf_counter()
		while charge_duration:
			charge_duration -= 1
			self.energy += 1
		print('Time of charging: {}'.format(end - start))

	def losing_energy(self, discharge_duration):
		"""
		This function should simulate how an object
		losses energy (for example, aPad is losing energy
		by charging mussels, but also by moving)
		:param start_energy: 
		:return: 
		"""
		start = time.perf_counter()
		yield env.timeout(discharge_duration)
		end = time.perf_counter()
		while discharge_duration:
			discharge_duration -= 1
			self.energy -= 1
		print('Time of discharging: {}'.format(end - start))


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
		self.coordinates = (None, None)


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
		proc = env.process(mussel_instance.charge(5))
		env.run(until=proc)
		print("Energy of mussel after charging: {}".format(mussel_instance.energy))

if __name__ == '__main__':
	my_mussel = aMussel(50)
	my_pad = aPad(10)
	my_pad.add_mussel_to_charge(my_mussel)
