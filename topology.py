from data_structures import aPad, aMussel
import random, numpy


class Topology(object):
	"""
	This class will define number of aMussels and
	aPads on the surface, their coordinated, and it will
	be able to plot them.
	"""
	def __init__(self):
		self.mussels = []
		self.pads = []

	def plot_topology(self):
		"""
		This function is used for 2D plotting
		topology. Mussels are marked with one colour, 
		Pads are marked with another.
		"""
		import matplotlib.pyplot as plt
		mussels_x = [item.coordinates[0] for item in self.mussels]
		mussels_y = [item.coordinates[1] for item in self.mussels]
		area = numpy.pi * (15 * 1) ** 2  # 0 to 15 point radii
		N = len(self.mussels)
		plt.scatter(mussels_x, mussels_y, s=area, alpha=0.5, label="Mussels")

		pads_x = [item.coordinates[0] for item in self.pads]
		pads_y = [item.coordinates[1] for item in self.pads]
		area = numpy.pi * (15 * 1) ** 2  # 0 to 15 point radii
		N = len(self.pads)
		plt.scatter(pads_x, pads_y, s=area, alpha=0.5, label="Pads")
		plt.legend()

		plt.grid()
		plt.show()

if __name__ == '__main__':

	topology = Topology()
	mussels = [aMussel(50) for i in range(10)]
	for mussel in mussels:
		mussel.coordinates = (random.randint(0, 10), random.randint(0, 10))
	topology.mussels = mussels
	pads = [aPad(80) for i in range(2)]
	for pad in pads:
		pad.coordinates = (random.randint(0, 10), random.randint(0, 10))
	topology.pads = pads
	topology.plot_topology()