import data_structures as ds


def test_too_many_mussels_charging():
	"""
	This function checks if AssertionError
	raises when too many mussels are sent to charging.
	"""
	first_mussel = ds.aMussel(50)
	my_pad = ds.aPad(10)
	my_pad.add_mussel_to_charge(first_mussel)
	second_mussel = ds.aMussel(25)
	third_mussel = ds.aMussel(95)
	fourth_mussel = ds.aMussel(100)
	my_pad.add_mussel_to_charge(second_mussel)
	my_pad.add_mussel_to_charge(third_mussel)
	my_pad.add_mussel_to_charge(fourth_mussel)
	try:
		my_pad.add_mussel_to_charge(ds.aMussel(2))
	except AssertionError:
		print("Error raised when fifth mussel sent to charging!")

if __name__ == '__main__':
	test_too_many_mussels_charging()