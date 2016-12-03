#!/usr/bin/python2
"""Utilities to use a Raspberry Pi Model B with Python.

Mostly, this defines a dictionary of pins of the Pi, which comes from
`this website <https://developer.microsoft.com/en-us/windows/iot/docs/pinmappingsrpi>`_.
Here's a pinout diagram of the Pi that comes from
`another website <http://www.jameco.com/Jameco/workshop/circuitnotes/raspberry_pi_circuit_note_fig2a.jpg>`_:

.. image:: raspberry_pi_circuit_note_fig2a.jpg

.. py:module:: RPiUtils
   :platform: Unix
   :synopsis: Utilities for a Raspberry Pi.
.. moduleauthor:: Aleksander Lidtke

"""
import RPi.GPIO

# All te usable pins.
GPIOPINS={"GPIO2":3,"GPIO3":5, # I2C SDA and SCL
	"GPIO4":7,"GPIO5":29,"GPIO6":31, # GPIO pull-up
	"GPIO14":8,"GPIO15":10, # UART TXD0 and RXT0
	"GPIO9":21,"GPIO10":19,"GPIO11":23, # SPI0 MISO, MOSI, CLK
	"GPIO8":24,"GPIO7":26, # SPI0 CE1_N, CE0_N
	"GPIO19":35,"GPIO20":38,"GPIO21":40, # SPI1 MISO, MOSI, CLK
	"GPIO16":36, # SPI1 CS0
	# GPIO pull-down follow:
	"GPIO12":32,"GPIO13":33,"GPIO17":11,"GPIO18":12,
	"GPIO22":15,"GPIO23":16,"GPIO24":18,"GPIO25":22,
	"GPIO26":37,"GPIO27":13,
	}

def _checkPin(p):
	"""Check if pin p is a valid GPIO pin on Raspberry Pi 3 Model B.

	Args:
		* p (int): which pin to check.

	Raises:
		* TypeError when pin is not int.
		* ValueError when pin is not a valid GPIO pin of RPi 3 Model B.

	"""
	if not type(p) is int:
		raise TypeError("Pin should be int, got {} instead.".format(type(p)))
	if not p in GPIOPINS.values():
		raise ValueError("Pin={} is not a valid GPIO pin on Raspberry Pi 3 Model B.".format(p))

def pinHigh(pin):
	"""Set the given pin high.

	Args:
		* pin (int): which pin to set high.

	Raises:
		* TypeError when pin is not int.
		* ValueError when pin is not a valid GPIO pin of RPi 3 Model B.

	"""
	_checkPin(pin)
	RPi.GPIO.output(pin,True)

def pinLow(pin):
	"""Set the given pin low.

	Args:
		* pin (int): which pin to set low.

	Raises:
		* TypeError when pin is not int.
		* ValueError when pin is not a valid GPIO pin of RPi 3 Model B.

	"""
	_checkPin(pin)
	RPi.GPIO.output(pin,False)
