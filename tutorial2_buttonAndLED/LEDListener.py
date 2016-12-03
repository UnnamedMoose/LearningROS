#!/usr/bin/python2
"""Make and spin a ROS node that listens to LED on/off commands.

Create a ``LEDListenerNode`` ROS node that will listen to ``LEDCtrlTopic``.
Messages in this topic of type ``std_msgs.msg.Bool`` will be interpretted
as LED being turned on or off.

The node is spun after creation, i.e. it stays active until it's stopped.

.. py:module:: LEDListenerNode
   :platform: Unix
   :synopsis: Create a LEDListenerNode.
.. moduleauthor:: Aleksander Lidtke

"""
from __future__ import print_function
from RPiUtils import GPIOPINS
import RPiUtils
import rospy, RPi.GPIO, time, std_msgs.msg

PIN="GPIO21" # LED is attached to this pin.

def LEDCallback(msg):
	"""Decide whether to turn the LED on or off based on a ROS message.

	If ``msg.data`` is ``True``, the LED will be turned on. If ``False``, the
	LED will be turned off.

	Args:
		* msg (bool): used to decide what to do with the LED.

	Raises:
		* TypeError when ``msg`` is not std_msgs.msg.Bool.

	"""
	rospy.loginfo(rospy.get_caller_id() + "LEDListener heard {} of type {}.".format(
		msg.data,type(msg.data)))
	if not type(msg.data) is bool:
		raise TypeError("Data type not understood. Type of LED control is "
						"message {}.".format(type(msg.data)))

	if msg.data:
		RPiUtils.pinHigh(GPIOPINS[PIN])
	else:
		RPiUtils.pinLow(GPIOPINS[PIN])

def makeLEDListener(pin="GPIO21"):
	"""Make a ROS node that listens to LED on/off commands.

	Create a ``LEDListenerNode`` ROS node that will listen to ``LEDCtrlTopic``.
	Messages in this topic of type ``std_msgs.msg.Bool`` will be used to turn
	the LED on or off.

	Args:
		* pin (str): Raspberry Pi GPIO pin name, to which the LED is connected.

	"""
	# First start the GPIO, then handle commands.
	RPi.GPIO.setmode(RPi.GPIO.BOARD) # Start the GPIO.
	RPi.GPIO.setup(GPIOPINS[pin],RPi.GPIO.OUT) # Prepare our pin.
	RPiUtils.pinLow(GPIOPINS[pin]) # Start with the LED off.

	# Make the node and subscribe to the correct topic.
	rospy.init_node('LEDListenerNode')
	rospy.Subscriber('LEDCtrlTopic',std_msgs.msg.Bool,LEDCallback)

	try: # Keep the listener going until the node is stopped.
		rospy.spin()
	except rospy.ROSInterruptException as roie:
		RPiUtils.pinLow(GPIOPINS[pin]) # Don't end with the LED on.
		RPi.GPIO.cleanup()
		rospy.loginfo(rospy.get_caller_id() + "LEDListener exitting due to {}.".format(roie))
	else:
		RPiUtils.pinLow(GPIOPINS[pin]) # Don't end with the LED on.
		RPi.GPIO.cleanup()
		rospy.loginfo(rospy.get_caller_id() + "LEDListener: clean exit.")

if __name__ == '__main__':
	makeLEDListener(PIN)
