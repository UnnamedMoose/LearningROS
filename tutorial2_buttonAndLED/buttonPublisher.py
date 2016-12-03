#!/usr/bin/python2
"""Make and run a ROS node that reads the logic state of a pin and publishes it.

Create a ``buttonPublisherNode`` ROS node that will subscribe to ``LEDCtrlTopic``.
Messages in this topic are of type ``std_msgs.msg.Bool``.

The node exectues in a while loop at a fixed refresh rate, i.e. it stays active
until it's stopped.

.. py:module:: buttonPublisherNode
   :platform: Linux
   :synopsis: Create a buttonPublisherNode.
.. moduleauthor:: Artur Lidtke

"""

from __future__ import print_function
import rospy
import std_msgs.msg
import time
import RPi.GPIO
import RPiUtils

PIN="GPIO17" # Input button is plugged into this pin.
REFRESH_RATE = 10 # refresh rate in Hz

# decode the string representation of the pin into a workable int
inputPin = RPiUtils.GPIOPINS[PIN]
# this variable holds the boolean state of the button
pinState = False

def makeButtonPublisher():
    """ Create a publisher object to read input pin state and relay it to subscribers

    Initialises a button publisher node which publishes a Bool message to all
    subscribers. The value of the message represents the state of the input
    pin connected to a button.
    """

    # initialise this node, enabling debug mode
    rospy.init_node('buttonPublisherNode', log_level=rospy.DEBUG)
    # create a publisher object subscribed to the LED control topic
    # this will publish boolean values depending on the state of the switch
    buttonPub = rospy.Publisher('LEDCtrlTopic', std_msgs.msg.Bool, queue_size=10)

    # set the loop frequency
    rate = rospy.Rate(REFRESH_RATE)

    # initialise the GPIO protocol
    RPi.GPIO.setmode(RPi.GPIO.BOARD)
    # set the pin mode to input - there is a dedicated pull-down resistor in
    # the circuit so no need to activate the built-in one
    RPi.GPIO.setup(inputPin, RPi.GPIO.IN)#, pull_up_down = RPi.GPIO.PUD_DOWN)

    # keep iterating until the thread is alive
    while not rospy.is_shutdown():

        # check the state of the input pin
        if (RPi.GPIO.input(inputPin) == 1):
            pinState = True
        else:
            pinState = False

        # publish the state in the topic for listeners to know
        buttonPub.publish(pinState)

        # construct and publish a log entry to post in the main log topic
        logMsg = "Input pin state time {} s = {}".format(rospy.get_time(), pinState)
        # rospy.loginfo(logMsg) # this would publish every cycle
        rospy.loginfo_throttle(int(10/REFRESH_RATE), logMst) # this will send the message every 10/rate seconds

        # wait for the next function
        rate.sleep()

    # clean up the serial set up
    RPi.GPIO.cleanup()

if __name__ == '__main__':
    try:
        makeButtonPublisher()
    except rospy.ROSInterruptException as roie:
        rospy.loginfo(rospy.get_caller_id() + "buttonPublisher exitting due to {}.".format(roie))
        # clean up the serial set up in case something got changed before the interrupt
        RPi.GPIO.cleanup()
