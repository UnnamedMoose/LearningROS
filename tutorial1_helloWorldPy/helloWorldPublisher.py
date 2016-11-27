#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import datetime

# get initial time, maybe?
t0 = datetime.datetime.now()

def talker():
    pub = rospy.Publisher('infoTopic', String, queue_size=10)
    rospy.init_node('publisherNode')#, anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world at time {} s".format(rospy.get_time())
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        
        # dump time to log
        with open("logfile","a") as f:
            f.write("{},{}\n".format(t0, datetime.datetime.now()))
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
