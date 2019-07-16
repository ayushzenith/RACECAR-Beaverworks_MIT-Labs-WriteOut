#!/usr/bin/env python
'''
This is my_node1 implemented in a very simple way.
'''
import rospy
from std_msgs.msg import String

rospy.init_node("my_node1")

my_pub=rospy.Publisher("blah",String,queue_size=1)
start_time=rospy.get_time()
print("This is node1 standing by!")

while not rospy.is_shutdown():
    if abs(rospy.get_time()-start_time-1)<0.001:
        start_time=rospy.get_time()
        my_pub.publish("The Time Is: "+str(start_time))
