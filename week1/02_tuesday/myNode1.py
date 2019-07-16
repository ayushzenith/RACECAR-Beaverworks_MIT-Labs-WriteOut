#!/usr/bin/env python

__author__ = "Ayush Zenith and Beaverworks"
__credits__ = ["Ayush Zenith", "Beaverworks"]
__date__ = '2019-07-11'

'''
This is my_node1 implemented using a class.
'''

import rospy
from std_msgs.msg import String

#<initialize the node here>
rospy.init_node("my_node1")

class MyNodeClass(object):
    def __init__(self):
        #<create your blah publisher here>
    	self.my_pub=rospy.Publisher("blah",String,queue_size=1)
    	self.start_time=rospy.get_time()
    	print("This is ~classy~ node1 standing by!")

###########################################

if __name__ == '__main__':
    MyNode = MyNodeClass()
    while not rospy.is_shutdown():
    	if abs(rospy.get_time()-MyNode.start_time-1)<0.001:
            MyNode.start_time=rospy.get_time()
    	    #<publish the time here to blah>
            MyNode.my_pub.publish("The Time Is: "+str(MyNode.start_time))
