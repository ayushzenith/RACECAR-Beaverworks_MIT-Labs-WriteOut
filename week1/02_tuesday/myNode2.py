#!/usr/bin/env python

__author__ = "Ayush Zenith and Beaverworks"
__credits__ = ["Ayush Zenith", "Beaverworks"]
__date__ = '2019-07-11'

'''
This is my_node2 implemented using a class.
'''

import rospy
from std_msgs.msg import String

#<initialize the node here>
rospy.init_node("my_node2")

class MyNodeClass(object):
    def __init__(self):
    	#<create your blah subscriber here>
        self.my_pub=rospy.Subscriber("blah",String,self.callback)
    	print("This is ~classy~ node2 standing by!")

    #<create your blah callback function here>
    def callback(self, msg):
        print "", msg.data

###########################################

if __name__ == '__main__':
    MyNode = MyNodeClass()
    rospy.spin()
