#!/usr/bin/env python

__author__ = "Ayush Zenith, and Beaverworks"
__credits__ = ["Ayush Zenith", "Beaverworks"]
__date__ = '2019-07-11'
__version__ = "simulator potential fields"

import numpy as np
import sys, math, random, copy
import rospy, copy, time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class PotentialField:
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")

    def __init__(self):
        self.data = None
        self.cmd = AckermannDriveStamped()
        # if on simulator range 100 or else 1080
        self.ranges = 100
        #write your publishers and subscribers here; they should be the same as the wall follower's
        self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan_callback, queue_size=1)
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        #cartesian points -- to be filled (tuples)
        self.cartPoints = [None for x in range(self.ranges)]

        #[speed, angle]
        self.finalVector = [0.5, 0]

    def scan_callback(self, data):
        '''Checks LIDAR data'''
        self.data = data.ranges
        self.drive_callback()

    def drive_callback(self):
        '''Publishes drive commands'''
        #make sure to publish cmd here
        self.finalVector = self.calcFinalVector()

        #sets speed and driving angle
        self.cmd.drive.speed = self.finalVector[0]
        self.cmd.drive.steering_angle = self.finalVector[1]

        #publishes the command
        self.drive_pub.publish(self.cmd)

    def convertPoints(self):
        '''Convert all current LIDAR data to cartesian coordinates'''
        for number in range(self.ranges):
            distance = self.data[number]
            degree = (270/self.ranges) * number
            self.cartPoints[number]=(distance*math.cos(math.radians(degree)),distance*math.sin(math.radians(degree)))
            return self.cartPoints

    def calcFinalVector(self):
        '''Calculate the final driving speed and angle'''
        oppVectorLength = [None for x in range(self.ranges)]
        oppVectorDegree = [None for x in range(self.ranges)]
        oppVectorPoint = [None for x in range(self.ranges)]
        x = self.convertPoints()
        average = 0
        max = -1
        maxIndex = 0
        min = self.data[0]
        minIndex = 0
        for number in range(self.ranges):
            average = average + self.data[number]
            if self.data[number] > max:
                max = self.data[number]
                maxIndex = number
            if self.data[number] < min:
                min = self.data[number]
                minIndex = number
        print 1
        realAverage = average/self.ranges

        for number in range(self.ranges):
            oppVectorLength[number] = max - self.data[number]

        oppVectorLength[maxIndex] = min
        oppVectorLength[minIndex] = max

        for number in range(self.ranges):
            oppVectorDegree[number] = 180 - (270/self.ranges) * number
            if oppVectorDegree[number] > 270 or oppVectorDegree[number] < 0:
                oppVectorDegree[number] = (270/self.ranges) * maxIndex

        for number in range(self.ranges):
            oppVectorPoint[number] = (math.cos(math.radians(oppVectorDegree[number])) * oppVectorLength[number], math.sin(math.radians(oppVectorDegree[number])) * oppVectorLength[number])
        xTotal = 0
        yTotal = 0
        for number in range(self.ranges):
            xTotal = oppVectorPoint[number][0] + xTotal
            yTotal = oppVectorPoint[number][1] + yTotal

        finalVector = (xTotal/self.ranges,yTotal/self.ranges)
        finalVectorLength = math.sqrt(finalVector[1]*finalVector[1] + finalVector[0]*finalVector[0])
        speed = 0.5
        print 2
        if finalVectorLength > average:
            speed = 10
        if finalVectorLength < average:
            speed = 5
        print 4
        angle = 0
        if finalVector[1]>0 and finalVector[0]>0:
            angle=-abs(math.atan2(finalVector[1],finalVector[0]))
        elif finalVector[1]>0 and finalVector[0]<0:
            angle=abs(math.atan2(finalVector[1],finalVector[0]))
        elif finalVector[1]<0 and finalVector[0]<0:
            angle=abs(math.atan2(finalVector[1],finalVector[0]))
        elif finalVector[1]<0 and finalVector[0]>0:
            angle=-abs(math.atan2(finalVector[1],finalVector[0]))
        else:
            angle=math.atan2(finalVector[1],finalVector[0])
        print speed, angle
        return(speed,angle)


if __name__ == "__main__":
    rospy.init_node('potential_field')
    potential_field = PotentialField()
    rospy.spin()
