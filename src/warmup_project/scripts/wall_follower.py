#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan 

class Wall_Follower():
    """
    Travels in a path parallel to a wall.
    """

    def __init__(self):
        rospy.init_node('wall_follower', anonymous=True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('scan', LaserScan, self.process_laser_scan)

        # FSM decisions
        self.frontFlag = False
        self.parallelFlag = False

        # lidar things
        self.side_lidar_measurements_a = []
        self.side_lidar_measurements_b = []

        self.side_distance_a = 0
        self.side_distance_b = 0

        self.radius = 1

        self.error = -1

        # proportional control things
        self.Kp = .5


    def process_laser_scan(self, msg):

        # comparing the average distance values for region a vs region b
        self.side_lidar_data_a = msg.ranges[40:51]
        self.side_lidar_data_b = msg.ranges[130:141]

        # "reinitialize" these variables for a new calculation for the averages
        self.side_usable_lidar_data_a = []
        self.side_usable_lidar_data_b = []

        # data filtering for side a :)
        for i in range(len(self.side_lidar_data_a)):
            if self.side_lidar_data_a[i] != 0 and self.side_lidar_data_a[i] < 5:
                self.side_usable_lidar_data_a.append(self.side_lidar_data_a[i])                    
        self.side_distance_a = sum(self.side_usable_lidar_data_a)/float(len(self.side_usable_lidar_data_a))

        # data filtering for side b :)
        for i in range(len(self.side_lidar_data_b)):
            if self.side_lidar_data_b[i] != 0 and self.side_lidar_data_b[i] < 5:
                self.side_usable_lidar_data_b.append(self.side_lidar_data_b[i])                    
        self.side_distance_b = sum(self.side_usable_lidar_data_b)/float(len(self.side_usable_lidar_data_b))

        if len(self.side_lidar_data_b) and len(self.side_lidar_data_a):
            # error, for proportional control things
            self.error = self.side_distance_b - self.side_distance_a
        else:
            self.error = None


    def run(self):
        print "Wall follower, I choose you!"
        r = rospy.Rate(5) # 10hz

        while not rospy.is_shutdown():
            if self.error != None:

                print "1 - abs(self.error)" + str(1 - abs(self.error))

                # # needs to turn
                # msg = Twist(angular=Vector3(z=-(self.error)*self.Kp/self.radius))

                # currently parallel
                if 1 - abs(self.error) > 0: 
                    msg = Twist(linear=Vector3(x=.3),angular=Vector3(z=-(self.error)*self.Kp/self.radius))

                    # option with proportional control on the linear part as well
                    msg = Twist(linear=Vector3(x=.3*(1-abs(self.error))),angular=Vector3(z=-(self.error)*self.Kp/self.radius))
                else: 
                    msg = Twist(angular=Vector3(z=-(self.error)*self.Kp/self.radius))

                print "self.side_distance_a " + str(self.side_distance_a)
                print "self.side_distance_b " + str(self.side_distance_b)
                print

            self.pub.publish(msg)
            r.sleep()        




    # attempt at FSM pseudocode for wall follower
    #     while going:
    #         ## Q1 : did you get 1m away front-wise?
    #         if not self.frontFlag:
    #             # move so you become 1m away
    #             # in 6helper function: once you're 1m away, self.frontFlag = True
    #             # at the moment, this false->true switch only happens once

    #         ## Q2: are you parallel? 
    #         # check your parallel-ness and set the flag appropriately
    #         if self.parallelFlag:
    #             # go forwards, it's good it's good
    #         if not self.parallelFlag:
    #             # determine necessary turn
    #             # turn it

if __name__ == '__main__':
    try:
        controller = Wall_Follower()
        controller.run()
    except rospy.ROSInterruptException: pass