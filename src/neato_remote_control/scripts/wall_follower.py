#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan

class Wall_Follower():
    """
    Uses proportional control to make sure that the neato
    stays one meter away from the walls.
    """

    def __init__(self):
        rospy.init_node('wall_follower', anonymous=True)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('scan', LaserScan, self.process_laser_scan)

        # for understanding laser scan things
        self.front_lidar_data = []
        self.front_distance = []
        self.front_distance_to_wall = -1

        self.side_lidar_data = []
        self.side_distance = []
        self.side_distance_to_wall = -1

        self.parallel = False
        self.radius = 1

        # for proportional control things
        self.Kp = .5


    def take_running_average(self, angles, ranges, front_or_side):
        if front_or_side == 'front':
            for i in angles: # getting laser data at the 358th, 359th, 0th, 1st, and 2nd degrees 
                if ranges[i] != 0 and ranges[i] < 5:
                    self.front_lidar_data.append(ranges[i])
            if len(self.front_lidar_data): # take the running average
                self.front_distance_to_wall = sum(self.front_lidar_data)/float(len(self.front_lidar_data))
                self.radius = self.front_distance_to_wall / 2
            else:
                self.front_distance_to_wall = -1.0
            # self.front_lidar_data = []

        elif front_or_side == 'side':
            for i in angles: # getting laser data at the 358th, 359th, 0th, 1st, and 2nd degrees 
                if ranges[i] != 0 and ranges[i] < 5:
                    self.side_lidar_data.append(ranges[i])
            if len(self.side_lidar_data): # take the running average
                self.side_distance_to_wall = sum(self.side_lidar_data)/float(len(self.side_lidar_data))
                if 0.9 < self.side_distance_to_wall < 1.1:
                    self.parallel = True
                else:
                    self.parallel = False
                # self.side_lidar_data = []
            else:
                self.side_distance_to_wall = -1.0

        print "front_distance_to_wall" + str(self.front_distance_to_wall)
        print "side_distance_to_wall" + str(self.side_distance_to_wall)


    def process_laser_scan(self, msg):
        """
        0degree version of the callback function for wall_follow() - 
        obtains the laser scan data and then returns the 
        running average of the distance from the neato to the walls. 

        The "_linear" refers to how the neato will only check the dista
        """

        front_angles = [358, 359, 0, 1, 2]
        # un/comment the side_angles depending on which side of wall you want to test for now
        side_angles = [88, 89, 90, 91, 92] 
        # side_angles = [268, 269, 270, 271, 272]

        self.take_running_average(front_angles, msg.ranges, 'front')
        self.take_running_average(side_angles, msg.ranges, 'side')

    def wall_follow(self):
        print "Wall follower, I choose you!"
        r = rospy.Rate(10) # 10hz
        turning = True

        while not rospy.is_shutdown():

            # msg = Twist()

            if not self.parallel:
                msg = Twist(linear=Vector3(x=(self.front_distance_to_wall-1)*self.Kp), angular=Vector3(z=-(self.front_distance_to_wall-1)*self.Kp/self.radius))
            elif self.parallel:
                msg = Twist(linear=Vector3(x=.2))

            self.pub.publish(msg)
            r.sleep()

    def run(self):
        self.wall_follow()

if __name__ == '__main__':
    try:
        controller = Wall_Follower()
        controller.run()
    except rospy.ROSInterruptException: pass