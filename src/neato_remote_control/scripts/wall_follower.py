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
        self.distance_to_wall = -1
        self.valid_measurements = []

        # for proportional control things
        self.Kp = .2


    def process_laser_scan(self, msg):
        """
        Callback function for wall_follow() - 
        determines where the neato should turn to become parallel with the wall.
        """

        # initialize useful things
        lidar_data = []

        # find the angle in which the neato is 1m away from the wall
        degree_to_wall = 0
        for i in range(len(msg.ranges)):
            print 'degree to wall %d' % degree_to_wall
            current_distance_at_angle = msg.ranges[degree_to_wall]
            print 'current_distance_at_angle %d' % current_distance_at_angle

            if current_distance_at_angle == 0.0 and msg.ranges[i] > 0:
                degree_to_wall = i
            elif current_distance_at_angle > 0.0 and msg.ranges[i] < current_distance_at_angle:
                degree_to_wall = i

        print degree_to_wall

        # rotate accordingly
        # how much does the neato need to rotate before it's parallel with the wall?

    def process_laser_scan_linear(self, msg):
        """
        0degree version of the callback function for wall_follow() - 
        obtains the laser scan data and then returns the 
        running average of the distance from the neato to the walls. 

        The "_linear" refers to how the neato will only check the dista
        """

        lidar_data = []
        for i in [358, 359, 0, 1, 2]: # getting laser data at the 358th, 359th, 0th, 1st, and 2nd degrees 
            if msg.ranges[i] != 0 and msg.ranges[i] < 5:
                lidar_data.append(msg.ranges[i])
        if len(lidar_data): # take the running average
            self.distance_to_wall = sum(lidar_data)/float(len(lidar_data))
        else:
            self.distance_to_wall = -1.0
        print "according to lasers, the neato is %f meters away from the wall" % self.distance_to_wall

    def wall_follow(self):
        print "Wall follower, I choose you!"
        r = rospy.Rate(10) # 10hz

        # parallel state variable

        while not rospy.is_shutdown():
            if self.distance_to_wall == -1: # the action hasn't started yet
                msg = Twist()
            else:
                msg = Twist(linear=Vector3(x=(self.distance_to_wall-1)*self.Kp))
            self.pub.publish(msg)
            r.sleep()

    def run(self):
        self.wall_follow()

if __name__ == '__main__':
    try:
        controller = Wall_Follower()
        controller.run()
    except rospy.ROSInterruptException: pass