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
        self.turn_start_time = 0

        # for proportional control things
        self.Kp = .2

    def process_laser_scan(self, msg):
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
        turning = True
        
        while not rospy.is_shutdown():
            if self.distance_to_wall == -1: # the action hasn't started yet ; i.e. no followable walls detected 
                msg = Twist()
            elif self.distance_to_wall <= 1.01 and self.distance_to_wall >= 0.9:
                self.turn_start_time = rospy.get_time()
                while turning: 
                    print 'wait what'
                    current_time = rospy.get_time()
                    msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 1))
                    if current_time - self.turn_start_time > math.pi/2:
                        turning = False
            elif turning == False:
                msg = Twist(Vector3(1, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
            else:
                msg = Twist(linear=Vector3(x=(self.distance_to_wall-1)*self.Kp))
            self.pub.publish(msg)
            r.sleep()

    # def ninety_degree_turn(self):
    #     print "Hey neato, turn 90 degrees!"
    #     r = rospy.Rate(10) # 10hz

    #     self.turn_start_time = rospy.get_time()
    #     while not rospy.is_shutdown():
    #         current_time = rospy.get_time()
    #         if current_time - self.turn_start_time >= math.pi/2:
    #             msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
    #         else:
    #             msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 1))
    #         self.pub.publish(msg)
    #         r.sleep()

    def run(self):
        self.wall_follow()
        # self.ninety_degree_turn()

if __name__ == '__main__':
    try:
        controller = Wall_Follower()
        controller.run()
    except rospy.ROSInterruptException: pass