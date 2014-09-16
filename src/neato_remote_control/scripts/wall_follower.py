#!/usr/bin/env python

import rospy

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


    def process_laser_scan(self, msg):
        """
        Callback function for wall_follow() - obtains the laser scan 
        data and then returns the running average of the distance from
        the neato to the walls. 
        """

        # from wall_section2.py
        valid_measurements = []
        for i in range(5):
            if msg.ranges[i] != 0 and msg.ranges[i] < 7:
                valid_measurements.append(msg.ranges[i])
        if len(valid_measurements):
            self.distance_to_wall = sum(valid_measurements)/float(len(valid_measurements))
        else:
            self.distance_to_wall = -1.0
        print "according to lasers, the neato is %f meters away from the wall" % self.distance_to_wall

    def wall_follow(self):
        print "Wall follower, I choose you!"
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            if self.distance_to_wall == -1: # the action hasn't started yet
                msg = Twist()
            else:
                msg = Twist(linear=Vector3(x=(self.distance_to_wall-1)*.2))
            self.pub.publish(msg)
            r.sleep()

    def run(self):
        self.wall_follow()

if __name__ == '__main__':
    try:
        controller = Wall_Follower()
        controller.run()
    except rospy.ROSInterruptException: pass