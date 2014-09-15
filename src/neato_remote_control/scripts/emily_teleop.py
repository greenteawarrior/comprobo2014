#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String

def getch():
    """ Return the next character typed on the keyboard """
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def process_laser_scan(msg):
    """
    Helper function for wall_detector() - obtains the laser scan 
    data and then returns the running average of the distance from
    the neato to the walls. 
    """
    # keep track of variables
    # calculate the average
    # print the average


def wall_detector():
    """
    Uses proportional control to make sure that the neato
    stays one meter away from the walls.
    """
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('scan', LaserScan, process_laser_scan) 
    # process_laser_scan is the callback function

    rospy.init_node('emily_wall_detector', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        # logic for determining msg here

        pub.publish(msg)
        r.sleep()

def keyboard_control():
    """ 
    Simple neato remote controller. 

    Keyboard controls:
    a : go forward
    k : pause
    r : rotate

    """
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('keyboard_control', anonymous=True)
   
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        ch = getch()
        if ch == 'a': # go forward
            msg = Twist(Vector3(1, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        elif ch == 'k': # pause
            msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        elif ch == 'r': # rotate
            msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 1))
        elif ch == 'q':
            break        
        pub.publish(msg)
        r.sleep()
        
if __name__ == '__main__':
    try:
        keyboard_control() # keyboard controller
        wall_detector() # uses proportional control to stay 1 meter away from the walls
    except rospy.ROSInterruptException: pass