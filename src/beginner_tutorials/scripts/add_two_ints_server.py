#!/usr/bin/env python

#http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29

from beginner_tutorials.srv import *
import rospy

def handle_add_two_ints(req):
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    # declare the node 
    rospy.init_node('add_two_ints_server')
    # declare the service
    # all requests are passed to the handle_add_two_ints_function
    # handle_add_two_ints is called with instances of AddTwoIntsRequest 
    # and returns instance sof AddTwoIntsResponse
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    
    print "Ready to add two ints."

    # spin keeps your code from exiting until the service is shut down
    rospy.spin()

# ask about rospy.Service
# where is AddTwoInts defined? or is the name of the object that we want the other
# scripts to refer to this to?
# ... explanation of the procedure here

if __name__ == "__main__":
    add_two_ints_server()