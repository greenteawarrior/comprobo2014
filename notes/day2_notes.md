comprobo day 2
09.09.2014

$ rospack find <package name>
$ roscd <package name> # will work as long as stuff is in ros $ package path
$ rosls <package name>

# woot package completion

echo $ROS_PACKAGE_PATH

---

creating catkin packages

rospack depends1 <package name> # first-order dependencies
rospack depends <package name> # includes all dependencies (and the meta things)

publish/subscribe architecture
kaitlin's colored whiteboard analogy
there are subscribers watching the whiteboard
publisher writes something (i.e. a message, in a pre-decided format) in a certain color
subscriber(s) are always listening
and will act upon things in their color

ex. sensor is publishing position 
brain/controller thing is subscribed to that data

---

node: an executable file within a ROS package
use a ROS client library to communicate with other nodes
nodes can publish or subscribe to a topic
nodes can also provide or use a service

ROS client libraries: allow nodes in diff. programming languages to communicate
rospy = python client library

$ roscore # first thing you should run when using ROS
# then open a new terminal to do other work..
$ rosnode list #displays info (publications, subscriptions, services, contacting node, Pid...) about the ROS nodes currently running
$ rosnode info /rosout

you can reassign names from the command-line!

$ rosrun turtlesim turtlesim_node __name:my_turtle

if needed, you can clean the rosnode list with $ rosnode cleanup

ping ittttt
$ rosnode ping my_turtle

--- 

ros topics!
$ rqt_graph

mouse over /turtle1/command_velocity
ROS nodes in blue and green
topics in red

$ rostopic -h
$ rostopic list -h

analogy: topic is like a forum b/t publishers and subscribers

message types b/t publishers/subscribers should havve the same type of message

$ rostopic pub [topic] [msg_type] [args]

$ rostopic hz reports the rate at which data is published.

$ rosrun rqt_plot rqt_plot
