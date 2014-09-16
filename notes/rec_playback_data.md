http://wiki.ros.org/ROS/Tutorials/Recording%20and%20playing%20back%20data

see what's up with topics
$ rostopic list -v

only published messages are recorded

###
to record the published data..
$ mkdir ~/bagfiles
$ cd ~/bagfiles
$ rosbag record -a
control-C when done with recording

the resulting file (with the .bag suffix) has all the topics published by any node while rosbag record was happening

###
to examine and play the bag file

$ rosbag info <your bag file> 
