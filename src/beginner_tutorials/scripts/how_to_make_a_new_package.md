Go to your github repo's src folder
```
cd ~/comprobo2014/src
```

Use the catkin_create_pkg command 
```
catkin_create_pkg <package name> std_msgs rospy roscpp
```

Establish the symbolic link so your package in your github repo works with your catkin workspace
```
ln -s <path to github repo>/src/<package folder name> ~/catkin_ws/src
```
