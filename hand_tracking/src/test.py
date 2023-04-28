#!/usr/bin/env python3

import rospkg

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

# list all packages, equivalent to rospack list
rospack.list() 

# get the file path for rospy_tutorials
path_to_pkg = str(rospack.get_path('hand_tracking'))
print(path_to_pkg)

