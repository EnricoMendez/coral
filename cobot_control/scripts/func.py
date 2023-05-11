#!/usr/bin/env python3

import roslaunch
import rospy
import rospkg

### Constants
rospack = rospkg.RosPack()
rospack.list()

def launch_file(pkg,file):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    hand_pkg_path = str(rospack.get_path(pkg))
    path_to_launch_file = hand_pkg_path+'/launch/'+file
    launch_file = roslaunch.rlutil.resolve_launch_arguments([path_to_launch_file])[0]
    output = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
    return output
    