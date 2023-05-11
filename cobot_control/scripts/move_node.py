#!/usr/bin/env python3


import sys

import actionlib
from func import *
import time
import geometry_msgs.msg as geometry_msgs
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from cartesian_control_msgs.msg import (CartesianTrajectoryPoint,
                                        FollowCartesianTrajectoryAction,
                                        FollowCartesianTrajectoryGoal)
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal)
from controller_manager_msgs.srv import (ListControllers,
                                         ListControllersRequest,
                                         LoadController, LoadControllerRequest,
                                         SwitchController,
                                         SwitchControllerRequest)
from trajectory_msgs.msg import JointTrajectoryPoint



# All of those controllers can be used to execute joint-based trajectories.
# The scaled versions should be preferred over the non-scaled versions.
JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]

# All of those controllers can be used to execute Cartesian trajectories.
# The scaled versions should be preferred over the non-scaled versions.
CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
]

# We'll have to make sure that none of these controllers are running, as they will
# be conflicting with the joint trajectory controllers
CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]

class TrajectoryClient:
    
    def __init__(self):
        ### Constantes
        rospy.on_shutdown(self.cleanup)
        self.flag = True
        self.posex = 0
        self.posey = 0
        self.posez = 0

        self.ori_x= 0.0004947227089390941
        self.ori_y= -0.9999966483194065
        self.ori_z= 0.0007266061044373305
        self.ori_w= 0.0024352911455313847

        rospy.init_node("move_node")
        print('Node init')

        rospy.Subscriber("/coordinates_coral",Float32MultiArray,self.callback_coordinates)
        self.flag_pub=rospy.Publisher("flag_coordinates",String,queue_size=10)
        self.finish_pub=rospy.Publisher("move_finish",String,queue_size=10)

        timeout = rospy.Duration(5)
        self.switch_srv = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
        )
        self.load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
        self.list_srv = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)
        try:
            self.switch_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller switch service. Msg: {}".format(err))
            sys.exit(-1)

        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[0]
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.flag_pub.publish('now')
            r.sleep()

    def send_cartesian_trajectory(self):
        """Creates a Cartesian trajectory and sends it using the selected action server"""
        self.switch_controller(self.cartesian_trajectory_controller)

        # make sure the correct controller is loaded and activated
        goal = FollowCartesianTrajectoryGoal()
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_cartesian_trajectory".format(self.cartesian_trajectory_controller),
            FollowCartesianTrajectoryAction,
        )

        # Wait for action server to be ready
        timeout = rospy.Duration(5)
        if not trajectory_client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)

        # The following list are arbitrary positions
        # Change to your own needs if desired
        pose_list = [
            geometry_msgs.Pose(
                geometry_msgs.Vector3(self.posex,self.posey,self.posez), 
                geometry_msgs.Quaternion(self.ori_x,self.ori_y,self.ori_z,self.ori_w)
            )
        ]
        duration_list = [1]
        for i, pose in enumerate(pose_list):
            point = CartesianTrajectoryPoint()
            point.pose = pose
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        #self.ask_confirmation(pose_list)
        rospy.loginfo(
            "Executing trajectory using the {}".format(self.cartesian_trajectory_controller)
        )
        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()

        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))
       
        rospy.signal_shutdown('Done')


    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        other_controllers = (
            JOINT_TRAJECTORY_CONTROLLERS
            + CARTESIAN_TRAJECTORY_CONTROLLERS
            + CONFLICTING_CONTROLLERS
        )

        other_controllers.remove(target_controller)

        srv = ListControllersRequest()
        response = self.list_srv(srv)
        for controller in response.controller:
            if controller.name == target_controller and controller.state == "running":
                return

        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_srv(srv)

        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_srv(srv)

    def cleanup (self):
        print('Im move is done')
        self.finish_pub.publish('now')
        print('I sent the finish flag')

    def callback_coordinates(self,msg):
        x0 = -0.3225
        y0 = -0.3430
        z0 = 0.15668

        x1 = 0.2330
        y1 = -0.621
        z1 = 0.3845

        x = msg.data[0]
        y = msg.data[1]
        z = msg.data[2]

        self.posex = x * (x1-x0) + x0
        self.posey = y * (y1-y0) + y0
        self.posez = z * (z1-z0) + z0

        self.send_cartesian_trajectory()




if __name__ == "__main__":
    
    client = TrajectoryClient()

