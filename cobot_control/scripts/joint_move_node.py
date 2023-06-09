#!/usr/bin/env python3

import sys
from math import pi

import actionlib
import geometry_msgs.msg as geometry_msgs
import rospy
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
from std_msgs.msg import Float32MultiArray, String
from trajectory_msgs.msg import JointTrajectoryPoint

from igm import igm

# If your robot description is created with a tf_prefix, those would have to be adapted
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

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
        rospy.on_shutdown(self.cleanup)
        ### Constants
        self.igm = igm()

        self.rx = 0
        self.ry =-pi
        self.rz = 0

        self.x0 = -0.22275
        self.y0 = -0.42281
        self.z0 = 0.129553
        
        self.x1 = 0.2297
        self.y1 = -0.6430
        self.z1 = 0.44
        
        # Variables
        self.posex = 0
        self.posey = 0
        self.posez = 0
        

        # Suscribers
        rospy.Subscriber("/coordinates_coral",Float32MultiArray,self.callback_coordinates)
        
        # Publishers
        self.flag_pub=rospy.Publisher("flag_coordinates",String,queue_size=10)
        self.finish_pub=rospy.Publisher("move_finish",String,queue_size=10)
        self.status_pub=rospy.Publisher("/move/status",String,queue_size=10)

        

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

        self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[0]
        r = rospy.Rate(20)
        self.status_pub.publish('Node init')
        while not rospy.is_shutdown():
            self.flag_pub.publish('now')
            r.sleep()

    def send_joint_trajectory(self):
        self.status_pub.publish('Starting movement')

        """Creates a trajectory and sends it using the selected action server"""

        # make sure the correct controller is loaded and activated
        self.switch_controller(self.joint_trajectory_controller)
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(self.joint_trajectory_controller),
            FollowJointTrajectoryAction,
        )

        # Wait for action server to be ready
        timeout = rospy.Duration(5)
        if not trajectory_client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)

        # Create and fill trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES

        # The following list are arbitrary positions
        # Change to your own needs if desired
        position_list = [self.best]
        duration_list = [1.4]
        for i, position in enumerate(position_list):
            point = JointTrajectoryPoint()
            point.positions = position
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        rospy.loginfo("Executing trajectory using the {}".format(self.joint_trajectory_controller))

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
        self.status_pub.publish('I sent the finish flag')
        self.finish_pub.publish('now')

    def callback_coordinates(self,msg):
        self.status_pub.publish('I recieved coordinates')

        x = round(msg.data[0],4)
        y = 0.5
        z = round(msg.data[1],4)+0.1

        self.posex = round(x * (self.x1-self.x0) + self.x0,4) * 1000
        self.posey = round(y * (self.y1-self.y0) + self.y0,4) * 1000
        self.posez = round(z * (self.z1-self.z0) + self.z0,4) * 1000

        U_params = [self.posex,self.posey,self.posez, self.rx,self.ry,self.rz]
        U = self.igm.vec2rot(U_params[3:6])
        U[:3, 3] = U_params[0:3]

        Q = []
        for i in range(1,9):
            q_igm = self.igm.UR_IGM(U, i)
            solution=[]
            element = q_igm.tolist()
            for number in element:
                solution.append(round(number[0],4))
            Q.append(solution)
        
        self.best = self.igm.select(Q)
        self.send_joint_trajectory()




if __name__ == "__main__":
    
    # Node init
    rospy.init_node("Move_node")
    
    client = TrajectoryClient()

