#!/usr/bin/env python



# -- BEGIN LICENSE BLOCK ----------------------------------------------

# Copyright 2021 FZI Forschungszentrum Informatik

# Created on behalf of Universal Robots A/S

#

# Licensed under the Apache License, Version 2.0 (the "License");

# you may not use this file except in compliance with the License.

# You may obtain a copy of the License at

#

#     http://www.apache.org/licenses/LICENSE-2.0

#

# Unless required by applicable law or agreed to in writing, software

# distributed under the License is distributed on an "AS IS" BASIS,

# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

# See the License for the specific language governing permissions and

# limitations under the License.

# -- END LICENSE BLOCK ------------------------------------------------

#

# ---------------------------------------------------------------------

# !\file

#

# \author  Felix Exner mauch@fzi.de

# \date    2021-08-05

#

#

# ---------------------------------------------------------------------

import sys



import rospy

import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

from trajectory_msgs.msg import JointTrajectoryPoint

from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController

from controller_manager_msgs.srv import LoadControllerRequest, LoadController

from controller_manager_msgs.srv import ListControllers, ListControllersRequest

import geometry_msgs.msg as geometry_msgs

from std_msgs.msg import Float64
from std_msgs.msg import Int32 
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage

from cartesian_control_msgs.msg import (

    FollowCartesianTrajectoryAction,

    FollowCartesianTrajectoryGoal,

    CartesianTrajectoryPoint,

)



# Compatibility for python2 and python3

if sys.version_info[0] < 3:

    input = raw_input



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

    """Small trajectory client to test a joint trajectory"""



    def __init__(self):

        rospy.init_node("test_move")



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

        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[0]



    def send_joint_trajectory(self):

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

        position_list = [[0, -1.57, -1.57, 0, 0, 0]]

        position_list.append([0.2, -1.57, -1.57, 0, 0, 0])

        position_list.append([-0.5, -1.57, -1.2, 0, 0, 0])

        duration_list = [3.0, 7.0, 10.0]

        for i, position in enumerate(position_list):

            point = JointTrajectoryPoint()

            point.positions = position

            point.time_from_start = rospy.Duration(duration_list[i])

            goal.trajectory.points.append(point)



        self.ask_confirmation(position_list)

        rospy.loginfo("Executing trajectory using the {}".format(self.joint_trajectory_controller))



        trajectory_client.send_goal(goal)

        trajectory_client.wait_for_result()



        result = trajectory_client.get_result()

        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))



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



        estado_terminado = 0



        #Our Flags ;)
        self.flag = 0

        self.flag_tf = 0

        self.flag_inicio = 1

        self.flag_translation = 0

        self.flag_termine = 0

        

        #Our Subscribers from camera_position.py
        

        rospy.Subscriber("/topico_pos", PoseStamped, self.positions_cb)
        rospy.Subscriber("/tf", TFMessage, self.urt)
        
        self.result_pub = rospy.Publisher("/topico_estado", Int32, queue_size=1)
        self.home_result_pub = rospy.Publisher("/topico_estado_home", Int32, queue_size=1)
        self.translation_pub = rospy.Publisher("/topico_estado_traslacion", Int32, queue_size=1)
        self.recaptura_pub = rospy.Publisher("/topico_recaptura", Int32, queue_size=1)

        #rospy.Subscriber("posx_requerida", Float64, self.posx_cb)

        #rospy.Subscriber("posy_requerida", Float64, self.posy_cb)

        #rospy.Subscriber("posz_requerida", Float64, self.posz_cb)

        #rospy.Subscriber("orienx_requerida", Float64, self.pos_orix_cb)

        #rospy.Subscriber("orieny_requerida", Float64, self.pos_oriy_cb)

        #rospy.Subscriber("orienz_requerida", Float64, self.pos_oriz_cb)

        #rospy.Subscriber("orienw_requerida", Float64, self.pos_oriw_cb)



        while not rospy.is_shutdown():



            if self.flag_inicio == 1:

                home_list = [

                    geometry_msgs.Pose(

                        geometry_msgs.Vector3(-0.41440, -0.44350, 0.58749), geometry_msgs.Quaternion(0.00028, -0.70710, 0.70710, 0.00028)

                    )

                ]

                duration_list = [10.0]

                for i, pose in enumerate(home_list):

                    point = CartesianTrajectoryPoint()

                    point.pose = pose

                    point.time_from_start = rospy.Duration(duration_list[i])

                    goal.trajectory.points.append(point)


                print("ESTOY EN EL IF DEL HOME")
                self.ask_confirmation(home_list)

                #rospy.loginfo(

                 #   "Executing trajectory using the {}".format(self.cartesian_trajectory_controller)

                #)

                trajectory_client.send_goal(goal)

                trajectory_client.wait_for_result()



                result = trajectory_client.get_result()

                if result.error_code == 0:
                    self.home_result_pub.publish(result.error_code)
                    #self.flag_tf = 1
                    self.posx = -0.41440
                    self.posy = -0.44350
                    self.posz = 0.58749


                    self.flag_inicio = 0
                    print("YA CAMBIE EL VALOR DE LA FLAG")
                    

                rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))





            if self.flag_inicio == 0 and self.flag == 1 and self.flag_translation == 0:

                print("ENTRE AL SEGUNDO IF")
                # The following list are arbitrary positions

                # Change to your own needs if desired

                orientation_list = [

                    geometry_msgs.Pose(

                        geometry_msgs.Vector3(self.posx, self.posy, self.posz), geometry_msgs.Quaternion(self.pose_ori_x, self.pose_ori_y, self.pose_ori_z, self.pose_ori_w)

                    )

                ]

                duration_list = [20.0]

                for i, pose in enumerate(orientation_list):

                    point = CartesianTrajectoryPoint()

                    point.pose = pose

                    point.time_from_start = rospy.Duration(duration_list[i])

                    goal.trajectory.points.append(point)


                print("ESTOY EN EL IF DE ORIENTACION")
                self.ask_confirmation(orientation_list)

                rospy.loginfo(

                    "Executing trajectory using the {}".format(self.cartesian_trajectory_controller)

                )

                trajectory_client.send_goal(goal)

                trajectory_client.wait_for_result()



                result = trajectory_client.get_result()
                if result.error_code == 0:
                    self.result_pub.publish(result.error_code)
                    #self.flag_tf = 1
                    self.orx = self.pose_ori_x
                    self.ory = self.pose_ori_y
                    self.orz = self.pose_ori_z
                    self.orw = self.pose_ori_w
                    self.flag_translation = 1
                    self.flag = 0
                    self.translation_pub.publish(self.flag_translation)
                    

                rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))
                
            if self.flag_inicio == 0 and self.flag == 1 and self.flag_translation == 1:

                # The following list are arbitrary positions

                # Change to your own needs if desired

                translation_list = [

                    geometry_msgs.Pose(

                        geometry_msgs.Vector3(self.pose_x, self.pose_y, self.pose_z), geometry_msgs.Quaternion(self.orx,  self.ory,  self.orz,  self.orw)

                    )

                ]

                duration_list = [30.0]

                for i, pose in enumerate(translation_list):

                    point = CartesianTrajectoryPoint()

                    point.pose = pose

                    point.time_from_start = rospy.Duration(duration_list[i])

                    goal.trajectory.points.append(point)


                print("ESTOY EN EL IF DE TRASLACION")
                self.ask_confirmation(translation_list)

                rospy.loginfo(

                    "Executing trajectory using the {}".format(self.cartesian_trajectory_controller)

                )

                trajectory_client.send_goal(goal)

                trajectory_client.wait_for_result()



                result = trajectory_client.get_result()
                if result.error_code == 0:
                    self.result_pub.publish(result.error_code)
                    #self.flag_tf = 1
                    self.flag = 0
                    self.posx = self.pose_x
                    self.posy = self.pose_y
                    self.posz = self.pose_z
                    self.flag_tf = 1
                    self.recaptura_pub(1)
                    
                    while (True):
                        if self.flag == 1:
                            break
                            
                    a1 = abs(self.translation_x - self.pose_x)
                    #a2 = abs(self.translation_y - self.pose_y)
                    a3 = abs(self.translation_z - self.pose_z)
                    b1 = abs(self.rotation_x - self.pose_ori_x)
                    b2 = abs(self.rotation_y - self.pose_ori_y)
                    b3 = abs(self.rotation_z - self.pose_ori_z)
                    b4 = abs(self.rotation_w - self.pose_ori_w)
                    
                    if a1 <= 0.01 and a3 <= 0.01 and b1 <= 0.01 and b2 <= 0.01 and b3 <= 0.01 and b4 <= 0.01:
                        sys.exit("Posicionamiento concluido exitosamente")
                        
                    self.flag_translation = 0
                    self.translation_pub.publish(self.flag_translation)
                    
                #FALTA SEGUIR ITERANDO
                    

                rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))


    ###############################################################################################

    #                                                                                             #

    # Methods defined below are for the sake of safety / flexibility of this demo script only.    #

    # If you just want to copy the relevant parts to make your own motion script you don't have   #

    # to use / copy all the functions below.                                                       #

    #                                                                                             #

    ###############################################################################################



    def ask_confirmation(self, waypoint_list):

        """Ask the user for confirmation. This function is obviously not necessary, but makes sense

        in a testing script when you know nothing about the user's setup."""

        rospy.logwarn("The robot will move to the following waypoints: \n{}".format(waypoint_list))

        confirmed = False

        valid = False

        while not valid:

            input_str = input(

                "Please confirm that the robot path is clear of obstacles.\n"

                "Keep the EM-Stop available at all times. You are executing\n"

                "the motion at your own risk. Please type 'y' to proceed or 'n' to abort: "

            )

            valid = input_str in ["y", "n"]

            if not valid:

                rospy.loginfo("Please confirm by entering 'y' or abort by entering 'n'")

            else:

                confirmed = input_str == "y"

        if not confirmed:

            rospy.loginfo("Exiting as requested by user.")

            sys.exit(0)



    def choose_controller(self):

        """Ask the user to select the desired controller from the available list."""

        rospy.loginfo("Available trajectory controllers:")

        for (index, name) in enumerate(JOINT_TRAJECTORY_CONTROLLERS):

            rospy.loginfo("{} (joint-based): {}".format(index, name))

        for (index, name) in enumerate(CARTESIAN_TRAJECTORY_CONTROLLERS):

            rospy.loginfo("{} (Cartesian): {}".format(index + len(JOINT_TRAJECTORY_CONTROLLERS), name))

        choice = -1

        while choice < 0:

            input_str = input(

                "Please choose a controller by entering its number (Enter '0' if "

                "you are unsure / don't care): "

            )

            try:

                choice = int(input_str)

                if choice < 0 or choice >= len(JOINT_TRAJECTORY_CONTROLLERS) + len(

                    CARTESIAN_TRAJECTORY_CONTROLLERS

                ):

                    rospy.loginfo(

                        "{} not inside the list of options. "

                        "Please enter a valid index from the list above.".format(choice)

                    )

                    choice = -1

            except ValueError:

                rospy.loginfo("Input is not a valid number. Please try again.")

        if choice < len(JOINT_TRAJECTORY_CONTROLLERS):

            self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[choice]

            return "joint_based"



        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[

            choice - len(JOINT_TRAJECTORY_CONTROLLERS)

        ]

        return "cartesian"



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
        
    def urt(self, msg):
        if self.flag_tf == 1:
            self.trans = msg
            self.translation_x = self.trans.transforms[0].transform.translation.x
            self.translation_y = self.trans.transforms[0].transform.translation.y
            self.translation_z = self.trans.transforms[0].transform.translation.z
            self.rotation_x = self.trans.transforms[0].transform.rotation.x
            self.rotation_y = self.trans.transforms[0].transform.rotation.y
            self.rotation_z = self.trans.transforms[0].transform.rotation.z
            self.rotation_w = self.trans.transforms[0].transform.rotation.w
            print("YA VOY A SALIR DEL URT_CB")
            self.flag_tf = 0
    
    def positions_cb(self, msg):
        extraccion = msg
        self.pose_x = extraccion.pose.position.x
        self.pose_y = extraccion.pose.position.y
        self.pose_z = extraccion.pose.position.z
        self.pose_ori_x = extraccion.pose.orientation.x
        self.pose_ori_y = extraccion.pose.orientation.y
        self.pose_ori_z = extraccion.pose.orientation.z
        self.pose_ori_w = extraccion.pose.orientation.w
        print("YA VOY A SALIR DEL POSITION_CB")
        self.flag = 1

    

    #def posx_cb(self, msg):

        #self.pose_x = msg

        #self.flag1 = 1

    #def posy_cb(self, msg):

        #self.pose_y = msg

        #self.flag2 = 1

    #def posz_cb(self, msg):

        #self.pose_z = msg

        #self.flag3 = 1

    #def pos_orix_cb(self, msg):

        #self.pose_ori_x = msg

        #self.flag4 = 1

    #def pos_oriy_cb(self, msg):

        #self.pose_ori_y = msg

        #self.flag5 = 1

    #def pos_oriz_cb(self, msg):

        #self.pose_ori_z = msg

        #self.flag6 = 1

    #def pos_oriw_cb(self, msg):

        #self.pose_ori_w = msg   

        #self.flag7 = 1 



if __name__ == "__main__":

    client = TrajectoryClient()



    # The controller choice is obviously not required to move the robot. It is a part of this demo

    # script in order to show all available trajectory controllers.

    trajectory_type = client.choose_controller()

    if trajectory_type == "joint_based":

        client.send_joint_trajectory()

    elif trajectory_type == "cartesian":

        client.send_cartesian_trajectory()

    else:

        raise ValueError(

            "I only understand types 'joint_based' and 'cartesian', but got '{}'".format(

                trajectory_type

            )
        )
