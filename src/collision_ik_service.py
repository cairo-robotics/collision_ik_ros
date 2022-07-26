#! /usr/bin/env python3

import csv
import ctypes
import numpy
import os
import rospkg
import rospy
import sys
import utils
import transformations as T
import yaml
from functools import partial

from collision_ik.msg import EEPoseGoals, JointAngles
from collision_ik.srv import CollisionIKSolution, CollisionFKSolution
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from timeit import default_timer as timer
from visualization_msgs.msg import InteractiveMarkerFeedback, InteractiveMarkerUpdate

from cairo_planning_core import CONFIG_PATH, Agent


def marker_feedback(msg, agent):
    pos_arr = (ctypes.c_double * 3)()
    quat_arr = (ctypes.c_double * 4)()
    pos_arr[0] = msg.pose.position.x
    pos_arr[1] = msg.pose.position.y
    pos_arr[2] = msg.pose.position.z
    quat_arr[0] = msg.pose.orientation.x
    quat_arr[1] = msg.pose.orientation.y
    quat_arr[2] = msg.pose.orientation.z
    quat_arr[3] = msg.pose.orientation.w
    # Call the rust callback function
    agent.dynamic_obstacle_cb(msg.marker_name, pos_arr, quat_arr)

def marker_update(msg, agent):
    # update dynamic collision obstacles in relaxed IK
    for pose_stamped in msg.poses:
        pos_arr = (ctypes.c_double * 3)()
        quat_arr = (ctypes.c_double * 4)()
        pos_arr[0] = pose_stamped.pose.position.x
        pos_arr[1] = pose_stamped.pose.position.y
        pos_arr[2] = pose_stamped.pose.position.z
        quat_arr[0] = pose_stamped.pose.orientation.x
        quat_arr[1] = pose_stamped.pose.orientation.y
        quat_arr[2] = pose_stamped.pose.orientation.z
        quat_arr[3] = pose_stamped.pose.orientation.w
        # Call the rust callback function
        agent.dynamic_obstacle_cb(pose_stamped.name, pos_arr, quat_arr)
class CollisionIKServiceHandler():
    
    def __init__(self, env_settings_file_path):
         # Load the infomation
        env_settings_file = open(env_settings_file_path, 'r')
        env_settings = yaml.load(env_settings_file, Loader=yaml.FullLoader)

        if 'loaded_robot' in env_settings:
            robot_info = env_settings['loaded_robot']
        else:
            raise NameError('Please define the relevant information of the robot!')

        info_file_name = robot_info['name']
        robot_name = info_file_name.split('_')[0]
        objective_mode = robot_info['objective_mode']
        print("\CollisionIK initialized!\nRobot: {}\nObjective mode: {}\n".format(robot_name, objective_mode))
        self.ik_service = rospy.Service('collision_ik/inverse_kinematics', CollisionIKSolution, self._handle_ik_call)
        self.fk_service = rospy.Service('collision_ik/forward_kinematics', CollisionFKSolution, self._handle_fk_call)
        
        # Rusty Robot Agent
        self.rusty_agent = Agent(env_settings_file_path, False, False)
        
        # Publishers
        time_pub = rospy.Publisher('/collision_ik/current_time', Float64, queue_size=10)
        self.ee_pose_pub = rospy.Publisher('/collision_ik/ee_pose_goals', EEPoseGoals, queue_size=10)
        self.jas_pub = rospy.Publisher('/collision_ik/joint_angle_solutions', JointAngles, queue_size=10)

        marker_feedback_cb = partial(marker_feedback, agent=self.rusty_agent)
        marker_update_cb = partial(marker_update, agent=self.rusty_agent)
            
        # Subscribers use to update the collision object positions etc in the Rust-based CollisionIK agent.
        self.marker_feedback_sub = rospy.Subscriber('/simple_marker/feedback', InteractiveMarkerFeedback, marker_feedback_cb)
        self.marker_update_sub = rospy.Subscriber('/simple_marker/update', InteractiveMarkerUpdate, marker_update_cb)
    
    def _handle_ik_call(self, req):
        pose_goals = req.ee_pose_goals.ee_poses
        header = req.ee_pose_goals.header
        self.ee_pose_pub.publish(req.ee_pose_goals)
        pos_arr = []
        quat_arr = []
        for i in range(len(pose_goals)):
            p = pose_goals[i]
            pos_arr.append(p.position.x)
            pos_arr.append(p.position.y)
            pos_arr.append(p.position.z)

            quat_arr.append(p.orientation.x)
            quat_arr.append(p.orientation.y)
            quat_arr.append(p.orientation.z)
            quat_arr.append(p.orientation.w)

        xopt = self.rusty_agent.collision_ik(pos_arr, quat_arr)
        ja = JointAngles()
        ja.header = header
        ja_str = "["
        for i in range(xopt.length):
            ja.angles.data.append(xopt.data[i])
            ja_str += str(xopt.data[i])
            if i == xopt.length - 1:
                ja_str += "]"
            else: 
                ja_str += ", "
        self.jas_pub.publish(ja)
        return ja

    def _handle_fk_call(self, req):
        fk_results = self.rusty_agent.forward_kinematics([value for value in req.angles.data])
        pose = Pose()
       
        init_pos = fk_results[0]
        init_rot = fk_results[1]
        pose.position.x = init_pos[0]
        pose.position.y = init_pos[1]
        pose.position.z = init_pos[2]
        
        pose.orientation.x = init_rot[0]
        pose.orientation.y = init_rot[1]
        pose.orientation.z = init_rot[2]
        pose.orientation.w = init_rot[3]

        return pose
        

def main(args=None):
    rospy.init_node('collision_ik_service')
    path_to_src = rospkg.RosPack().get_path('collision_ik')
    env_settings_file_path =  path_to_src + "/config/settings.yaml"
    cik_sh = CollisionIKServiceHandler(env_settings_file_path)
    rospy.spin()
        
if __name__ == '__main__':
    main()
