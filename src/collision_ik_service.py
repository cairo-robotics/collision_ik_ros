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
        self.rusty_agent = Agent(env_settings_file_path, True, True)
        
        # Publishers
        self.ee_pose_pub = rospy.Publisher('/collision_ik/ee_pose_goals', EEPoseGoals, queue_size=10)

        # the solver loops in a separate process in real time and published to this topic. We pass the results along as the current IK result.
        self.jas_pub = rospy.Subscriber('/collision_ik/joint_angle_solutions', JointAngles, self._current_ja_cb)
        ja = JointAngles()
        ja.angles.data = [0, 0, 0, 0, 0, 0, 0]
        self.current_joint_angles = ja

    def _handle_ik_call(self, req):
        self.ee_pose_pub.publish(req.ee_pose_goals)
        return self.current_joint_angles

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

    def _current_ja_cb(self, msg):
        self.current_joint_angles = msg
        

def main(args=None):
    rospy.init_node('collision_ik_service')
    path_to_src = rospkg.RosPack().get_path('collision_ik')
    env_settings_file_path =  path_to_src + "/config/settings.yaml"
    cik_sh = CollisionIKServiceHandler(env_settings_file_path)
    rospy.spin()
        
if __name__ == '__main__':
    main()
