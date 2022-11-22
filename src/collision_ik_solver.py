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
from std_msgs.msg import Float64
from timeit import default_timer as timer
from visualization_msgs.msg import InteractiveMarkerFeedback, InteractiveMarkerUpdate

path_to_src = rospkg.RosPack().get_path('collision_ik')
animation_folder_path = path_to_src + '/animation_files/'
geometry_folder_path = path_to_src + '/geometry_files/'
# env_settings_file_path = CONFIG_PATH + '/settings.yaml'
env_settings_file_path =  path_to_src + "/config/settings.yaml"


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

eepg = None
def eePoseGoals_cb(msg):
    global eepg
    eepg = msg

def main(args=None):
    rospy.init_node('collision_ik')

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
    
    # Rusty Robot Agent
    rusty_agent = Agent(env_settings_file_path, False, False)
    
    # Publishers
    angles_pub = rospy.Publisher('/collision_ik/joint_angle_solutions', JointAngles, queue_size=10)
    time_pub = rospy.Publisher('/collision_ik/current_time', Float64, queue_size=10)

    marker_feedback_cb = partial(marker_feedback, agent=rusty_agent)
    marker_update_cb = partial(marker_update, agent=rusty_agent)
        
    # Subscribers
    rospy.Subscriber('/simple_marker/feedback', InteractiveMarkerFeedback, marker_feedback_cb)
    rospy.Subscriber('/simple_marker/update', InteractiveMarkerUpdate, marker_update_cb)

    cur_time = 0.0
    delta_time = 0.01
    step = 1 / 30.0
  
    global eepg
    rospy.Subscriber('/collision_ik/ee_pose_goals', EEPoseGoals, eePoseGoals_cb)

    while eepg == None: continue

    rate = rospy.Rate(750)
    speed_list = []
    while not rospy.is_shutdown():
        cur_time_msg = Float64()
        cur_time_msg.data = cur_time
        time_pub.publish(cur_time_msg)
        cur_time += delta_time * step

        pose_goals = eepg.ee_poses
        header = eepg.header
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

        start = timer()
        xopt = rusty_agent.collision_ik(pos_arr, quat_arr)
        end = timer()
        speed = 1.0 / (end - start)
        
        if speed < 3:
            print("Slow collision_ik call.")
        # print("Speed: {}".format(speed))
        speed_list.append(speed)

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
        angles_pub.publish(ja)
        # print(ja_str)
        rate.sleep()

    print("Average speed: {} HZ".format(numpy.mean(speed_list)))
    print("Min speed: {} HZ".format(numpy.min(speed_list)))
    print("Max speed: {} HZ".format(numpy.max(speed_list)))

if __name__ == '__main__':
    main()
