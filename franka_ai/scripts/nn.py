#!/usr/bin/env python

import rospy
import rospkg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

import torch
import torch.nn as nn
from safetensors.torch import load_model
import os
import numpy as np
import pickle
import mujoco
import mink


CTRL_DIM = 7
OBS_DIM = 16


model = None
configuration = None

agent = None
obs_mean = None
obs_std = None

phase_phi = 0.0


class Agent(nn.Module):
    def __init__(self):
        super().__init__()
        self.actor_mean = nn.Sequential(
            nn.Linear(OBS_DIM, 512),
            nn.Tanh(),
            nn.Linear(512, 512),
            nn.Tanh(),
            nn.Linear(512, CTRL_DIM),
        )


def main():
    rospy.init_node('nn')





    # ---- Mujoco setup ----

    global model

    pkg = 'franka_ai'
    rospack = rospkg.RosPack()
    pkg_share = rospack.get_path(pkg)
    model_file = os.path.join(pkg_share, 'mujoco', 'franka_emika_panda', 'panda_nohand.xml')
    rospy.loginfo('Loading mujoco model: %s', model_file)
    model = mujoco.MjModel.from_xml_path(model_file)
    rospy.loginfo('Model loaded successfully')






    # ---- Mink setup ----

    global configuration, tasks, end_effector_task, posture_task, limits

    rospy.loginfo('Loading mink')
    configuration = mink.Configuration(model)
    rospy.loginfo('Mink setup complete')





    # ---- NN setup ----

    global agent, obs_mean, obs_std

    pkg = 'franka_ai'
    rospack = rospkg.RosPack()
    pkg_share = rospack.get_path(pkg)
    tensors_file = os.path.join(pkg_share, 'models', 'agent.safetensors')
    mean_std_file = os.path.join(pkg_share, 'models', 'mean_std.pkl')
    rospy.loginfo('Loading NN model: %s', tensors_file)
    agent = Agent()
    load_model(agent, tensors_file, strict=False)
    agent.eval()
    rospy.loginfo('Loading mean_std: %s', mean_std_file)
    with open(mean_std_file, 'rb') as f:
        obs_mean, obs_std = pickle.load(f)
    rospy.loginfo('NN loaded successfully')




    # ---- ROS setup ----

    global equilibrium_pose_pub

    equilibrium_pose_pub = rospy.Publisher('equilibrium_pose', PoseStamped, queue_size=1)
    rospy.Subscriber('joint_states', JointState, joint_state_callback, queue_size=1)

    rospy.spin()


def joint_state_callback(msg):
    global phase_phi

    joint_positions = msg.position[:7]
    joint_velocities = msg.velocity[:7]
    phase_sin = np.sin(phase_phi)
    phase_cos = np.cos(phase_phi)

    obs = np.concatenate([
        joint_positions,
        joint_velocities,
        phase_sin,
        phase_cos,
    ])

    obs = (obs - obs_mean) / obs_std

    with torch.inference_mode():
        ctrl = agent.actor_mean(torch.Tensor(obs[np.newaxis, :])).numpy()[0]

    position_setpoints = joint_positions + ctrl.clip(np.radians(-10), np.radians(10))
    configuration.update(position_setpoints)
    new_ee_transform = configuration.get_transform_frame_to_world("gripper_site", "site")
    w, x, y, z, x, y, z = new_ee_transform.wxyz_xyz

    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "panda_link0"
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    pose_msg.pose.position.z = z
    pose_msg.pose.orientation.x = x
    pose_msg.pose.orientation.y = y
    pose_msg.pose.orientation.z = z
    pose_msg.pose.orientation.w = w

    equilibrium_pose_pub.publish(pose_msg)

    phase_phi += 0.001
    if phase_phi > np.pi:
        phase_phi -= 2 * np.pi

if __name__ == '__main__':
    main()
