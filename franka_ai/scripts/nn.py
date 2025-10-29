#!/usr/bin/env python

import rospy
import rospkg
from franka_msgs.msg import FrankaState

import torch
import torch.nn as nn
from safetensors.torch import load_model
import os
import numpy as np
import pickle


CTRL_DIM = 7
OBS_DIM = 16


agent = None
obs_mean = None
obs_std = None


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
    global agent
    global obs_mean, obs_std

    rospy.init_node('nn')

    pkg = 'franka_ai'
    rospack = rospkg.RosPack()
    pkg_share = rospack.get_path(pkg)
    tensors_file = os.path.join(pkg_share, 'models', 'agent.safetensors')
    mean_std_file = os.path.join(pkg_share, 'models', 'mean_std.pkl')
    rospy.loginfo('Loading model: %s', tensors_file)
    agent = Agent()
    load_model(agent, tensors_file, strict=False)
    agent.eval()
    rospy.loginfo('Loading mean_std: %s', mean_std_file)
    with open(mean_std_file, 'rb') as f:
        obs_mean, obs_std = pickle.load(f)
    rospy.loginfo('NN loaded successfully')

    rospy.Subscriber('franka_state_controller/franka_states', FrankaState, robot_state_callback)

    rospy.spin()


def robot_state_callback(msg):
    joint_positions = ...
    joint_velocities = ...
    phase_sin = ...
    phase_cos = ...

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

    position_controller_pub.publish(setpoint_msg)

if __name__ == '__main__':
    main()
