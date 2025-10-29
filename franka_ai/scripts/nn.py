#!/usr/bin/env python

import rospy
import rospkg

import torch
import torch.nn as nn
from safetensors.torch import load_model
import os
import numpy as np
import pickle


CTRL_DIM = 7
OBS_DIM = 16


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

    pkg = 'franka_ai'
    rospack = rospkg.RosPack()
    pkg_share = rospack.get_path(pkg)
    tensors_file = os.path.join(pkg_share, 'models', 'agent.safetensors')
    mean_std_file = os.path.join(pkg_share, 'models', 'mean_std.pkl')
    rospy.loginfo('Loading model: %s', tensors_file)
    agent = Agent()
    load_model(agent, tensors_file, strict=False)
    rospy.loginfo('Loading mean_std: %s', mean_std_file)
    with open(mean_std_file, 'rb') as f:
        obs_mean, obs_std = pickle.load(f)
    rospy.loginfo('NN loaded successfully')

    rospy.spin()


if __name__ == '__main__':
    main()
