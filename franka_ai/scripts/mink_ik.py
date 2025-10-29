#!/usr/bin/env python

import rospy
import rospkg
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import os
import numpy as np
import mink
import mujoco


joint_group_position_controller_pub = None
first_joint_state_received = False

model = None

configuration = None
end_effector_task = None
posture_task = None
tasks = None
limits = None

solver = "daqp"
pos_threshold = 0.010
ori_threshold = 0.1
max_iters = 10
timestep = 0.01
critical_pos_threshold = 0.100
critical_ori_threshold = 0.3


def solve_and_publish():
    if not first_joint_state_received:
        rospy.logwarn('No joint states received yet, skipping IK')
        return

    # Save configuration before solving inplace
    qpos0 = configuration.q[:]

    solved = False

    for _ in range(max_iters):
        vel = mink.solve_ik(
            configuration, tasks, timestep, solver, limits=limits
        )
        configuration.integrate_inplace(vel, timestep)
        err = end_effector_task.compute_error(configuration)
        pos_error = np.linalg.norm(err[:3])
        ori_error = np.linalg.norm(err[3:])
        pos_achieved = pos_error <= pos_threshold
        ori_achieved = ori_error <= ori_threshold
        if pos_achieved and ori_achieved:
            rospy.logdebug('Solved within threshold')
            solved = True
            break
    else:
        if pos_error > critical_pos_threshold or ori_error > critical_ori_threshold:
            rospy.logerr('Failed to solve within critical threshold: pos_error=%f, ori_error=%f', pos_error, ori_error)
            # Do not execute infeasible solutions
        else:
            rospy.logwarn('Failed to solve within threshold: pos_error=%f, ori_error=%f', pos_error, ori_error)
            # Acceptable error
            solved = True

    if solved:
        msg = Float64MultiArray()
        msg.data = configuration.q[:]
        joint_group_position_controller_pub.publish(msg)

    # Restore configuration to actual robot state
    configuration.update(qpos0)


def joint_state_callback(msg):
    rospy.logdebug('Received joint states:\n%s', msg)
    configuration.update(msg.position[:7])
    global first_joint_state_received
    if not first_joint_state_received:
        end_effector_task.set_target_from_configuration(configuration)
        first_joint_state_received = True
    solve_and_publish()


def pose_callback(msg):
    rospy.logdebug('Received pose:\n%s', msg.pose)

    end_effector_task.set_target(mink.SE3(
        wxyz_xyz=np.array([
            msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        ])
    ))


def twist_callback(msg):
    rospy.logdebug('Received twist: %s', msg)

    current_pose = configuration.get_transform_frame_to_world("gripper_site", "site")
    new_target = (
        mink.SE3.from_rotation_and_translation(
            mink.SO3.from_rpy_radians(msg.angular.x, msg.angular.y, msg.angular.z),
            np.array([msg.linear.x, msg.linear.y, msg.linear.z]),
        )
    ) @ current_pose

    end_effector_task.set_target(new_target)


def main():
    rospy.init_node('mink_ik')





    # ---- Mujoco setup ----

    global model

    pkg = 'franka_ai'
    rospack = rospkg.RosPack()
    pkg_share = rospack.get_path(pkg)
    model_file = os.path.join(pkg_share, 'mujoco', 'franka_emika_panda', 'panda_nohand.xml')
    rospy.loginfo('Loading model: %s', model_file)
    model = mujoco.MjModel.from_xml_path(model_file)
    rospy.loginfo('Model loaded successfully')





    # ---- Mink setup ----

    global configuration, tasks, end_effector_task, posture_task, limits

    rospy.loginfo('Loading mink')
    configuration = mink.Configuration(model)
    tasks = [
        end_effector_task := mink.FrameTask(
            frame_name="gripper_site",
            frame_type="site",
            position_cost=1.0,
            orientation_cost=1.0,
            lm_damping=1.0,
        ),
        posture_task := mink.PostureTask(model, cost=1e-2),
    ]

    limits = [
        mink.ConfigurationLimit(model=configuration.model, gain=0.9, min_distance_from_limits=np.radians(10)),
        mink.VelocityLimit(model=model, velocities={
            "joint1": 2,
            "joint2": 2,
            "joint3": 2,
            "joint4": 2,
            "joint5": 2,
            "joint6": 2,
            "joint7": 2,
        }),
    ]

    configuration.update_from_keyframe('home')
    end_effector_task.set_target_from_configuration(configuration)
    posture_task.set_target_from_configuration(configuration)

    rospy.loginfo('Mink setup complete')





    # ---- ROS setup ----

    global joint_group_position_controller_pub
    joint_group_position_controller_pub = rospy.Publisher('joint_group_position_controller/command', Float64MultiArray, queue_size=1)
    rospy.Subscriber('joint_states', JointState, joint_state_callback, queue_size=1)
    rospy.Subscriber('equilibrium_pose', PoseStamped, pose_callback, queue_size=1)
    rospy.Subscriber('twist_cmd', Twist, twist_callback, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()
