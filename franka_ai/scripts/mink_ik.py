#!/usr/bin/env python

import rospy
import rospkg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import os
import numpy as np
import mink
import mujoco


joint_group_position_controller_pub = None

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
timestep = 0.05


def joint_state_callback(msg):
    rospy.logdebug('Received joint states:\n%s', msg)
    configuration.update(msg.position[:7])


def pose_callback(msg):
    rospy.logdebug('Received pose:\n%s', msg.pose)

    # Save configuration before solving inplace
    qpos0 = configuration.q[:]

    end_effector_task.set_target(mink.SE3(
        wxyz_xyz=np.array([
            msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        ])
    ))

    for _ in range(max_iters):
        vel = mink.solve_ik(
            configuration, tasks, timestep, solver, limits=limits
        )
        configuration.integrate_inplace(vel, timestep)
        err = end_effector_task.compute_error(configuration)
        pos_achieved = np.linalg.norm(err[:3]) <= pos_threshold
        ori_achieved = np.linalg.norm(err[3:]) <= ori_threshold
        if pos_achieved and ori_achieved:
            rospy.logdebug('Solved within threshold')
            break
    else:
        pos_error = np.linalg.norm(err[:3])
        ori_error = np.linalg.norm(err[3:])
        rospy.logwarn('Failed to solve within threshold: pos_error=%f, ori_error=%f', pos_error, ori_error)

    msg = Float64MultiArray()
    msg.data = configuration.q[:] # Exclude gripper joint
    joint_group_position_controller_pub.publish(msg)

    # Restore configuration to actual robot state
    configuration.update(qpos0)


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
            position_cost=4.0,
            orientation_cost=1.0,
            lm_damping=1e-6,
        ),
        posture_task := mink.PostureTask(model, cost=1e-1),
    ]

    limits = [
        mink.ConfigurationLimit(model=configuration.model),
    ]

    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)
    configuration.update(data.qpos)
    posture_task.set_target_from_configuration(configuration)

    rospy.loginfo('Mink setup complete')




    global joint_group_position_controller_pub
    joint_group_position_controller_pub = rospy.Publisher('joint_group_position_controller/command', Float64MultiArray, queue_size=1)
    rospy.Subscriber('joint_states', JointState, joint_state_callback)
    rospy.Subscriber('equilibrium_pose', PoseStamped, pose_callback)
    rospy.spin()


if __name__ == '__main__':
    main()
