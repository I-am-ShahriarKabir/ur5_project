#!/usr/bin/env python3
import rospy
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def talker():
    # Publish to the command topic of the controller (adjust topic name as needed)
    pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('ur5_sine_wave_commander', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.Time.now().to_sec()

    # List the joint names in the same order as defined in your UR5 model.
    joint_names = [
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    ]

    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - start_time

        traj_msg = JointTrajectory()
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        # Compute sine wave positions for each joint.
        point.positions = [math.sin(t + i * 0.5) for i in range(len(joint_names))]
        # Set the time duration in which the controller should reach these positions.
        point.time_from_start = rospy.Duration(1.0)

        traj_msg.points = [point]

        pub.publish(traj_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
