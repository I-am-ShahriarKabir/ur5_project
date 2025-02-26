#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import JointState

def talker():
    # Publisher topic: adjust topic name if your controller expects a specific one.
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rospy.init_node('ur5_sine_wave_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.Time.now().to_sec()
    
    # List the joint names in the same order as defined in your UR5 model.
    joint_names = [
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    ]
    
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - start_time
        # Create the JointState message with sine-wave values (phase shifted per joint)
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = joint_names
        js.position = [math.sin(t + i*0.5) for i in range(len(joint_names))]
        pub.publish(js)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
