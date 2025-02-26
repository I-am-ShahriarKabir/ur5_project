#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState, Image
import csv
from cv_bridge import CvBridge
import cv2

class DataCollector:
    def __init__(self):
        # Open a CSV file to store joint data (this will be created in the working directory)
        self.joint_file = open('joint_data.csv', 'w')
        self.joint_writer = csv.writer(self.joint_file)
        # Write header row
        self.joint_writer.writerow(['time', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'])
        
        # For converting ROS images to OpenCV format (if a camera is used)
        self.bridge = CvBridge()
        
        # Subscribe to joint states and camera image topics
        rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
    
    def joint_callback(self, msg):
        # Record joint state data with timestamp
        t = msg.header.stamp.to_sec()
        self.joint_writer.writerow([t] + list(msg.position))
    
    def image_callback(self, msg):
        try:
            # Convert the ROS image to an OpenCV image and display it for real-time visualization
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Camera View", cv_image)
            cv2.waitKey(1)
            # Optionally, images can be saved to disk or forwarded to another service.
        except Exception as e:
            rospy.logerr("Image conversion failed: %s", str(e))
    
    def shutdown(self):
        # Cleanly close the CSV file and any OpenCV windows on shutdown
        self.joint_file.close()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('data_collector', anonymous=True)
    collector = DataCollector()
    rospy.on_shutdown(collector.shutdown)
    rospy.spin()
