#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        self.cmd_vel_pub = rospy.Publisher('/duckiegk/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/duckiegk/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)

        # No tag detected state
        self.no_tag_counter = 0
        
        rospy.spin() # Spin forever but listen to message callbacks

    # Apriltag Detection Callback
    def tag_callback(self, msg):
        self.move_robot(msg.detections)
 
    # Stop Robot before node has shut down. This ensures the robot doesn't keep moving with the latest velocity command
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Sends zero velocity to stop the robot
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def move_robot(self, detections):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        
        if len(detections) == 0:
            # No detections, seek an object
            self.no_tag_counter += 1
            if self.no_tag_counter > 5:
                cmd_msg.v = 0.0
                cmd_msg.omega = 0.5 # Rotate to seek object
                rospy.loginfo("No tags detected. Seeking object...")
            else:
                self.stop_robot()
                rospy.loginfo("No tags detected. Stopping robot...")
        else:
            self.no_tag_counter = 0
            # Detected an object, look at it
            x = detections[0].transform.translation.x
            y = detections[0].transform.translation.y
            z = detections[0].transform.translation.z

            rospy.loginfo("Detected AprilTag at x,y,z: %f, %f, %f", x, y, z)

            # Proportional control for angular velocity
            cmd_msg.v = 0.0
            cmd_msg.omega = -0.5 * x # Adjust this factor as needed
            rospy.loginfo("Adjusting orientation to center tag with omega: %f", cmd_msg.omega)

        self.cmd_vel_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass
