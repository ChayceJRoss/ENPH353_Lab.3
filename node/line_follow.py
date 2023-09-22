#!/usr/bin/env python3

import rospy
import cv2
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

class LineFollower():
    '''
    A class to follow the line using the camera.

    This class defines a robot controller that uses camera input to follow a
    line on the ground. It subscribes to the camera image topic, processes the
    image, and publishes control commands to steer the robot.
    '''

    def __init__(self) -> None:
        '''
        Constructor for the LineFollower class.

        Initializes the class, sets up ROS node, subscribes to camera image topic,
        and initializes necessary parameters.

        Args:
            None

        Returns:
            None
        '''
        self.p = 20.0 / 400.0
        self.d = 10.0 / 400.0
        rospy.init_node('line_follow', anonymous=True)
        self.last_error = -1
        rospy.Subscriber("/robot/camera1/image_raw", Image, self.callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(2)
        rospy.spin()

    def find_line_centre(self, cv_image):
        '''
        Find the center of the line in the camera image.

        This function takes a camera image as input, processes it to detect the
        line, and calculates the position of the line's center.

        Args:
            cv_image (numpy.ndarray): The camera image in grayscale.

        Returns:
            int: The position of the line's center.
        '''
        # Apply a binary threshold (110.0) that was found with Otsu's method
        (_, bw) = cv2.threshold(cv_image, 110.0, 255, cv2.THRESH_BINARY)

        # Find the positions of white pixels in a specific row (line detection)
        high_positions, = np.where(bw[799, :] == 0)
        if high_positions.size == 0:
            return 0
        # Calculate the average position of white pixels
        position = int(np.average(high_positions[:]))
        return position

    def callback(self, data):
        '''
        Callback function to process camera image and control the robot.

        This function is called whenever a new camera image is received. It
        processes the image, calculates the error in line following, and publishes
        control commands to steer the robot.

        Args:
            data (sensor_msgs.msg.Image): The camera image data.

        Returns:
            None
        '''
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, "mono8")
        line_centre = self.find_line_centre(cv_image)
        move = Twist()
        error = line_centre - 400.0
        if abs(error) > 399:
            error = 399 * np.sign(self.last_error)
        move.linear.x = 1.5
        move.angular.z = -1 * (self.p * error + self.d * (error - self.last_error))
        self.last_error = error
        self.pub.publish(move)

if __name__ == '__main__':
    try:
        LineFollower()
    except rospy.ROSInterruptException:
        pass



    