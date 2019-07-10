#!/usr/bin/env python
# -*- coding: utf-8 -*-


import sys
import os
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class SaveImage(object):
    def __init__(self):
        image_topic = rospy.get_param('~image_topic', '/kinect2/hd/image_color')
        self.time_span = rospy.get_param('~time_span', 3)
        self.save_directory = rospy.get_param('~save_directory', '~/ros/data/saved_image/')
        if not os.path.exists(self.save_directory):
            os.mkdir(self.save_directory)
        if not self.save_directory[-1] == '/':
            self.save_directory += '/'
        self.__sub = rospy.Subscriber(image_topic, Image, self.callback)
        self.__temp_image = None
        self.__bridge = CvBridge()

    def callback(self, msg):
        """
        Callback function
        """
        try:
            self.__temp_image = self.__bridge.imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            rospy.logerr(e)

    def save_image(self, filename):
        """
        Save self.__temp_image with the given file name
        """
        if not self.__temp_image is None:
            rospy.loginfo('Saving image as {}'.format(filename))
            cv2.imwrite(filename, self.__temp_image)
            return True
        return False

    def main(self):
        """
        Save image every $(time_span) seconds
        """
        image_id = 0
        try:
            while True:
                filename = self.save_directory + str(image_id).zfill(5) + '.jpg'
                if not self.save_image(filename):
                    rospy.logwarn('No image saved')
                else:
                    image_id += 1
                rospy.sleep(self.time_span)
        except KeyboardInterrupt:
            sys.exit(0)


if __name__ == '__main__':
    rospy.init_node('save_image')
    SAVE = SaveImage()
    while True:
        try:
            SAVE.main()
        except KeyboardInterrupt:
            sys.exit(0)
    rospy.spin()
