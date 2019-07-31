#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt


bridge = CvBridge()
X = []
Y = []


class trajectory:
    def __init__(self):
        self.X = []
        self.Y = []
        self.cv_image = Image()
        self.x = None
        self.y = None
        self.action = ''

    def listener(self):
        rospy.Subscriber("/object_position", Point, self.callback_position)
        rospy.Subscriber("/aruco_simple/result", Image, self.callback)
        rospy.Subscriber("/Action", Int32, self.callback_action)

    def callback(self, data):
        self.cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        (rows, cols, channels) = self.cv_image.shape
        if cols > 60 and rows > 60:
            cv2.circle(self.cv_image, (50, 50), 10, 255)
            length = len(self.X)
        # cv2.line(cv_image, (0, 0), (50, 50), (255, 255, 0), 2)
        length = len(self.X)
        # pts = self.coords.reshape((-1, 1, 2))
        for i in range(2, length - 1):
            # print self.X[0], self.Y[0]
            # print self.X[i], self.Y[i]
            if i == 1:
                continue
            cv2.line(self.cv_image, (self.X[i], self.Y[i]), (self.X[i - 1], self.Y[i - 1]), (255, 255, 0), 2, lineType=8)
        #cv2.line(self.cv_image, (self.X[length - 1], self.Y[length - 1]), (self.X[length - 2], self.Y[length - 2]), (255, 255, 0), 2, lineType=8)

        # cv2.polylines(cv_image, [pts], True, (0, 255, 255))
        # cv2.line(self.cv_image, (10, 10), (20, 20), (255, 255, 0), 2)
        # cv2.line(self.cv_image, (10, 10), (10, 20), (255, 255, 0), 2)
        #cv2.circle(self.cv_image, (self.X[i], self.Y[i]), 4, (0, 0, 255), -1)

        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(self.cv_image, self.action, (10, 250), font, 1, (0, 0, 255), 2)
        cv2.imshow("Image window", self.cv_image)
        cv2.waitKey(3)

    def callback_position(self, msg):
        # print 123
        global X, Y
        self.x = int(msg.x * 1942.347 + 298.124)
        self.y = int(-msg.y * 1984. 738  + 243.968)
        self.X.append(self.x)
        self.Y.append(self.y)

        print "Length of array=", len(self.X)
        X = self.X
        Y = self.Y
        # self.coords.append([self.x, self.y])
        # print self.x, self.y

    def callback_action(self, act):
        if (act == 0):
            self.action = 'Left Slide down'
        if (act == 1):
            self.action = 'Right Slide down'
        if (act == 3):
            self.action = 'Left Slide up'
        if (act == 4):
            self.action = 'Right Slide up'
        if (act == 5):
            self.action = 'Anti-clockwise Rotation'
        if (act == 6):
            self.action = 'Clockwise Rotation'


'''
def callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    (rows, cols, channels) = cv_image.shape
    if cols > 60 and rows > 60:
        cv2.circle(cv_image, (50, 50), 10, 255)
    # cv2.line(cv_image, (X[len(X) - 1], Y[len(X) - 1]), (X[len(X) - 2], Y[len(X) - 2]), (255, 255, 0), 2)
    # print len(X)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)


def callback_position(msg):
    print 123
    x = msg.x * 100
    y = msg.y * 100
    X.append(x)
    Y.append(y)
    print len(X)


def listener():
    rospy.Subscriber("/aruco_simple/result", Image, callback)
    rospy.Subscriber("/aruco_simple/pose1", Point, callback_position)

'''


def main():
    global X, Y
    rospy.init_node('Image', anonymous=True)
    t = trajectory()
    t.listener()
    print X, Y
    plt.plot(X, Y)
    rospy.spin()
    # plt.show()


if __name__ == '__main__':
    main()
