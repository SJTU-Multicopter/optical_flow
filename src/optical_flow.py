#!/usr/bin/env python

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import time
import matplotlib.pyplot as plt
from matplotlib import animation

global vx_new, vy_new
vx_new = 0.0
vy_new = 0.0


class Plot:
    def __init__(self):
        fig = plt.figure()
        ax1 = fig.add_subplot(2, 1, 1, xlim=(0, 1000), ylim=(-50, 50))  # graph length, height, index, x range, y range
        ax2 = fig.add_subplot(2, 1, 2, xlim=(0, 1000), ylim=(-50, 50))

        self.line1, = ax1.plot([], [], lw=2)
        self.line2, = ax2.plot([], [], lw=2)

        self.vx_show = np.zeros(1000)
        self.vy_show = np.zeros(1000)

        anim1 = animation.FuncAnimation(fig, self.updateData, init_func=self.initLine, frames=1000, interval=33)
        plt.show()

    def initLine(self):
        self.line1.set_data([], [])
        self.line2.set_data([], [])

        return self.line1, self.line2

    def updateData(self, i):
        x = np.linspace(0, 1000, 1000)

        if i == 0:
            self.vx_show = np.zeros(1000)
            self.vy_show = np.zeros(1000)

        global vx_new, vy_new
        self.vx_show[i] = vx_new
        self.vy_show[i] = vy_new

        self.line2.set_data(x, self.vx_show)
        self.line1.set_data(x, self.vy_show)
        return self.line1, self.line2



class OpticalFlow:
    def __init__(self):
        self.bridge = CvBridge()

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.poseCallBack)
        rospy.Subscriber("/opt_image", Image, self.imgCallBack)

        self.frame0 = []
        self.frame1 = []

        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.height = 1.0

        self.first_image_recieved = 0

        self.feature_params = dict(maxCorners=3,
                                   qualityLevel=0.3,
                                   minDistance=7,
                                   blockSize=7)

        self.lk_params = dict(winSize=(15, 15),
                              maxLevel=2,
                              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        print "Initialized"

        plot = Plot()

    def poseCallBack(self, pose):

        self.height = pose.pose.position.z
        att_x = pose.pose.orientation.x
        att_y = pose.pose.orientation.y
        att_z = pose.pose.orientation.z
        att_w = pose.pose.orientation.w

        self.yaw = np.arctan2(2 * att_y * att_x - 2 * att_z * att_w, -2 * att_y * att_y - 2 * att_w * att_w + 1) + np.pi
        self.pitch = np.arcsin(2 * att_z * att_x - 2 * att_y * att_w)
        self.roll = np.arctan2(2 * att_x * att_w + 2 * att_y * att_z, 1 - 2 * att_y * att_y - 2 * att_x * att_x)

        print "pitch: " + str(self.pitch) + " roll: " + str(self.roll) + " yaw: " + str(self.yaw)

    def imgCallBack(self, image):
        # Convert image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # For test
        # cv2.imshow("capture2", cv_image)
        # cv2.waitKey(10)

        if self.first_image_recieved == 0:
            self.first_image_recieved = 1
            self.frame0 = cv_image[:, :, 2]
            print "First image received!"
            return

        # Cut image right below here
        self.frame1 = cv_image[:, :, 2]
        p0_check = cv2.goodFeaturesToTrack(self.frame0, mask=None, **self.feature_params);
        p1_check = cv2.goodFeaturesToTrack(self.frame1, mask=None, **self.feature_params);

        # Calculate velocity here
        if (p0_check is None) or (p1_check is None):
            print 'bad image'
            return

        try:
            Vx, Vy = self.flowMove(self.frame0, self.frame1, self.lk_params, self.feature_params)
            self.frame0 = self.frame1
            global vx_new, vy_new
            vx_new = Vx
            vy_new = Vy
            print 'Vx:', Vx, 'Vy:', Vy
        except Exception as e:
            print e

    def flowMove(self, frame0, frame1, lk_params, feature_params):
        """fra,e0,frame1 is gray is rgbframe[:,:,2]"""
        ds = 0.000001  # camera pixel size
        drone_h = self.height  # height ,dynemics input
        fx = 0.3  # camera params
        dw = ds * drone_h / fx

        tic = time.time()

        p0 = cv2.goodFeaturesToTrack(frame0, mask=None, **feature_params);
        p1, st, err = cv2.calcOpticalFlowPyrLK(frame0, frame1, p0, None, **lk_params)

        if p1 is not None:
            good_new = p1[st == 1]
            good_old = p0[st == 1]
            vx = []
            vy = []
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = new.ravel()
                c, d = old.ravel()
                vx.append(a - c)
                vy.append(b - d)
            vxm = np.array(vx).mean()
            vym = np.array(vy).mean()
            # toc = time.time()
            # dt = toc - tic
            # output = 'Vx:' + str(vxm / dt * dw) + 'Vy:' + str(vym / dt * dw)

            # font = cv2.FONT_HERSHEY_SIMPLEX
            # imgzi = cv2.putText(frame0, output, (5, 50), font, 1.2, 255, 2)
            cv2.imshow("frame", frame0)
            cv2.waitKey(2)

            # print 1/dt,'Hz'  #update frequence
        return vxm, vym


if __name__ == '__main__':
    rospy.init_node('optical_flow')

    optical_flow = OpticalFlow()

    rospy.spin()


