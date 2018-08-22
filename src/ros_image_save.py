#!/usr/bin/env python
#coding: utf-8
import cv2
import time
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
import os

bridge = CvBridge()

path = "/home/clarence/Desktop/Video/"
counter_img = 0
range_max = 5.0


def findPillar(depth_img):
    """
    This function is used to find pillar
    pl,pr,pd = findPillar(depth)
    :param depth:
    :return:pl,pr,pd,which means pillar right, pillar left, and the mean depth
    """
    depth = depth_img.copy()
    depth[np.where(depth > 10)] = 0
    depth = depth/10.0*255
    depth = depth.astype(np.uint8)
    # np.isnan()

    # global counter_img
    # cv2.imwrite(path + str(counter_img) + ".png", depth)
    # counter_img += 1
    # return None, None, None

    plt.ion()
    blur_1 = cv2.medianBlur(depth, 15)
    #blur_1 = blur_1>100
    plt.imshow(blur_1)
    plt.show()
    plt.pause(0.5)
    plt.clf()


    _, bw = cv2.threshold(blur_1, 37, 255, cv2.THRESH_BINARY_INV)
    kernel = np.ones((10, 10), np.uint8)        # adj params for erosion
    erosion = cv2.erode(bw, kernel, iterations=2)
    #plt.imshow(erosion)
    #plt.show()

    center = int(depth.shape[0]/2.0)
    rows_1 = erosion[center:center+50, :]    # adj params for rows region
    rows_1[:, 0:rows_1.shape[1]/5] = 0
    rows_1[:, rows_1.shape[1]*4/5:rows_1.shape[1]] = 0
    blur = cv2.medianBlur(rows_1, 11)        # adj params for median fliter
    line_1 = blur.sum(axis=0)
    win_c = int(line_1.shape[0]/2.0)

    plt.plot(line_1)
    plt.show()
    plt.pause(0.1)
    plt.clf()
    plt.ioff()


    # find the pillar range
    pillar = np.where(line_1[win_c-200:win_c+200]>line_1.max()*0.8)    # adj paras 200, and 0.8

    if len(pillar) > 0:
        try:
            print("entry")
            print(pillar[0][0])
            pl, pr = pillar[0][0]+win_c-200,pillar[0][-1]+win_c-200
            pd = depth[center:center+50, pl:pr]
            cv2.line(depth,(pl,center),(pr,center),(0),10)
            cv2.rectangle(depth,(pl,center),(pr,center+50),(255),5)
            cv2.imshow("reslut",depth)
            return pl, pr, pd.median()
        except Exception as e:
            print(e)
            return None, None, None
        except:
            print("pillar is not in center")
            return None, None, None


def callback(data):
    try:
        img = bridge.imgmsg_to_cv2(data, desired_encoding='32FC1')
    except CvBridgeError as e:
        print(e)
        return
    findPillar(img)
    pl, pr, pd = findPillar(img)
    print(pl, pr, pd)

    cv2.waitKey(10)


if __name__ == '__main__':

    rospy.init_node('optical_flow')
    image_sub = rospy.Subscriber("/zed/depth/depth_registered", Image, callback)

    rospy.spin()