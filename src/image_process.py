#!/usr/bin/env python

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import os
import threading
import time

fake_mode_enable = 1

class MultiCamProcess:
    def __init__(self):
        self.bridge = CvBridge()
        self.zed_depth_img = []
        self.zed_color_img = []
        self.usb_cam0_img = []
        self.usb_cam1_img = []

        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.height_received = 0
        self.height = 1.0

        zed_depth_sub = rospy.Subscriber("/zed/depth/depth_registered", Image, self.depthCallback)
        zed_color_sub = rospy.Subscriber("/zed/rgb/image_rect_color", Image, self.colorCallback)
        position_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.poseCallBack)

        self.result_pub = rospy.Publisher("/image_result", PoseStamped)

        self.result = PoseStamped()

        # mode sub !!!

        # Flags
        self.mode = 0  # Decided by outside request
        self.zed_color_required = 0  # 0: init 1: request 2: received
        self.zed_depth_required = 0
        self.usb_cam0_required = 0
        self.usb_cam1_required = 0

        # USB camera 0 thread
        t0 = threading.Thread(target=self.usbCam0Read, args=())
        t0.start()

        # USB camera 1 thread
        t1 = threading.Thread(target=self.usbCam1Read, args=())
        t1.start()

        # Main loop
        self.processLoop()

    def processLoop(self):
        r = rospy.Rate(20)  #hz

        fake_mode_counter = 0
        if fake_mode_enable:
            self.mode = 1

        while not rospy.is_shutdown():

            if fake_mode_enable:
                fake_mode_counter += 1
                if fake_mode_counter < 200:
                    self.mode = 1
                elif fake_mode_counter < 400:
                    self.mode = 2
                elif fake_mode_counter < 600:
                    self.mode = 3
                else:
                    self.mode = 4

            # Image Process Algorithms run here

            # 1. Detect only parking apron with usb cam downside
            if self.mode == 1:
                # Flags to change
                self.zed_color_required = 0
                self.zed_depth_required = 0
                self.usb_cam1_required = 0

                if self.usb_cam0_required == 2:
                    print "Detecting parking apron!"
                    # Detect and publish here

                    cv2.imshow("usb0", self.usb_cam0_img)
                    cv2.waitKey(10)

                    x, y = self.findDownRoI(self.usb_cam0_img)

                    if x != None and y != None:
                        self.result.pose.position.x = x
                        self.result.pose.position.y = -y
                        self.result.pose.position.z = 0

                        self.result.pose.orientation.x = 1  # landing board
                        self.result.pose.orientation.y = 0  # circle
                        self.result.pose.orientation.z = 0  # tree
                        self.result.pose.orientation.w = 0  # qr

                        self.result_pub.publish(self.result)  # publish

                    self.usb_cam0_required = 1
                else:
                    self.usb_cam0_required = 1

            # 2. Detect parking apron and circle board with zed and front down camera1
            elif self.mode == 2:
                # Flags to change
                self.usb_cam0_required = 0

                # Detect circle board
                if self.zed_depth_required == 2: #self.zed_color_required == 2 and
                    print "Detecting circle board!"

                    cv2.imshow("depth_raw", self.zed_depth_img)
                    cv2.imshow("color_raw", self.zed_color_img)
                    cv2.waitKey(10)

                    # TEST !!!!
                    x, y, z = self.findFrontRoI(self.zed_color_img, self.zed_depth_img)

                    if x != None and y != None:
                        self.result.pose.position.x = x  # to enu
                        self.result.pose.position.y = z
                        self.result.pose.position.z = -y

                        self.result.pose.orientation.x = 0  # landing board
                        self.result.pose.orientation.y = 1  # circle
                        self.result.pose.orientation.z = 0  # tree
                        self.result.pose.orientation.w = 0  # qr

                        self.result_pub.publish(self.result)  # publish

                    self.zed_color_required = 1
                    self.zed_depth_required = 1
                else:
                    self.zed_color_required = 1
                    self.zed_depth_required = 1


                # Detect parking apron
                if self.usb_cam1_required == 2:
                    print "Detecting parking apron!"
                    # Test
                    cv2.imshow("usb1", self.usb_cam1_img)
                    cv2.waitKey(10)

                    x, y = self.findFrontDownRoI(self.usb_cam1_img)

                    if x != None and y != None:
                        self.result.pose.position.x = x  # to enu
                        self.result.pose.position.y = -y
                        self.result.pose.position.z = 0

                        self.result.pose.orientation.x = 0  # landing board
                        self.result.pose.orientation.y = 1  # circle
                        self.result.pose.orientation.z = 0  # tree
                        self.result.pose.orientation.w = 0  # qr

                        self.result_pub.publish(self.result)  # publish

                    self.usb_cam1_required = 1
                else:
                    self.usb_cam1_required = 1

                print "TODO"

            # 3. Detect circle board
            elif self.mode == 3:
                # Flags to change
                self.usb_cam0_required = 0
                self.usb_cam1_required = 0

                if self.zed_color_required == 2 and self.zed_depth_required == 2:
                    print "Detecting circle board!"

                    x, y, z = self.findFrontRoI(self.zed_color_img, self.zed_depth_img)

                    if x != None and y != None:
                        self.result.pose.position.x = x  # to enu
                        self.result.pose.position.y = z
                        self.result.pose.position.z = -y

                        self.result.pose.orientation.x = 0  # landing board
                        self.result.pose.orientation.y = 1  # circle
                        self.result.pose.orientation.z = 0  # tree
                        self.result.pose.orientation.w = 0  # qr

                        self.result_pub.publish(self.result)  # publish

                    self.zed_color_required = 1
                    self.zed_depth_required = 1
                else:
                    self.zed_color_required = 1
                    self.zed_depth_required = 1

            # 4. Detect pillar and QR code
            elif self.mode == 4:
                # Flags to change
                self.usb_cam0_required = 0
                self.usb_cam1_required = 0

                if self.zed_depth_required == 2: # self.zed_color_required == 2 and
                    print "Detecting pillar!"

                    # tic = time.time()
                    x_c, z = self.findPillar(self.zed_depth_img)
                    # toc = time.time()
                    # print "z" + str(z) + "time=" + str(toc - tic)
                    # TODO QRCODE, DISTANCE

                    if z != None and x_c != None:
                        self.result.pose.position.x = x_c
                        self.result.pose.position.y = z
                        self.result.pose.position.z = 0

                        self.result.pose.orientation.x = 0  # landing board
                        self.result.pose.orientation.y = 0  # circle
                        self.result.pose.orientation.z = 1  # tree
                        self.result.pose.orientation.w = 0  # qr

                        self.result_pub.publish(self.result) #publish


                    self.zed_color_required = 1
                    self.zed_depth_required = 1
                else:
                    #self.zed_color_required = 1
                    self.zed_depth_required = 1

            else:
                self.usb_cam0_required = 0
                self.usb_cam1_required = 0
                self.zed_color_required = 0
                self.zed_depth_required = 0

                print "Do nothing!"

            r.sleep()


    def usbCam0Read(self):

        r0 = rospy.Rate(10)  #hz
        # Camera parameters
        cap0 = cv2.VideoCapture(0)
        cap0.set(3, 640)
        cap0.set(4, 480)

        while not rospy.is_shutdown():
            # Read USB image if needed
            if self.usb_cam0_required == 1:
                ret, cv_image = cap0.read()
                if ret:
                    self.usb_cam0_img = cv_image
                    self.usb_cam0_required = 2

            r0.sleep()

    def usbCam1Read(self):

        r1 = rospy.Rate(10)  # 30hz
        # Camera parameters
        cap1 = cv2.VideoCapture(1)
        cap1.set(3, 640)
        cap1.set(4, 480)

        while not rospy.is_shutdown():
            # Read USB image if needed
            if self.usb_cam1_required == 1:
                ret, cv_image = cap1.read()
                if ret:
                    self.usb_cam1_img = cv_image
                    self.usb_cam1_required = 2
                    #print "lalalala"

            r1.sleep()

    def depthCallback(self, data):

        if self.zed_depth_required == 1:
            try:
                self.zed_depth_img = self.bridge.imgmsg_to_cv2(data, "passthrough")
            except CvBridgeError as e:
                print(e)
                return
            self.zed_depth_required = 2

    def colorCallback(self, data):
        if 1: # self.zed_color_required == 1:
            try:
                self.zed_color_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)
                return
            self.zed_color_required = 2

    def poseCallBack(self, pose):

        self.height = pose.pose.position.z
        self.height_received = 1

        att_x = pose.pose.orientation.x
        att_y = pose.pose.orientation.y
        att_z = pose.pose.orientation.z
        att_w = pose.pose.orientation.w

        self.yaw = np.arctan2(2 * att_y * att_x - 2 * att_z * att_w, -2 * att_y * att_y - 2 * att_w * att_w + 1) + np.pi
        self.pitch = np.arcsin(2 * att_z * att_x - 2 * att_y * att_w)
        self.roll = np.arctan2(2 * att_x * att_w + 2 * att_y * att_z, 1 - 2 * att_y * att_y - 2 * att_x * att_x)


    def findFrontDownCenter(self, box, img_set):
        """
        This funciton is to refine the box location
        @params:
        box is the lcoation of roi, box = [x0,y0,w0,h0],from coarse detection
        img_set is the input image
        """

        [x1, y1, w1, h1] = box
        if x1 - w1 > 0:
            x_l = int(x1 - w1)
        else:
            x_l = 0

        if x1 - w1 * 2 < img_set.shape[1]:
            x_r = int(x1 + w1 * 2)
        else:
            x_r = img_set.shape[1]

        img_roi = img_set[y1:y1 + h1, x_l:x_r, :]  # !!!!!! May be wrong
        # plt.imshow(img_roi)
        # plt.show()
        img_roi_hsv = cv2.cvtColor(img_roi, cv2.COLOR_BGR2HSV)
        low_yellow = np.array([15, 60, 100])
        high_yellow = np.array([35, 250, 255])
        img_roi_hsv_bw = cv2.inRange(img_roi_hsv, low_yellow, high_yellow)
        # plt.imshow(img_roi_hsv_bw)
        # plt.show()

        ke = np.ones((5, 5))
        img_hsv_bw_erd = cv2.erode(img_roi_hsv_bw, ke, iterations=2)
        kd = np.ones((10, 10))
        img_hsv_bw_dit = cv2.dilate(img_hsv_bw_erd, kd)

        _, contours, _ = cv2.findContours(img_hsv_bw_dit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours)>0:
            max_con = contours[0]
            for con in contours:
                if cv2.contourArea(con) > cv2.contourArea(max_con):
                    max_con = con
            x0, y0, w0, h0 = cv2.boundingRect(max_con)
            #roi_box = [x0, y0, w0, h0]
            roi_box = [x0 + x_l, y0, w0, h0]  # consider the location change double frame

            return roi_box

        else:
            return box


    def findFrontDownRoI(self, img):
        """
        This function is to find the yellow region
        name is the file name(.jpg,.png)

        if you want to intergrate into program,just comment imread()line
        and CHANGE the argv form name as a Mat img, like this:
        c_x,c_y = findFrontDownRoI(img)
        they are the center location.

        """
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # plt.imshow(img_hsv[:,:,1])

        # Find yellow area in HSV
        low_yellow = np.array([15, 90, 90])
        high_yellow = np.array([35, 250, 255])
        img_hsv_bw = cv2.inRange(img_hsv, low_yellow, high_yellow)
        # plt.imshow(img_hsv_bw)
        # plt.show()
        ke = np.ones((5, 5))
        img_hsv_bw_erd = cv2.erode(img_hsv_bw, ke, iterations=3)
        kd = np.ones((10, 10))
        img_hsv_bw_dit = cv2.dilate(img_hsv_bw_erd, kd)
        # plt.imshow(img_hsv_bw_dit)   #remove small part regions
        # plt.show()

        _, contours, _ = cv2.findContours(img_hsv_bw_dit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours)>0:
            max_con = contours[0]
            for con in contours:
                if cv2.contourArea(con) > cv2.contourArea(max_con):
                    max_con = con
            x0, y0, w0, h0 = cv2.boundingRect(max_con)

        else:
            return None, None

        box = [x0, y0, w0, h0]
        roi_box = self.findFrontDownCenter(box, img)  # second detection function

        [x, y, w, h] = roi_box[0], y0 + roi_box[1], roi_box[2], roi_box[3]  # roi is corrected in upper function
        # print x

        img_set = img.copy()
        cv2.rectangle(img_set, (x, y), (x + w, y + h), (255, 255, 0), 10)  # plot bounding box
        cv2.circle(img_set, (x + int(w / 2.0), y + int(h / 2.0)), 10, (255, 0, 255), 10)
        # plt.imshow(img_set)
        # plt.show()
        cv2.imshow("origin", img_set)
        cv2.waitKey(10)

        c_x = x + int(w / 2.0)
        c_y = y + int(h / 2.0)
        return c_x, c_y


    def findDownRoI(self, img):
        """
        This function is to find the yellow region
        name is the file name(.jpg,.png)

        if you want to intergrate into program,just comment imread()line
        and CHANGE the argv form name as a Mat img, like this:
        c_x,c_y = findFrontDownRoI(img)
        they are the center location.

        """

        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # plt.imshow(img_hsv[:,:,1])

        # Find yellow area in HSV
        low_yellow = np.array([15, 90, 90])
        high_yellow = np.array([35, 250, 255])
        img_hsv_bw = cv2.inRange(img_hsv, low_yellow, high_yellow)
        # plt.imshow(img_hsv_bw)
        # plt.show()
        ke = np.ones((5, 5))
        img_hsv_bw_erd = cv2.erode(img_hsv_bw, ke, iterations=3)
        kd = np.ones((10, 10))
        img_hsv_bw_dit = cv2.dilate(img_hsv_bw_erd, kd)
        # plt.imshow(img_hsv_bw_dit)   #remove small part regions
        # plt.show()

        _, contours, _ = cv2.findContours(img_hsv_bw_dit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours)>0:
            max_con = contours[0]
            for con in contours:
                if cv2.contourArea(con) > cv2.contourArea(max_con):
                    max_con = con
            x0, y0, w0, h0 = cv2.boundingRect(max_con)


        else:
            return None, None
            print "No result"

        box = [x0, y0, w0, h0]
        roi_box = box  # second detection function

        [x, y, w, h] = roi_box[0], roi_box[1], roi_box[2], roi_box[3]  # roi is corrected in upper function
        # print x

        img_set = img.copy()
        cv2.rectangle(img_set, (x, y), (x + w, y + h), (255, 255, 0), 10)  # plot bounding box
        cv2.circle(img_set, (x + int(w / 2.0), y + int(h / 2.0)), 10, (255, 0, 255), 10)
        # plt.imshow(img_set)
        # plt.show()
        cv2.imshow("origin", img_set)
        cv2.waitKey(10)

        c_x = x + int(w / 2.0)
        c_y = y + int(h / 2.0)
        return c_x, c_y


    def findFrontRoI(self, img, depth_img):
        """
        This function is to find the yellow region
        name is the file name(.jpg,.png)

        if you want to intergrate into program,just comment imread()line
        and CHANGE the argv form name as a Mat img, like this:
        c_x,c_y = findFrontDownRoI(img)
        they are the center location.

        """

        #Image I/O
        depth = depth_img.copy()
        depth[np.where(depth > 10)] = 0
        depth = depth / 10.0 * 255
        depth = depth.astype(np.uint8)

        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # plt.imshow(img_hsv[:,:,1])

        # Find yellow area in HSV
        low_yellow = np.array([15, 90, 90])
        high_yellow = np.array([35, 250, 255])
        img_hsv_bw = cv2.inRange(img_hsv, low_yellow, high_yellow)
        # plt.imshow(img_hsv_bw)
        # plt.show()
        ke = np.ones((5, 5))
        img_hsv_bw_erd = cv2.erode(img_hsv_bw, ke, iterations=3)
        kd = np.ones((10, 10))
        img_hsv_bw_dit = cv2.dilate(img_hsv_bw_erd, kd)
        # plt.imshow(img_hsv_bw_dit)   #remove small part regions
        # plt.show()

        _, contours, _ = cv2.findContours(img_hsv_bw_dit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours)>0:
            max_con = contours[0]
            for con in contours:
                if cv2.contourArea(con) > cv2.contourArea(max_con):
                    max_con = con
            x0, y0, w0, h0 = cv2.boundingRect(max_con)


        else:
            return None, None, None
            print "No result"

        box = [x0, y0, w0, h0]
        roi_box = box  # second detection function
        [x, y, w, h] = roi_box[0], roi_box[1], roi_box[2], roi_box[3]  # roi is corrected in upper function
        #print x, y, w, h
        if y+h<depth.shape[0]:
            y2 = y+h
        else:
            y2 = depth.shape[0]

        if x+w<depth.shape[1]:
            x2 = x + w
        else:
            x2 = depth.shape[1]

        depth_part = depth[y:y2, x:x2].copy()
        #print x,x2,y,y2
        #depth_part[np.isnan(depth_part)] = 0
        d_d = depth_part.mean()*10/255
        cv2.rectangle(depth, (x, y), (x + w, y + h), (0, 255, 255), 10)  # plot bounding box
        cv2.imshow("depth", depth)
        cv2.imshow("part", depth_part)
        cv2.waitKey(10)


        img_set = img.copy()
        cv2.rectangle(img_set, (x, y), (x + w, y + h), (255, 255, 0), 10)  # plot bounding box
        cv2.circle(img_set, (x + int(w / 2.0), y + int(h / 2.0)), 10, (255, 0, 255), 10)
        # plt.imshow(img_set)
        # plt.show()
        cv2.imshow("origin", img_set)
        cv2.waitKey(10)

        c_x = x + int(w / 2.0)
        c_y = y + int(h / 2.0)

        fx = 700.5    # focus distance in meter
        fy = 700.5
        cx = 645.0
        cy = 361.3
        d_x = (c_x - cx) * d_d / fx
        d_y = (c_y - cy) * d_d / fy

        #return c_x, c_y, d_d
        return d_x, d_y, d_d

    def findPillar(self, depth_img):

        depth = depth_img.copy()
        depth[np.where(depth > 10)] = 0
        depth = depth / 10.0 * 255
        depth = depth.astype(np.uint8)

        # Image I/O to find a good pillar
        blur = cv2.medianBlur(depth, 15)
        _, bw = cv2.threshold(blur, 40, 255, cv2.THRESH_BINARY_INV)
        kernel = np.ones((13, 13), np.uint8)
        erosion = cv2.erode(bw, kernel, iterations=3)
        center = int(depth.shape[0] / 2.0)
        rows_1 = erosion[center + 50:center + 100, :]
        rows_1 = cv2.dilate(rows_1, kernel, iterations=5)

        # refine a pillar and get the depth curve
        rows_1[:, 0:rows_1.shape[1] / 5] = 0
        rows_1[:, rows_1.shape[1] * 4 / 5:rows_1.shape[1]] = 0
        blur = cv2.medianBlur(rows_1, 11)
        line_1 = blur.sum(axis=0)

        win_c = int(line_1.shape[0] / 2.0)
        pillar = np.where(line_1[win_c - 200:win_c + 200] > line_1.max() * 0.8)
        if pillar != None:
            if len(pillar) > 0 and len(pillar[0]) > 0:
                try:
                    p_c = int((pillar[0][0] + win_c - 200 + pillar[0][-1] + win_c - 200) / 2.0)
                    if pillar[0][-1] - pillar[0][0] > 50:
                        win_c = p_c
                        pillar2 = np.where(line_1[win_c - 200:win_c + 200] > line_1.max() * 0.8)
                        cv2.rectangle(depth, (pillar2[0][0] + win_c - 200, center + 50),
                                      (pillar2[0][-1] + win_c - 200, center + 100), (0, 255, 0), 10)
                        # cv2.rectangle(depth,(pillar[0][0]+win_c-200, center+50),(pillar[0][-1]+win_c-200, center+100),(0,255,0),10)
                        cv2.imshow('depth', depth)
                        cv2.waitKey(10)
                        patch = depth[center + 50:center + 100,
                                pillar[0][0] + win_c - 200:pillar[0][-1] + win_c - 200]
                        #return pillar[0][0] + win_c - 200, pillar[0][-1] + win_c - 200, patch.mean() / 255 * 10
                        pillar_cd = int((pillar[0][0] + win_c - 200 + pillar[0][-1] + win_c - 200) / 2.0) - depth_img.shape[1] / 2
                        pillar_w = pillar[0][-1] - pillar[0][0]  # pillar wideness in pixel
                        pillar_delat = pillar_cd*0.3/pillar_w   # 0.3 is the pillar wideness in meter
                        return pillar_delat, patch.mean() / 255 * 10

                    else:
                        print "not in center and wide enough"
                        return None, None  #, None

                except Exception as e:
                    # plt.plot(line_1)
                    # plt.show()
                    print " not in center"
                    print e
                    return None, None   #, None
            else:
                print "not find a peak"
                return None, None  #, None


if __name__ == '__main__':

    rospy.init_node('optical_flow')

    multi_cam_process = MultiCamProcess()
