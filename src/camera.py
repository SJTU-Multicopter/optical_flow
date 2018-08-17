#!/usr/bin/env python

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

if __name__ == '__main__':
    rospy.init_node("camera")

    image_pub = rospy.Publisher("/opt_image", Image, queue_size=1)

    cap = cv2.VideoCapture(1)
    cap.set(3, 640)
    cap.set(4, 480)

    rate = rospy.Rate(120)  # 120hz max
    bridge = CvBridge()

    while not rospy.is_shutdown():

        # get a frame
        ret, frame = cap.read()
        # publish a frame
        if ret:
            img = bridge.cv2_to_imgmsg(frame, "bgr8")
            img.header.stamp = rospy.Time.now()
            image_pub.publish(img)

        # show a frame
        # cv2.imshow("capture", frame)
        # if cv2.waitKey(5) & 0xFF == ord('q'):
        #     break

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()
