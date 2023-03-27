
#!/usr/bin/env python3
from __future__ import print_function

import sys
import rospy
import cv2
import tf
import time
from rospy import Time 
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

distortion_coefficients = []
matrix_coefficients = []
betweenPose = PoseStamped()
postPose = PoseStamped()
mode = "start"
rot=[]
rospy.init_node('aruco_moco', anonymous=True)
class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("between_image", Image, queue_size=10)
        self.between_pub = rospy.Publisher("output_point", PoseStamped, queue_size=10)
        self.bridge = CvBridge()
        self.calibrate_camera = rospy.Subscriber("/zed/zed_node/left/camera_info", CameraInfo, self.callbackCalibrate)
        self.image_sub = rospy.Subscriber("/zed/zed_node/left_raw/image_raw_gray", Image, self.callback)

    def callback(self, data):

        global distortion_coefficients
        global matrix_coefficients,translation,rotation,rot

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image_gray = cv_image.copy()
        cv_image_gray = cv2.cvtColor(cv_image_gray, cv2.COLOR_BGR2GRAY)

        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        cv_image_gray = clahe.apply(cv_image_gray)

        cv_image_gray = cv2.medianBlur(cv_image_gray, 5)

        height, width = cv_image.shape[:2]

        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        arucoParams = cv2.aruco.DetectorParameters_create()

        (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image_gray, arucoDict, parameters=arucoParams)

        positionX = 0
        positionY = 0
        positionZ = 0

        orientationX = 0
        orientationY = 0
        orientationZ = 0
        orientationW = 0

        if len(corners) > 0:
            for i in range(0, len(ids)):

                rvec, tvec,_objPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.2, matrix_coefficients,
                                                                 distortion_coefficients)

                cv2.aruco.drawDetectedMarkers(cv_image, corners)
                cv2.aruco.drawAxis(cv_image, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.1)
                rotation_matrix = np.array([[0, 0, 0, 0],
                                            [0, 0, 0, 0],
                                            [0, 0, 0, 0],
                                            [0, 0, 0, 1]],
                                           dtype=float)
                rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvec)

                quaternion = tf.transformations.quaternion_from_matrix(rotation_matrix)

                if mode == "start":
                    self.publishPostPose(tvec[0][0][0], tvec[0][0][1], tvec[0][0][2], quaternion[0], quaternion[1],
                                         quaternion[2], quaternion[3])

                else:
                    positionX = tvec[0][0][0] + positionX
                    positionY = tvec[0][0][1] + positionY
                    positionZ = tvec[0][0][2] + positionZ

                    orientationX = quaternion[0] + orientationX
                    orientationY = quaternion[1] + orientationY
                    orientationZ = quaternion[2] + orientationZ
                    orientationW = quaternion[3] + orientationW

        cv2.imshow("Image window", cv_image)
        translation = (positionX, positionY, positionZ)
        rotation = [orientationX, orientationY, orientationZ, orientationW]
        rotation = np.array(rotation)
        # print(translation)
        # print(rotation)  tf.transformations.quaternion_from_euler(0, 0, 1.0), 
        b = TransformBroadcaster()
        rot_eul= euler_from_quaternion(rotation)
        # print(type(rot_eul))
        quat = quaternion_from_euler (2.7, 0.0,0.2)
        b.sendTransform(translation, tf.transformations.quaternion_from_euler(rot_eul[0], rot_eul[1], rot_eul[2]), Time.now(),'ar_frame','zed_left_camera_optical_frame')
        
        self.drawMarker(corners, ids, cv_image=cv_image)
        # print(type(rotation))
        if ids is not None:
            aruco_ids.publish(str(ids.tolist()))
        else:
            aruco_ids.publish(str([[]]))

    def publishPostPose(self, pX, pY, pZ, oX, oY, oZ, oW):
        global postPose
        postPose = PoseStamped()
        postPose.header.frame_id = "output_p"
        postPose.pose.position.x = pZ
        postPose.pose.position.y = pX
        postPose.pose.position.z = pY

        postPose.pose.orientation.x = oX
        postPose.pose.orientation.y = oY
        postPose.pose.orientation.z = oZ
        postPose.pose.orientation.w = oW

        self.between_pub.publish(postPose)


    def drawMarker(self, corners, ids, cv_image):
        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                cv2.line(cv_image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(cv_image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(cv_image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(cv_image, bottomLeft, topLeft, (0, 255, 0), 2)
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(cv_image, (cX, cY), 4, (0, 0, 255), -1)
                cv2.putText(cv_image, str(markerID),
                            (topLeft[0], topLeft[1] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0, 255, 0), 2)

                cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def callbackCalibrate(self, data):
        global distortion_coefficients
        global matrix_coefficients

        distortion_coefficients = []
        matrix_coefficients = []

        distortion_coefficients.append(data.D[0])
        distortion_coefficients.append(data.D[1])
        distortion_coefficients.append(data.D[2])
        distortion_coefficients.append(data.D[3])
        distortion_coefficients.append(data.D[4])

        matrix_coefficients.append([])
        matrix_coefficients[0].append(data.K[0])
        matrix_coefficients[0].append(data.K[1])
        matrix_coefficients[0].append(data.K[2])

        matrix_coefficients.append([])
        matrix_coefficients[1].append(data.K[3])
        matrix_coefficients[1].append(data.K[4])
        matrix_coefficients[1].append(data.K[5])

        matrix_coefficients.append([])
        matrix_coefficients[2].append(data.K[6])
        matrix_coefficients[2].append(data.K[7])
        matrix_coefficients[2].append(data.K[8])

        distortion_coefficients = np.array(distortion_coefficients)
        matrix_coefficients = np.array(matrix_coefficients)

        self.unsubscribe()

    def unsubscribe(self):
        self.calibrate_camera.unregister()


def main(args):
    global mode
    mode = args[0]
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    aruco_ids = rospy.Publisher('aruco_id_list', String,queue_size=2)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        main(sys.argv)


