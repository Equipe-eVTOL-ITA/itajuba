import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from custom_msgs.msg import ArucoDetection
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge

import cv2
import numpy as np
import math

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self._publisher = self.create_publisher(ArucoDetection, 'aruco_detection', 10)
        
        self._subscriber = self.create_subscription(
            CompressedImage,
            '/vertical_camera/compressed',
            self.aruco_detection_callback,
            10
        )
        self.bridge = CvBridge()

        # definindo o dicionario de marcadores
        self.dicionario = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.tamanho_real = 0.7 # lado do quadrado preto em metros
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dicionario, self.aruco_params)

        self.calibrate_camera()


    def calibrate_camera(self):
        # (Trocar pela calibracao da camera: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
        self.camera_matrix = np.array([[921.1707, 0, 459.9043],
                                        [0, 919.0183, 351.2383],
                                        [0, 0, 1]])
        self.dist_coeffs = np.array([-0.033458, 0.105152, 0.001256, -0.004597, 0])


    def aruco_detection_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        corners, ids, rejected = self.detector.detectMarkers(cv_image)

        detection_msg = ArucoDetection()
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.tamanho_real, self.camera_matrix, self.dist_coeffs)

            for i in range(len(ids)):
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.03)

                detection_msg.ids.append(int(ids[i][0]))

                pose_msg = Pose()
                pose_msg.position.x = float(tvecs[i][0][0])
                pose_msg.position.y = float(tvecs[i][0][1])
                pose_msg.position.z = float(tvecs[i][0][2])
                # Para orientação, converter rvec para quaternion
                rot_matrix, _ = cv2.Rodrigues(rvecs[i][0])
                qw = math.sqrt(1.0 + rot_matrix[0,0] + rot_matrix[1,1] + rot_matrix[2,2]) / 2.0
                qx = (rot_matrix[2,1] - rot_matrix[1,2]) / (4.0 * qw)
                qy = (rot_matrix[0,2] - rot_matrix[2,0]) / (4.0 * qw)
                qz = (rot_matrix[1,0] - rot_matrix[0,1]) / (4.0 * qw)
                pose_msg.orientation.x = qx
                pose_msg.orientation.y = qy
                pose_msg.orientation.z = qz
                pose_msg.orientation.w = qw
                detection_msg.poses.append(pose_msg)

                detection_msg.confidences.append(1.0)

        self._publisher.publish(detection_msg)
