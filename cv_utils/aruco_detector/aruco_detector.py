import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from custom_msgs.msg import ArucoMarkerMsg
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge

import cv2
import numpy as np
import math

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self._publisher = self.create_publisher(ArucoMarkerMsg, 'aruco_detection', 10)
        
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
        
        # Compatibilidade com diferentes versões do OpenCV
        try:
            # OpenCV 4.7+ com ArucoDetector
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.dicionario, self.aruco_params)
            self.use_new_api = True
        except AttributeError:
            # OpenCV 4.6 e anteriores
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.use_new_api = False

        # self.calibrate_camera_real()  # Para uso com câmera real
        self.calibrate_camera_sim()     # Para uso em simulação



    def calibrate_camera_real(self):
        # Parâmetros reais (exemplo, troque pelos calibrados da sua câmera física)
        self.camera_matrix = np.array([[921.1707, 0, 459.9043],
                                       [0, 919.0183, 351.2383],
                                       [0, 0, 1]])
        self.dist_coeffs = np.array([-0.033458, 0.105152, 0.001256, -0.004597, 0])

    def calibrate_camera_sim(self):
        # Parâmetros ideais para simulação (sem distorção)
        width = 800  # ajuste conforme o SDF da câmera vertical
        height = 800
        fov = 1.047  # horizontal_fov 1.047 ajuste conforme o SDF da sua câmera vertical
        fx = fy = 0.5 * width / np.tan(0.5 * fov)
        cx = width / 2
        cy = height / 2
        self.camera_matrix = np.array([[fx, 0, cx],
                                       [0, fy, cy],
                                       [0,  0,  1]])
        self.dist_coeffs = np.zeros(5)


    def aruco_detection_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Detecção compatível com diferentes versões do OpenCV
        if self.use_new_api:
            corners, ids, rejected = self.detector.detectMarkers(cv_image)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, self.dicionario, parameters=self.aruco_params)

        detection_msg = ArucoMarkerMsg()
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            # Checa se a função está disponível
            if hasattr(cv2.aruco, 'estimatePoseSingleMarkers'):
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.tamanho_real, self.camera_matrix, self.dist_coeffs)
            else:
                raise RuntimeError("Sua instalação do OpenCV não possui o módulo aruco completo. Instale com: pip install opencv-contrib-python")

            for i in range(len(ids)):
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.03)

                detection_msg.ids.append(int(ids[i][0]))

                pose_msg = Pose()
                pose_msg.position.x = float(tvecs[i][0][0])
                pose_msg.position.y = float(tvecs[i][0][1])
                pose_msg.position.z = float(tvecs[i][0][2])
                # Para orientação, converter rvec para quaternion
                rot_matrix, _ = cv2.Rodrigues(rvecs[i][0])
                qw = math.sqrt(max(0.0, 1.0 + rot_matrix[0,0] + rot_matrix[1,1] + rot_matrix[2,2])) / 2.0
                qx = (rot_matrix[2,1] - rot_matrix[1,2]) / (4.0 * qw)
                qy = (rot_matrix[0,2] - rot_matrix[2,0]) / (4.0 * qw)
                qz = (rot_matrix[1,0] - rot_matrix[0,1]) / (4.0 * qw)
                pose_msg.orientation.x = qx
                pose_msg.orientation.y = qy
                pose_msg.orientation.z = qz
                pose_msg.orientation.w = qw
                detection_msg.poses.append(pose_msg)

                detection_msg.confidences.append(1.0)

                pts = corners[i][0].astype(int)
                cv2.polylines(cv_image, [pts], isClosed=True, color=(0,255,0), thickness=2)

                cX = int(np.mean(pts[:,0]))
                cY = int(np.mean(pts[:,1]))

                cv2.putText(cv_image, f"ID:{ids[i][0]}", (cX-20, cY-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
                cv2.circle(cv_image, (cX, cY), 4, (255,0,0), -1)
                cv2.putText(cv_image, f"({cX},{cY})", (cX+10, cY+10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,0,0), 1)

        # Exibir imagem com OpenCV
        cv2.imshow("Aruco Detection", cv_image)
        cv2.waitKey(1)

        self._publisher.publish(detection_msg)

def main(args=None):
    rclpy.init(args=args)
    ad = ArucoDetector()
    rclpy.spin(ad)
    ad.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()