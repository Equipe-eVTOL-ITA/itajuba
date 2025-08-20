import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from custom_msgs.msg import LaneDirection

import cv2
import numpy as np
import math
from cv_bridge import CvBridge

class LaneDetector(Node):
	def __init__(self):
		super().__init__('lane_detector')

		self._publisher = self.create_publisher(LaneDirection, 'lane_detection', 10)

		self._subscriber = self.create_subscription(
			CompressedImage,
			'/vertical_camera/compressed',
			self.lane_detection_callback,
			10
		)
		self.bridge = CvBridge()
	
	def lane_detection_callback(self, msg):
		np_arr = np.frombuffer(msg.data, np.uint8)
		cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

		if cv_image is None:
			self.get_logger().error('Failed to decode compressed color image.')
			return

		# Inicializar variáveis com valores padrão
		theta = 0.0
		cx = 0
		cy = 0

		hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

		lower_blue = np.array([100, 150, 50])
		upper_blue = np.array([140, 255, 255])

		binary_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

		contornos, hierarquia = cv2.findContours(
            binary_mask,
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE
        )

		output_image = cv_image.copy()
		
		# Obter dimensões da imagem para normalização
		image_height, image_width = cv_image.shape[:2]
		center_x = image_width / 2.0
		center_y = image_height / 2.0

		if contornos:
			# filtra por tamanho (a faixa deve ser o maior contorno na imagem)
			maior_contorno = max(contornos, key=cv2.contourArea)
			area = cv2.contourArea(maior_contorno)

			if area > 200:
				cv2.drawContours(output_image, [maior_contorno], 0, (0, 255, 0), 3)

				# técnica dos momentos de imagem
				M = cv2.moments(maior_contorno)
				A = M['m00']
				if A > 0:
					# centroide em pixels (sistema original)
					cx_pixels = int(M['m10']/A)
					cy_pixels = int(M['m01']/A)
					
					# Normalizar coordenadas em relação ao centro da imagem
					# Valores entre -1 e +1, onde (0,0) é o centro da imagem
					cx = (cx_pixels - center_x) / center_x
					cy = (cy_pixels - center_y) / center_y
					
					# Para visualização, ainda usar coordenadas em pixels
					cv2.circle(output_image, (cx_pixels, cy_pixels), 7, (0, 0, 255), -1)

					# orientacao
					mu20 = M['mu20']
					mu02 = M['mu02']
					mu11 = M['mu11']
					theta = 0.5*math.atan2(2*mu11, mu20 - mu02)

					# desenha a seta da orientação (usando coordenadas em pixels para visualização)
					tamanho = 100
					start_point = (cx_pixels, cy_pixels)
					end_point = (
						int(cx_pixels + tamanho*math.cos(theta)),
						int(cy_pixels + tamanho*math.sin(theta))
					)
					cv2.arrowedLine(output_image, start_point, end_point, (0, 0, 255), 3)
					
					# Adicionar texto com informações normalizadas para debug
					info_text = f"Norm: ({cx:.2f}, {cy:.2f}), Theta: {math.degrees(theta):.1f}°"
					cv2.putText(output_image, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
				else:
					self.get_logger().warn('Contorno detectado mas momento zero é inválido')
			else:
				self.get_logger().debug(f'Contorno muito pequeno: área = {area}')

		# Publicar sempre, mesmo que não haja detecção válida
		# Agora cx e cy são valores normalizados entre -1 e +1
		lane_msg = LaneDirection()
		lane_msg.theta = float(theta)
		lane_msg.x_centroid = int(cx * 1000)  # Multiplicar por 1000 para manter precisão como int
		lane_msg.y_centroid = int(cy * 1000)  # Dividir por 1000 no lado C++ para obter float
		
		print(f"Lane Direction - Theta: {lane_msg.theta:.2f}, X: {lane_msg.x_centroid}, Y: {lane_msg.y_centroid}")

		self._publisher.publish(lane_msg)

		# Opcional: mostrar imagens para debug (descomente se necessário)
		self.show_image(binary_mask, output_image)

	def show_image(self, binary_mask, output_image):
		#cv2.imshow('mascara', binary_mask)
		cv2.imshow('Lane Detection', output_image)
		cv2.waitKey(1)


def main(args=None):
	rclpy.init(args=args)
	ld = LaneDetector()
	rclpy.spin(ld)
	ld.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()