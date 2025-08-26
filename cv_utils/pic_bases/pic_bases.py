#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
from datetime import datetime

class PicBasesNode(Node):
    def __init__(self):
        super().__init__('pic_bases')
        
        self.declare_parameter('save_folder', '/tmp/bases_detected')
        self.save_folder = self.get_parameter('save_folder').get_parameter_value().string_value
        os.makedirs(self.save_folder, exist_ok=True)
        self.get_logger().info(f'Salvando as imagens em: {self.save_folder}')

        self.declare_parameter('image_topic', '/base_detector/image')
        self.image = None

        self.declare_parameter('counter_topic', '/fase1_vision/counter')
        self.count = -1  # Começar com -1 para capturar a primeira base
        

        self.bridge = CvBridge()


        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.counter_sub = self.create_subscription(
            Int32,
            self.get_parameter('counter_topic').get_parameter_value().string_value,
            self.counter_callback,
            10
        )
        
        self.image_sub = self.create_subscription(
            Image,
            self.get_parameter('image_topic').get_parameter_value().string_value,
            self.image_callback,
            qos_profile
        )



    def counter_callback(self, msg):
        current_count: int = msg.data

        if current_count > self.count:
            self.count = current_count
            self.get_logger().info(f'New base detected! Count: {current_count}')
            self.save_current_image()

    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')
            return

    def save_current_image(self):

        if self.image is None:
            self.get_logger().warning('No image available to save.')
            return
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        info_text = f"Base #{self.count} - {timestamp}"
        cv2.putText(self.image, info_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        
        # Salvar imagem com nome único
        filename = f"base_{self.count:03d}_{timestamp}.jpg"
        filepath = os.path.join(self.save_folder, filename)
        
        try:
            cv2.imwrite(filepath, self.image)
            self.get_logger().info(f'Saved base image: {filepath}')
        except Exception as e:
            self.get_logger().error(f'Failed to save image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PicBasesNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down PicBases node...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
