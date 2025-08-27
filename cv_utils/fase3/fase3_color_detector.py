import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class Fase3ColorDetector(Node):
    def __init__(self):
        super().__init__('fase3_color_detector')
        
        # Declare parameters
        self.declare_parameter('image_topic', '/vertical_camera')
        self.declare_parameter('area_minima_bases', 2000)
        
        # Debug mode parameters - similar to CBR base_detector
        self.declare_parameter('full_debug_mode', False)
        self.declare_parameter('light_debug_mode', False)
        self.declare_parameter('light_debug_topic', '/telemetry/camera_debug/compressed')
        self.declare_parameter('light_debug_size', 400)  # Resize debug images for bandwidth
        self.declare_parameter('light_debug_quality', 75)  # JPEG compression quality (75% for good quality/size balance)
        
        # Frame counting for debug publishing rate control (3Hz instead of camera rate)
        self.frame_count = 0
        self.debug_frame_skip = 3  # Publish debug every 3th frame (10Hz -> 3.33Hz)
        
        # Declare HSV parameters for each shape
        self.shape_names = ['circulo', 'triangulo', 'hexagono', 'hexagono2', 'pentagono', 
                           'estrela', 'cruz', 'casa', 'quadrado', 'quadrado2']
        
        for shape in self.shape_names:
            self.declare_parameter(f'{shape}.hue_lower', 0)
            self.declare_parameter(f'{shape}.hue_upper', 180)
            self.declare_parameter(f'{shape}.saturation_lower', 0)
            self.declare_parameter(f'{shape}.saturation_upper', 255)
            self.declare_parameter(f'{shape}.value_lower', 0)
            self.declare_parameter(f'{shape}.value_upper', 255)
        
        # Get parameter values
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        
        # Get debug mode parameters
        self.full_debug_mode = self.get_parameter('full_debug_mode').get_parameter_value().bool_value
        self.light_debug_mode = self.get_parameter('light_debug_mode').get_parameter_value().bool_value
        self.light_debug_topic = self.get_parameter('light_debug_topic').get_parameter_value().string_value
        self.light_debug_size = self.get_parameter('light_debug_size').get_parameter_value().integer_value
        self.light_debug_quality = self.get_parameter('light_debug_quality').get_parameter_value().integer_value
        
        # Load HSV ranges from ROS2 parameters
        self.hsv_ranges = {}
        for shape in self.shape_names:
            self.hsv_ranges[shape] = {
                'hue_lower': self.get_parameter(f'{shape}.hue_lower').get_parameter_value().integer_value,
                'hue_upper': self.get_parameter(f'{shape}.hue_upper').get_parameter_value().integer_value,
                'saturation_lower': self.get_parameter(f'{shape}.saturation_lower').get_parameter_value().integer_value,
                'saturation_upper': self.get_parameter(f'{shape}.saturation_upper').get_parameter_value().integer_value,
                'value_lower': self.get_parameter(f'{shape}.value_lower').get_parameter_value().integer_value,
                'value_upper': self.get_parameter(f'{shape}.value_upper').get_parameter_value().integer_value,
            }
        
        self.get_logger().info(f'Loaded HSV ranges for shapes: {list(self.hsv_ranges.keys())}')
        
        self.bridge = CvBridge()
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriber = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile
        )
        
        # Publishers for debug images
        if self.full_debug_mode:
            self.image_pub = self.create_publisher(Image, '/fase3_color_detector/image', 10)
            
        # Telemetry debug publisher for compressed images at 3Hz
        if self.light_debug_mode:
            self.telemetry_debug_pub = self.create_publisher(
                CompressedImage,
                self.light_debug_topic,
                10
            )
        
        # Para envio das info de deteccao para o ros2
        self.classification_topic = "/vertical_camera/classification"
        self.classification_publisher = self.create_publisher(Detection2DArray, self.classification_topic, 10)


    def image_callback(self, msg):
        detection_array = Detection2DArray()
        detection_array.header = msg.header
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")
            return
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        annotated = frame.copy()
        height, width = frame.shape[:2]  # Para normalização
        
        # Increment frame counter for debug rate control
        self.frame_count += 1
        for color, params in self.hsv_ranges.items():
            lower = np.array([
                params['hue_lower'],
                params['saturation_lower'],
                params['value_lower']
            ])
            upper = np.array([
                params['hue_upper'],
                params['saturation_upper'],
                params['value_upper']
            ])
            mask = cv2.inRange(hsv, lower, upper)
            # Morphology to clean up
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > self.get_parameter('area_minima_bases').get_parameter_value().integer_value:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(annotated, (x, y), (x+w, y+h), (0,255,0), 2)
                    cv2.putText(annotated, color, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

                    # Preencher Detection2D para cada base detectada (normalizado)
                    det = Detection2D()
                    det.header = msg.header
                    det.bbox = BoundingBox2D()
                    det.bbox.center.position.x = (x + w/2) / width
                    det.bbox.center.position.y = (y + h/2) / height
                    det.bbox.size_x = w / width
                    det.bbox.size_y = h / height
                    hypo = ObjectHypothesisWithPose()
                    hypo.hypothesis.class_id = color
                    hypo.hypothesis.score = 1.0  # Score fixo, pois é segmentação clássica
                    det.results.append(hypo)
                    detection_array.detections.append(det)
        
        # Publish full debug image if enabled
        if self.full_debug_mode:
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                self.image_pub.publish(annotated_msg)
            except CvBridgeError as e:
                self.get_logger().error(f"Failed to convert annotated image: {str(e)}")
        
        # Publish compressed debug image at 3Hz if light debug mode is enabled
        if self.light_debug_mode and (self.frame_count % self.debug_frame_skip == 0):
            try:
                # Create telemetry debug image with additional info
                telemetry_debug = self._create_telemetry_debug_image(annotated, detection_array, msg.header)
                
                # Convert to compressed image
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header
                compressed_msg.format = "jpeg"
                
                # Encode with specified quality
                encode_param = [cv2.IMWRITE_JPEG_QUALITY, self.light_debug_quality]
                _, encoded_img = cv2.imencode('.jpg', telemetry_debug, encode_param)
                compressed_msg.data = encoded_img.tobytes()
                
                self.telemetry_debug_pub.publish(compressed_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to create compressed debug image: {str(e)}")
        
        # Publicar detections
        self.classification_publisher.publish(detection_array)

    def _create_telemetry_debug_image(self, annotated_image: np.ndarray, detection_array, header) -> np.ndarray:
        """
        Create optimized debug image for telemetry with shape detection info.
        Resized to reduce bandwidth usage.
        
        Args:
            annotated_image: Image with bounding boxes and labels
            detection_array: Detection2DArray with detection results
            header: ROS header for timestamp info
            
        Returns:
            Resized debug image for telemetry
        """
        # Resize image to reduce bandwidth
        height, width = annotated_image.shape[:2]
        if width > self.light_debug_size or height > self.light_debug_size:
            # Calculate aspect-ratio preserving resize
            scale = min(self.light_debug_size / width, self.light_debug_size / height)
            new_width = int(width * scale)
            new_height = int(height * scale)
            telemetry_debug = cv2.resize(annotated_image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
        else:
            telemetry_debug = annotated_image.copy()
        
        # Add telemetry-specific information
        debug_height, debug_width = telemetry_debug.shape[:2]
        
        # Add timestamp and frame info
        timestamp_text = f"Fase3ColorDetector - {header.stamp.sec}.{header.stamp.nanosec//1000000:03d}"
        cv2.putText(telemetry_debug, timestamp_text, (10, debug_height - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        
        # Add detection count
        detection_count = len(detection_array.detections)
        detection_count_text = f"Detections: {detection_count}"
        cv2.putText(telemetry_debug, detection_count_text, (10, debug_height - 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        
        # Add processing resolution info
        res_text = f"Src: {width}x{height} -> {debug_width}x{debug_height}"
        cv2.putText(telemetry_debug, res_text, (10, debug_height - 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        
        # Add quality info
        quality_text = f"Quality: {self.light_debug_quality}% JPEG"
        cv2.putText(telemetry_debug, quality_text, (10, debug_height - 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        
        return telemetry_debug


def main(args=None):
    rclpy.init(args=args)
    node = Fase3ColorDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
