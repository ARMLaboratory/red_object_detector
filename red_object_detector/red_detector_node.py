import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class RedObjectDetector(Node):
    def __init__(self):
        super().__init__('red_object_detector')

        self.declare_parameter('color_topic', '/camera/color/image_rect_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')

        color_topic = self.get_parameter('color_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value

        self.bridge = CvBridge()

        # Publishers
        self.point_pub = self.create_publisher(Point, 'detected_red_object', 10)

        # Subscribers
        self.color_sub = self.create_subscription(Image, color_topic, self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        self.color_image = None
        self.depth_image = None
        self.intrinsics = None

    def camera_info_callback(self, msg):
        if self.intrinsics is None:
            self.intrinsics = {
                'fx': msg.k[0],
                'fy': msg.k[4],
                'cx': msg.k[2],
                'cy': msg.k[5]
            }
            self.get_logger().info('Camera intrinsics received.')

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def color_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.depth_image is not None and self.intrinsics is not None:
            self.process_images()

    def process_images(self):
        img = self.color_image.copy()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Improved Red Color Range
        lower_red1 = np.array([0, 150, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 150, 100])
        upper_red2 = np.array([180, 255, 255])

        # Create masks and clean with morphology
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Apply morphological operations to clean noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        object_count = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 1000:  # Filter small detections
                continue

            object_count += 1
            x, y, w, h = cv2.boundingRect(cnt)

            # Mask the red region in depth image
            mask_roi = mask[y:y+h, x:x+w]
            depth_roi = self.depth_image[y:y+h, x:x+w]

            # Extract depth where red mask is active
            red_depths = depth_roi[mask_roi > 0]
            red_depths = red_depths[red_depths > 0]

            if len(red_depths) == 0:
                continue

            median_depth = np.median(red_depths)

            u = x + w // 2
            v = y + h // 2
            z = median_depth / 1000.0  # from mm to meters
            x_real = (u - self.intrinsics['cx']) * z / self.intrinsics['fx']
            y_real = (v - self.intrinsics['cy']) * z / self.intrinsics['fy']

            # Publish (X, Y, Z) Point
            point_msg = Point()
            point_msg.x = x_real
            point_msg.y = y_real
            point_msg.z = z
            self.point_pub.publish(point_msg)

            # Draw bounding box and coordinates
            cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
            text = f"X:{x_real:.2f}m Y:{y_real:.2f}m Z:{z:.2f}m"
            cv2.putText(img, text, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # Show object count
        cv2.putText(img, f"Red Objects: {object_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

        cv2.imshow('Red Object Detection', img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RedObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

