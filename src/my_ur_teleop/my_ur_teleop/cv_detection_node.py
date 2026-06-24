import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import threading

class CVDetectionNode(Node):
    def __init__(self):
        super().__init__('cv_detection_node')
        self.bridge = CvBridge()
        
        self.current_frame = None
        self.current_depth = None
        self._frame_lock = threading.Lock()
        
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        
        self.last_published_pose = None
        self.pose_threshold = 0.005 
        
        self.color_ranges = {
            'red': {
                'lower1': np.array([0, 120, 70]), 'upper1': np.array([10, 255, 255]),
                'lower2': np.array([170, 120, 70]), 'upper2': np.array([180, 255, 255])
            }
        }
        self.target_color = 'red' 

        self.gui_timer = self.create_timer(0.033, self.gui_loop)
        self.get_logger().info("CV Node: Đang quét vật thể màu đỏ (Side-View Pinhole)...")

    def image_callback(self, msg):
        with self._frame_lock:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        with self._frame_lock:
            self.current_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def gui_loop(self):
        with self._frame_lock:
            if self.current_frame is None or self.current_depth is None:
                return
            cv_image = self.current_frame.copy()
            depth_image = self.current_depth.copy()
        
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        cfg = self.color_ranges[self.target_color]
        mask = cv2.bitwise_or(
            cv2.inRange(hsv_image, cfg['lower1'], cfg['upper1']),
            cv2.inRange(hsv_image, cfg['lower2'], cfg['upper2'])
        )
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 20: 
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    cv2.drawContours(cv_image, [largest_contour], -1, (0, 255, 0), 2)
                    cv2.circle(cv_image, (cx, cy), 5, (255, 0, 0), -1)
                    
                    height, width = depth_image.shape[:2]
                    if 0 <= cx < width and 0 <= cy < height:
                        depth_val = depth_image[cy, cx]

                        if 0.01 < depth_val < 2.0:
                            # --- MÔ HÌNH TOÁN HỌC CHO CAMERA BÊN HÔNG ---
                            fx, fy = 465.0, 465.0 
                            
                            X_c = (cx - 320) * depth_val / fx
                            Y_c = (cy - 240) * depth_val / fy
                            Z_c = depth_val

                            # Camera hiện tại đặt ở X=0.45, Y=-0.60, Z=0.10
                            robot_x = 0.45 + X_c   
                            robot_y = -0.60 + Z_c   
                            robot_z = 0.10 - Y_c   

                            self.process_and_publish(robot_x, robot_y, robot_z)
        
        cv2.imshow("Camera View", cv_image)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)

    def process_and_publish(self, x, y, z):
        if self.last_published_pose is not None:
            dx = x - self.last_published_pose[0]
            dy = y - self.last_published_pose[1]
            dz = z - self.last_published_pose[2]
            if math.sqrt(dx**2 + dy**2 + dz**2) < self.pose_threshold:
                return 
                
        self.last_published_pose = (x, y, z)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        self.pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CVDetectionNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()