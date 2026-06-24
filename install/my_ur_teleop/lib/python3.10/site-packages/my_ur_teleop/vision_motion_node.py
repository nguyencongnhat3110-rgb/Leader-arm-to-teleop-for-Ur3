import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Empty
import numpy as np
import threading

class VisionMotionNode(Node):
    def __init__(self):
        super().__init__('vision_motion_node')
        
        self._lock = threading.Lock()
        self._target_obj = None
        self._event_received = False  
        
        self.state = 'WAITING'
        self.state_start_time = 0.0
        
        self.create_subscription(PoseStamped, '/target_pose', self._on_target, 10)
        self.create_subscription(Empty, '/target_reached', self._on_target_reached, 10)
        
        self.cartesian_pub = self.create_publisher(PoseStamped, '/cartesian_target', 10)
        self.gripper_pub = self.create_publisher(Float64, '/gripper_cmd', 10)
        
        self.create_timer(0.1, self._loop)
        self.get_logger().info("Brain Node: Hệ thống điều khiển Task Planner phối hợp Sự kiện đã sẵn sàng!")

    def _on_target(self, msg: PoseStamped):
        with self._lock: 
            if self.state == 'WAITING':
                self._target_obj = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def _on_target_reached(self, msg: Empty):
        with self._lock:
            if self.state == 'DESCEND':
                self._event_received = True

    def _set_gripper(self, value):
        msg = Float64()
        msg.data = float(value)
        self.gripper_pub.publish(msg)

    def _publish_cartesian(self, target_pos: np.ndarray):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, target_pos)
        self.cartesian_pub.publish(msg)

    def _loop(self):
        with self._lock:
            target_obj = self._target_obj.copy() if self._target_obj is not None else None
            event_flag = self._event_received
            
        if target_obj is None: return
        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.state == 'WAITING':
            self._set_gripper(0.0) 
            with self._lock: self._event_received = False
            self.state = 'APPROACH'
            self.state_start_time = current_time
            self.get_logger().info('-> APPROACH: Di chuyển tới đỉnh vật thể...')

        elif self.state == 'APPROACH':
            approach = target_obj.copy()
            approach[2] += 0.20 
            self._publish_cartesian(approach)
            
            if current_time - self.state_start_time > 3.0:
                self.state = 'DESCEND'
                self.state_start_time = current_time
                self.get_logger().info('-> DESCEND: Hạ thấp trục Z, chờ phản hồi chạm mục tiêu từ MuJoCo...')

        elif self.state == 'DESCEND':
            descend = target_obj.copy()
            descend[2] += 0.02 
            self._publish_cartesian(descend)
            
            # Chuyển sang gắp nếu nhận được Sự kiện HOẶC đã chờ quá 4 giây (Timeout bảo vệ)
            if event_flag or (current_time - self.state_start_time > 4.0):
                self.state = 'GRASP'
                self.state_start_time = current_time
                self.get_logger().info('-> KÍCH HOẠT: Đóng kẹp Robotiq!')

        elif self.state == 'GRASP':
            self._set_gripper(0.025) 
            if current_time - self.state_start_time > 1.2:
                self.state = 'LIFT'
                self.state_start_time = current_time

        elif self.state == 'LIFT':
            lift = target_obj.copy()
            lift[2] += 0.30 
            self._publish_cartesian(lift)
            
            if current_time - self.state_start_time > 3.0:
                self.state = 'DONE'
                self.state_start_time = current_time 
                
        elif self.state == 'DONE':
            if current_time - self.state_start_time > 3.0:
                self._set_gripper(0.0) 
                self.state = 'RESETTING'
                self.state_start_time = current_time

        elif self.state == 'RESETTING':
            if current_time - self.state_start_time > 1.0:
                with self._lock:
                    self._target_obj = None
                    self._event_received = False
                self.state = 'WAITING'

def main(args=None):
    rclpy.init(args=args)
    node = VisionMotionNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()