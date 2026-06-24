import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
import numpy as np
import time
from .hardware.feetech import FeetechRobot
import sys

class FeetechBinaryHaptic(Node):
    def __init__(self):
        super().__init__('feetech_master_real')

        self.publisher_ = self.create_publisher(JointState, '/master/joint_states', 1)
        self.force_sub = self.create_subscription(
            WrenchStamped, '/follower/tcp_force', self.force_callback, 1)

        try:
            self.robot = FeetechRobot(
                joint_ids=[1, 2, 3, 4, 5, 6], port='/dev/ttyUSB0', real=True)
            self.get_logger().info('🟢 CỔNG USB ĐÃ MỞ: Logic Binary Haptic Sẵn Sàng!')
        except Exception as e:
            self.get_logger().error(f"❌ KHÔNG MỞ ĐƯỢC CỔNG FEETECH: {e}")
            sys.exit(1)

        self.timer = self.create_timer(0.008, self.timer_callback)

        self.is_system_locked = False
        self.lock_timestamp = 0.0
        self.auto_release_sec = 1.5
        self.mute_until = 0.0

        self.last_joints = None
        self.publish_frozen_when_locked = True  

        # 🌟 VÁ LỖI DDS TỐI ĐA: Khởi tạo message sạch 100%
        self.joint_msg = JointState()
        self.joint_msg.header.frame_id = "master_link"
        self.joint_msg.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.joint_msg.position = [0.0] * 6
        self.joint_msg.velocity = []
        self.joint_msg.effort = []

    def timer_callback(self):
        try:
            now = time.time()
            
            # 🌟 FIX TỬ HUYỆT: Chuyển logic nhả khóa sang Timer để không phụ thuộc vào UR3
            if self.is_system_locked and (now - self.lock_timestamp > self.auto_release_sec):
                self.get_logger().info("✅ Đã nhả khóa an toàn (Hết 1.5s hoặc UR3 mất kết nối)!")
                for i in range(1, 7):
                    self.robot.set_torque_lock(joint_id=i, enable=False)
                self.is_system_locked = False
                self.mute_until = now + 1.0

            raw_joints = self.robot.get_joint_state()
            if raw_joints is None or len(raw_joints) < 6:
                return

            joints = np.array(raw_joints[:6], dtype=float)

            if self.is_system_locked:
                if self.publish_frozen_when_locked and self.last_joints is not None:
                    self.joint_msg.header.stamp = self.get_clock().now().to_msg()
                    self.joint_msg.position = self.last_joints.tolist()
                    self.publisher_.publish(self.joint_msg)
                return

            self.last_joints = joints

            self.joint_msg.header.stamp = self.get_clock().now().to_msg()
            self.joint_msg.position = joints.tolist()
            self.publisher_.publish(self.joint_msg)

        except Exception:
            pass

    def force_callback(self, msg):
        try:
            now = time.time()
            signal_force = float(msg.wrench.force.x)

            if now < self.mute_until or self.is_system_locked:
                return

            if signal_force > 50.0:
                self.get_logger().warn("💥 UR3 BÁO KẸT CỨNG VẬT LÝ! KHÓA TAY MASTER NGAY!")
                for i in range(1, 7):
                    self.robot.set_torque_lock(joint_id=i, enable=True)
                self.is_system_locked = True
                self.lock_timestamp = now

        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = FeetechBinaryHaptic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[Ctrl+C] Đang tắt hệ thống...")
    finally:
        print("🔌 ĐANG GIẢI PHÓNG TORQUE MASTER...")
        if hasattr(node, 'robot'):
            # Đảm bảo lệnh xả torque được truyền xong trước khi tắt
            for i in range(1, 7): 
                try:
                    node.robot.set_torque_lock(joint_id=i, enable=False)
                except Exception: pass
            time.sleep(0.3) 
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("✅ Đã thoát an toàn!")
        sys.exit(0)

if __name__ == '__main__':
    main()