import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
import numpy as np
import time
from .hardware.feetech import FeetechRobot 

class FeetechSimPublisher(Node):
    def __init__(self):
        super().__init__('feetech_master_sim')
        self.publisher_ = self.create_publisher(JointState, '/master/joint_states', 10)
        self.force_sub = self.create_subscription(WrenchStamped, '/follower/tcp_force', self.force_callback, 10)
        
        self.robot = FeetechRobot(joint_ids=[1, 2, 3, 4, 5, 6], port='/dev/ttyUSB0', real=True)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info('🔵 CHẾ ĐỘ: GIAO TIẾP VỚI MÔ PHỎNG MUJOCO (Bản ổn định)')

        # Ngưỡng cao (35N) và đếm frame dài (15 frames) để triệt tiêu hoàn toàn nhiễu
        self.base_threshold_N = 35.0   
        self.smoothed_force = 0.0
        self.alpha_filter = 0.4             

        self.frames_above_threshold = 0
        self.frames_needed_to_lock = 15 

        self.is_system_locked = False
        self.lock_timestamp = 0.0
        self.auto_release_sec = 1.0  
        self.mute_until = 0.0 
        self.last_log_time = 0.0
        self.last_joints = None
        self.smoothed_velocity = 0.0

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        try:
            joints = self.robot.get_joint_state()
            if joints is None: return

            if self.is_system_locked:
                self.last_joints = joints 
                return  
            
            if self.last_joints is None:
                self.last_joints = joints

            delta_joints = np.abs(joints - self.last_joints)
            current_vel = np.max(delta_joints)
            
            self.smoothed_velocity = (0.15 * current_vel) + (0.85 * self.smoothed_velocity)
            self.last_joints = joints
            
            msg.position = joints.tolist()
            self.publisher_.publish(msg)
        except Exception:
            pass

    def force_callback(self, msg):
        now = time.time()

        raw_magnitude_N = np.sqrt(msg.wrench.force.x**2 + msg.wrench.force.y**2 + msg.wrench.force.z**2)
        if raw_magnitude_N > 400.0: raw_magnitude_N = 0.0
        
        self.smoothed_force = (self.alpha_filter * raw_magnitude_N) + ((1 - self.alpha_filter) * self.smoothed_force)

        # Asymmetric Scaling: Bóp nghẹt lực khi tay vung nhanh
        dynamic_threshold = self.base_threshold_N + (self.smoothed_velocity * 4000.0)

        if now - self.last_log_time > 1.0:
            status = "ĐANG KHÓA" if self.is_system_locked else ("ĐANG KHIÊN" if now < self.mute_until else "SẴN SÀNG")
            self.get_logger().info(f"🔵 MUJOCO [{status}] Lực Ảo: {self.smoothed_force:.1f}N | Ngưỡng: {dynamic_threshold:.1f}N")
            self.last_log_time = now

        if self.is_system_locked:
            if now - self.lock_timestamp > self.auto_release_sec:
                self.get_logger().info("✅ Đã nhả khóa. HỆ THỐNG ĐIẾC 1.5s để bạn rút tay!")
                for i in range(1, 7):
                    self.robot.set_torque_lock(joint_id=i, enable=False)
                self.is_system_locked = False
                self.mute_until = now + 1.5
            return 

        if now < self.mute_until:
            self.frames_above_threshold = 0
            return

        if self.smoothed_force > dynamic_threshold:
            self.frames_above_threshold += 1
        else:
            self.frames_above_threshold = 0

        if self.frames_above_threshold >= self.frames_needed_to_lock:
            self.get_logger().warn(f"💥 ĐÂM VẬT THỂ ẢO! Lực {self.smoothed_force:.1f}N vượt Ngưỡng {dynamic_threshold:.1f}N. KHÓA!")
            for i in range(1, 7):
                self.robot.set_torque_lock(joint_id=i, enable=True)
            self.is_system_locked = True
            self.lock_timestamp = now
            self.frames_above_threshold = 0

def main(args=None):
    rclpy.init(args=args)
    node = FeetechSimPublisher()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        for i in range(1, 7):
            node.robot.set_torque_lock(joint_id=i, enable=False)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()