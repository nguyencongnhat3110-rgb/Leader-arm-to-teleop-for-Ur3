import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import csv

class TrajectoryLogger(Node):
    def __init__(self):
        super().__init__('trajectory_logger')
        
        self.num_joints = 6
        self.master_q = [0.0] * self.num_joints
        self.ur3_q = [0.0] * self.num_joints
        
        # Mở file CSV để ghi đè (append)
        self.file = open('teleop_data.csv', mode='w', newline='')
        self.writer = csv.writer(self.file)
        # Ghi Header
        self.writer.writerow(['Time', 'Master_J1', 'UR3_J1', 'Master_J2', 'UR3_J2'])

        self.start_time = time.time()

        # Đọc tọa độ từ Feetech và UR3
        self.create_subscription(JointState, '/feetech_cmd', self.feetech_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.ur3_cb, 10)

        # Ghi dữ liệu ở tần số 50Hz (đủ mịn để vẽ đồ thị)
        self.create_timer(0.02, self.log_data)
        self.get_logger().info("Đang ghi dữ liệu quỹ đạo ra file teleop_data.csv...")

    def feetech_cb(self, msg):
        if len(msg.position) >= self.num_joints:
            self.master_q = msg.position

    def ur3_cb(self, msg):
        if len(msg.position) >= self.num_joints:
            self.ur3_q = msg.position

    def log_data(self):
        current_time = time.time() - self.start_time
        # Ví dụ này ghi lại Khớp 1 (Base) và Khớp 2 (Shoulder)
        self.writer.writerow([current_time, 
                              self.master_q[0], self.ur3_q[0], 
                              self.master_q[1], self.ur3_q[1]])

    def destroy_node(self):
        self.file.close()
        self.get_logger().info("Đã lưu file CSV thành công!")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
