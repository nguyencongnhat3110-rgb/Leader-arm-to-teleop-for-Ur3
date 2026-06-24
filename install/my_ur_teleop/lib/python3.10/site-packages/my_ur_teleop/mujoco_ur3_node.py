import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
import mujoco
import mujoco.viewer
import numpy as np
import threading
import time

class MuJoCoFollowerNode(Node):
    def __init__(self):
        super().__init__('mujoco_ur3_follower')
        
        # 1. LOAD MÔ HÌNH VẬT LÝ MUJOCO
        # Bạn cần có file XML mô hình UR3. Bạn có thể tải từ thư viện MuJoCo Menagerie trên GitHub
        xml_path = "path/to/your/ur3e/scene.xml" 
        try:
            self.model = mujoco.MjModel.from_xml_path(xml_path)
            self.data = mujoco.MjData(self.model)
        except Exception as e:
            self.get_logger().error(f"Lỗi load mô hình: {e}")
            raise

        # 2. KHỞI TẠO BIẾN ROS
        self.target_joints = np.zeros(6)
        self.data_lock = threading.Lock()
        
        # ROS PUB/SUB
        self.sub = self.create_subscription(JointState, '/master/joint_states', self.joint_callback, 10)
        self.force_pub = self.create_publisher(WrenchStamped, '/follower/tcp_force', 10)

        # 3. CHẠY LUỒNG VẬT LÝ VÀ ĐỒ HỌA
        self.sim_running = True
        self.sim_thread = threading.Thread(target=self.physics_loop)
        self.sim_thread.start()

    def joint_callback(self, msg):
        if len(msg.position) < 6: return
        with self.data_lock:
            # Lưu tọa độ mục tiêu từ Master
            self.target_joints = np.array(msg.position[:6])

    def physics_loop(self):
        # Mở cửa sổ 3D của MuJoCo
        viewer = mujoco.viewer.launch_passive(self.model, self.data)
        
        # Tần số mô phỏng vật lý (thường là 500Hz - 1000Hz)
        step_time = self.model.opt.timestep

        while self.sim_running and viewer.is_running():
            start_time = time.time()

            with self.data_lock:
                target_q = self.target_joints.copy()

            # --- ĐIỀU KHIỂN KHỚP VẬT LÝ BẰNG PD CONTROL ---
            # Trong file XML, bạn cần định nghĩa <actuator> cho 6 khớp (position hoặc motor)
            # Gán tín hiệu điều khiển vào data.ctrl
            for i in range(6):
                self.data.ctrl[i] = target_q[i]

            # Bước nhảy vật lý
            mujoco.mj_step(self.model, self.data)

            # --- ĐỌC CẢM BIẾN LỰC F/T TỪ MUJOCO ---
            # Trong XML bạn phải gắn thẻ <force> và <torque> ở thẻ <sensor> vào điểm TCP
            # Giả sử cảm biến lực F/T nằm ở index 0 đến 2 trong mảng sensordata
            if self.model.nsensor > 0:
                # Trích xuất lực X, Y, Z (đơn vị Newton chuẩn của MuJoCo)
                fx, fy, fz = self.data.sensordata[0:3] 
                
                # Publish lên mạng ROS cho tay Master đọc
                force_msg = WrenchStamped()
                force_msg.header.stamp = self.get_clock().now().to_msg()
                force_msg.wrench.force.x = float(fx)
                force_msg.wrench.force.y = float(fy)
                force_msg.wrench.force.z = float(fz)
                self.force_pub.publish(force_msg)

            # Đồng bộ đồ họa (Render khoảng 60 FPS)
            viewer.sync()

            elapsed = time.time() - start_time
            if elapsed < step_time:
                time.sleep(step_time - elapsed)

        viewer.close()

    def destroy_node(self):
        self.sim_running = False
        self.sim_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MuJoCoFollowerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
