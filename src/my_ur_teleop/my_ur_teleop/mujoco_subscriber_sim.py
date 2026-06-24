import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
import mujoco
import mujoco.viewer
import numpy as np
import threading
import time
import os
from ament_index_python.packages import get_package_share_directory

class MuJoCoSimSubscriber(Node):
    def __init__(self):
        super().__init__('mujoco_sim_follower')
        self.get_logger().info("🔵 Đang khởi tạo môi trường MuJoCo...")
        
        try:
            pkg_dir = get_package_share_directory('my_ur_teleop')
            xml_path = os.path.join(pkg_dir, 'models', 'scene.xml')
            self.model = mujoco.MjModel.from_xml_path(xml_path)
            self.data = mujoco.MjData(self.model)

            # [BẢN VÁ]: Dựng con robot đứng lên ở tư thế Home trước khi nhận lệnh
            home_qpos = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]
            for i in range(6):
                self.data.qpos[i] = home_qpos[i]
                self.data.ctrl[i] = home_qpos[i]
            mujoco.mj_step(self.model, self.data)

        except Exception as e:
            self.get_logger().error(f"Lỗi load mô hình XML: {e}")
            raise

        self.target_joints = np.zeros(6)
        self.data_lock = threading.Lock()
        
        # --- BIẾN ĐỒNG BỘ TỌA ĐỘ (AUTO-CALIBRATION) ---
        self.first_msg_received = False
        self.calibrated = False
        self.joint_offset = np.zeros(6)

        self.sub = self.create_subscription(JointState, '/master/joint_states', self.joint_callback, 10)
        self.force_pub = self.create_publisher(WrenchStamped, '/follower/tcp_force', 10)

        self.sim_running = True
        self.sim_thread = threading.Thread(target=self.physics_loop)
        self.sim_thread.start()

    def joint_callback(self, msg):
        if len(msg.position) < 6: return
        with self.data_lock:
            self.target_joints = np.array(msg.position[:6])
            self.first_msg_received = True

    def physics_loop(self):
        viewer = mujoco.viewer.launch_passive(self.model, self.data)
        step_time = self.model.opt.timestep

        while self.sim_running and viewer.is_running():
            start_time = time.time()

            # [BẢN VÁ]: Chờ nhận được tọa độ thật từ Master rồi mới bắt đầu tính toán
            if not self.first_msg_received:
                time.sleep(0.01)
                continue

            with self.data_lock:
                target_q = self.target_joints.copy()

            # Lấy tọa độ hiện tại của UR3 trong MuJoCo
            curr_joints = np.array(self.data.qpos[:6])

            # [BẢN VÁ]: Tự động đồng bộ Offset ở ngay frame đầu tiên
            if not self.calibrated:
                self.joint_offset = target_q - curr_joints
                self.calibrated = True

            # Trừ đi độ chênh lệch để UR3 ảo di chuyển tương đối theo tay Master
            aligned_target = target_q - self.joint_offset

            # Bơm tín hiệu vào động cơ ảo
            for i in range(6):
                self.data.ctrl[i] = aligned_target[i]

            mujoco.mj_step(self.model, self.data)

            # Đọc cảm biến lực phản hồi
            if self.model.nsensor > 0:
                fx, fy, fz = self.data.sensordata[0:3] 
                
                force_msg = WrenchStamped()
                force_msg.header.stamp = self.get_clock().now().to_msg()
                # Ở mô phỏng thì không có gai dòng điện, nên không cần low-pass filter nặng nề
                force_msg.wrench.force.x = float(fx)
                force_msg.wrench.force.y = float(fy)
                force_msg.wrench.force.z = float(fz)
                self.force_pub.publish(force_msg)

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
    node = MuJoCoSimSubscriber()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()