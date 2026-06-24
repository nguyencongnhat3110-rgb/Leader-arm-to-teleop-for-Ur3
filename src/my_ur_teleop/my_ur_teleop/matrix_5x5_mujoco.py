import rclpy
from rclpy.node import Node
import mujoco
import mujoco.viewer
import numpy as np
import threading
import time
import os
import math
from ament_index_python.packages import get_package_share_directory

class UR3AutoFivePointsSim(Node):
    def __init__(self):
        super().__init__('ur3_auto_five_points_sim')
        self.get_logger().info("🔵 Đang khởi tạo mô phỏng hệ TỌA ĐỘ ĐỀ-CÁC (Task Space Home)...")
        
        try:
            pkg_dir = get_package_share_directory('my_ur_teleop')
            xml_path = os.path.join(pkg_dir, 'models', 'scene.xml')
            self.model = mujoco.MjModel.from_xml_path(xml_path)
            self.data = mujoco.MjData(self.model)

            # 🎯 VỊ TRÍ ĐÍCH BAN ĐẦU: Lấy thẳng tọa độ Đề-các hệ BASE của Điểm 3 (m / rad) thực tế từ ảnh
            self.home_pos_target = np.array([-0.22661, -0.35167, 0.02804])
            self.home_quat_target = self.rv_to_quat(np.array([0.083, 3.153, -0.035]))

            # Cho robot ảo xuất phát từ tư thế đứng thẳng mặc định làm gốc
            self.cmd_q = np.zeros(self.model.nq)
            self.cmd_q[:6] = [
                -0.722566,   # Base      ( 41.40 độ)
                -0.965340,  # Shoulder  (-55.31 độ)
                1.589122,   # Elbow     ( 91.05 độ)
                -2.220059,  # Wrist 1   (-127.20 độ)
                -1.566956,  # Wrist 2   (-89.78 độ)
                -0.796217] 
            
            for i in range(6):
                self.data.qpos[i] = self.cmd_q[i]
                self.data.ctrl[i] = self.cmd_q[i]
            mujoco.mj_step(self.model, self.data)

        except Exception as e:
            self.get_logger().error(f"Lỗi load mô hình XML: {e}")
            raise

        self.ee_site_name = 'attachment_site'
        self.ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, self.ee_site_name) if \
                          mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, self.ee_site_name) >= 0 else 0

        # MẢNG 5 TOẠ ĐỘ ĐỀ-CÁC ĐÃ ĐỒNG BỘ VÙNG LÀM VIỆC AN TOÀN TRÊN MUJOCO ẢO
        self.POINTS_LIST = [
            [0.25, -0.25, 0.10, 0.004, -2.777, -0.102], 
            [0.22, -0.27, 0.10, 0.155,  3.086, -0.372], 
            [0.24, -0.24, 0.10, 0.083,  3.153, -0.035], # Điểm 3 (Đỉnh vòm cảm biến)
            [0.26, -0.28, 0.10, 0.127, -3.089, -0.251], 
            [0.23, -0.26, 0.10, 0.029,  2.918, -0.117], 
        ]

        self.PLUS_SIZE = 0.005       
        self.SAFE_RETRACT = -0.015   

        self.trajectory_ready = False
        self.waypoints = []
        self.current_wp_idx = 0

        self.sim_running = True
        self.sim_thread = threading.Thread(target=self.physics_loop, daemon=True)
        self.sim_thread.start()

    def rv_to_quat(self, rv):
        """Đổi vector xoay [rx, ry, rz] sang Quaternion [w, x, y, z]"""
        angle = np.linalg.norm(rv)
        quat = np.zeros(4)
        if angle < 1e-6:
            quat[0] = 1.0
            return quat
        axis = rv / angle
        mujoco.mju_axisAngle2Quat(quat, axis, angle)
        return quat

    def generate_auto_trajectory(self, current_any_pos, current_any_quat):
        """Sinh chuỗi hành trình mượt mà sau khi đã hút về điểm Home thành công"""
        current_any_mat = np.zeros(9)
        mujoco.mju_quat2Mat(current_any_mat, current_any_quat)
        current_any_mat = current_any_mat.reshape(3, 3)
        
        # Nhấc lùi nhẹ dọc trục công cụ 10mm đầu hành trình để tạo đà
        retract_first = current_any_pos + current_any_mat[:, 2] * -0.010
        self.waypoints.append((retract_first, current_any_quat, "Nhấc lùi an toàn ban đầu"))

        for idx, pt in enumerate(self.POINTS_LIST):
            center_pos = np.array(pt[0:3])
            center_quat = self.rv_to_quat(np.array(pt[3:6]))
            
            center_mat = np.zeros(9)
            mujoco.mju_quat2Mat(center_mat, center_quat)
            center_mat = center_mat.reshape(3, 3)

            name = f"Điểm {idx + 1}"

            safe_pose = center_pos + center_mat[:, 2] * self.SAFE_RETRACT
            self.waypoints.append((safe_pose, center_quat, f"🚀 {name} -> Điểm chờ"))
            self.waypoints.append((center_pos, center_quat, f"👇 {name} -> Tiếp cận mặt gel"))

            # Quỹ đạo hình dấu cộng bám sườn dốc Tool-Frame
            self.waypoints.append((center_pos + center_mat[:, 0] * self.PLUS_SIZE, center_quat, f"➕ {name} -> +X"))
            self.waypoints.append((center_pos, center_quat, f"➕ {name} -> Về tâm"))
            self.waypoints.append((center_pos - center_mat[:, 0] * self.PLUS_SIZE, center_quat, f"➕ {name} -> -X"))
            self.waypoints.append((center_pos, center_quat, f"➕ {name} -> Về tâm"))
            self.waypoints.append((center_pos + center_mat[:, 1] * self.PLUS_SIZE, center_quat, f"➕ {name} -> +Y"))
            self.waypoints.append((center_pos, center_quat, f"➕ {name} -> Về tâm"))
            self.waypoints.append((center_pos - center_mat[:, 1] * self.PLUS_SIZE, center_quat, f"➕ {name} -> -Y"))
            self.waypoints.append((center_pos, center_quat, f"➕ {name} -> Về tâm"))

            self.waypoints.append((safe_pose, center_quat, f"⬆️ {name} -> Thoát ly an toàn"))

        self.trajectory_ready = True
        self.get_logger().info("✅ Khởi tạo thành công chuỗi Waypoints từ mốc Đề-các!")

    def physics_loop(self):
        viewer = mujoco.viewer.launch_passive(self.model, self.data)
        step_time = self.model.opt.timestep
        start_loop_time = time.time()

        while self.sim_running and viewer.is_running():
            start_time = time.time()

            # PHASE 1 (1.5 Giây đầu): Tự động giải toán IK đưa đầu kẹp từ tư thế thẳng đứng về ôm sát vị trí Đề-các Điểm 3
            if time.time() - start_loop_time < 1.5:
                current_pos = self.data.site_xpos[self.ee_site_id]
                current_quat = np.zeros(4)
                mujoco.mju_mat2Quat(current_quat, self.data.site_xmat[self.ee_site_id])
                
                pos_error = self.home_pos_target - current_pos
                inv_current_quat = np.zeros(4)
                mujoco.mju_negQuat(inv_current_quat, current_quat)
                quat_error = np.zeros(4)
                mujoco.mju_mulQuat(quat_error, self.home_quat_target, inv_current_quat)
                rot_error = quat_error[1:4] * np.sign(quat_error[0])
                
                error_6d = np.concatenate([pos_error, rot_error])
                
                jac_pos = np.zeros((3, self.model.nv))
                jac_rot = np.zeros((3, self.model.nv))
                mujoco.mj_jacSite(self.model, self.data, jac_pos, jac_rot, self.ee_site_id)
                J = np.vstack([jac_pos[:, :6], jac_rot[:, :6]])
                
                J_inv = np.linalg.pinv(J, rcond=1e-4)
                dq = J_inv @ error_6d * 3.0  # Hút về từ tốn, mượt mà không bị bốc đầu giật cục
                
                self.cmd_q += dq * step_time
                self.data.ctrl[:6] = self.cmd_q
                
                mujoco.mj_step(self.model, self.data)
                viewer.sync()
                continue

            # PHASE 2: Đọc vị trí chính xác sau khi đã hạ cánh Home để sinh chuỗi hành trình tự động
            if not self.trajectory_ready:
                current_any_pos = self.data.site_xpos[self.ee_site_id].copy()
                current_any_quat = np.zeros(4)
                mujoco.mju_mat2Quat(current_any_quat, self.data.site_xmat[self.ee_site_id])
                self.generate_auto_trajectory(current_any_pos, current_any_quat)

            # PHASE 3: Thực thi chạy chuỗi hành trình qua 5 điểm dốc cảm biến
            if self.current_wp_idx < len(self.waypoints):
                target_pos, target_quat, description = self.waypoints[self.current_wp_idx]
                
                current_pos = self.data.site_xpos[self.ee_site_id]
                current_quat = np.zeros(4)
                mujoco.mju_mat2Quat(current_quat, self.data.site_xmat[self.ee_site_id])
                
                pos_error = target_pos - current_pos
                
                inv_current_quat = np.zeros(4)
                mujoco.mju_negQuat(inv_current_quat, current_quat)
                quat_error = np.zeros(4)
                mujoco.mju_mulQuat(quat_error, target_quat, inv_current_quat)
                rot_error = quat_error[1:4] * np.sign(quat_error[0])
                
                error_6d = np.concatenate([pos_error, rot_error])
                
                # Sai số chuyển điểm an toàn (1mm) chống treo máy hoàn toàn
                if np.linalg.norm(pos_error) < 0.001 and np.linalg.norm(rot_error) < 0.01:
                    self.get_logger().info(f"✅ Đã đạt mốc: {description}")
                    self.current_wp_idx += 1
                else:
                    jac_pos = np.zeros((3, self.model.nv))
                    jac_rot = np.zeros((3, self.model.nv))
                    mujoco.mj_jacSite(self.model, self.data, jac_pos, jac_rot, self.ee_site_id)
                    J = np.vstack([jac_pos[:, :6], jac_rot[:, :6]])
                    
                    J_inv = np.linalg.pinv(J, rcond=1e-4)
                    dq = J_inv @ error_6d * 4.0  
                    
                    self.cmd_q += dq * step_time
                    self.data.ctrl[:6] = self.cmd_q
            else:
                self.get_logger().info("✨ HOÀN THÀNH XUẤT SẮC TOÀN BỘ CHU TRÌNH MÔ PHỎNG ĐỀ-CÁC!", once=True)

            mujoco.mj_step(self.model, self.data)
            viewer.sync()

            elapsed = time.time() - start_time
            if elapsed < step_time:
                time.sleep(step_time - elapsed)

        viewer.close()

    def destroy_node(self):
        self.sim_running = False
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UR3AutoFivePointsSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()