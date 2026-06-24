import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
import rtde_control
import rtde_receive
import numpy as np
import time
import threading
import sys
import csv
from collections import deque

class UR3HybridHaptic(Node):
    def __init__(self):
        super().__init__('ur3_hybrid_haptic')
        ur_ip = "192.168.1.1" # Sửa lại cho đúng IP thật nếu cần
        
        self.get_logger().info(f"Đang kết nối với UR3 tại {ur_ip}...")
        try:
            self.rtde_c = rtde_control.RTDEControlInterface(ur_ip)
            self.rtde_r = rtde_receive.RTDEReceiveInterface(ur_ip)
            self.get_logger().info("✅ Kết nối RTDE thành công!")
        except Exception as e:
            self.get_logger().error(f"❌ Không thể kết nối: {e}")
            sys.exit(1)

        self.joint_signs = np.array([1.0, -1.0, 1.0, 1.0, 1.0, 1.0], dtype=float)
        self.target_vel = np.zeros(6)
        
        self.is_stuck = False
        self.bounce_start_time = 0.0
        self.bounce_vel = np.zeros(6)
        
        self.force_history = deque(maxlen=40)
        self.hard_stuck_frames = 0
        self.slow_stuck_frames = 0  
        
        self.system_running = True
        self.lock = threading.Lock()
        
        self.calibrated = False
        self.master_start_joints = None
        self.ur_start_joints = None
        self.target_joints = np.array(self.rtde_r.getActualQ(), dtype=float)
        self.first_msg_received = False

        self.force_msg = WrenchStamped()
        self.force_msg.header.frame_id = "ur3_link"
        self.force_msg.wrench.force.x = 0.0
        self.force_msg.wrench.force.y = 0.0
        self.force_msg.wrench.force.z = 0.0
        self.force_msg.wrench.torque.x = 0.0
        self.force_msg.wrench.torque.y = 0.0
        self.force_msg.wrench.torque.z = 0.0

        self.subscription = self.create_subscription(JointState, '/master/joint_states', self.ros_callback, 1)
        self.force_pub = self.create_publisher(WrenchStamped, '/follower/tcp_force', 1)

        # --- BIẾN PHỤC VỤ ĐO TRỄ (LƯU TRÊN RAM) ---
        self.log_data = []
        self.log_start_time = time.time()

        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()

    def ros_callback(self, msg):
        if len(msg.position) < 6: return
        with self.lock:
            self.target_joints = np.array(msg.position[:6], dtype=float)
            self.first_msg_received = True

    def control_loop(self):
        self.get_logger().info("🚀 KHỞI ĐỘNG: STRICT 3-RULES (CHẾ ĐỘ BÁM ĐUỔI TỐC ĐỘ CAO)...")
        
        try:
            while self.system_running:
                if not self.first_msg_received:
                    time.sleep(0.01)
                    continue
                    
                try:
                    if not self.rtde_c.isProgramRunning():
                        self.get_logger().error("🛑 TỦ ĐIỆN ĐÃ NGẮT SCRIPT HOẶC KHÔNG PHẢN HỒI!")
                        break

                    t_start = self.rtde_c.initPeriod()
                    
                    with self.lock:
                        raw_master = self.target_joints.copy()
                        
                    curr_joints = np.array(self.rtde_r.getActualQ())
                    curr_qd = np.array(self.rtde_r.getActualQd()) 
                    
                    if not self.calibrated:
                        self.master_start_joints = raw_master.copy()
                        self.ur_start_joints = curr_joints.copy()
                        self.calibrated = True

                    master_displacement = raw_master - self.master_start_joints
                    q_target = self.ur_start_joints + (master_displacement * self.joint_signs)
                    error = q_target - curr_joints
                    error = (error + np.pi) % (2 * np.pi) - np.pi 
                    
                    # 🔥 TỐI ƯU 1: Tăng P-Gain lên 15.0, mở rộng trần vận tốc lên 2.5 rad/s
                    self.target_vel = np.clip(error * 15.0, -2.5, 2.5)

                    live_tcp_force = np.array(self.rtde_r.getActualTCPForce())
                    live_force_N = float(np.linalg.norm(live_tcp_force[:3]))
                    
                    # --- GHI HÌNH QUỸ ĐẠO KHỚP 0 VÀO RAM ---
                    if self.calibrated and not self.is_stuck:
                        t_now = time.time() - self.log_start_time
                        self.log_data.append([t_now, q_target[0], curr_joints[0]])

                    if not self.is_stuck:
                        self.force_history.append(live_force_N)

                    max_vel = np.max(np.abs(curr_qd))
                    max_err = np.max(np.abs(error))

                    if not self.is_stuck:
                        if (max_err > 0.08) and (max_vel < 0.05):
                            self.hard_stuck_frames += 1
                        else:
                            self.hard_stuck_frames = 0
                            
                        if (max_err > 0.05) and (max_vel < 0.15):
                            self.slow_stuck_frames += 1
                        else:
                            self.slow_stuck_frames = 0

                        if (self.hard_stuck_frames >= 2) or (self.slow_stuck_frames >= 12): 
                            self.is_stuck = True
                            self.bounce_start_time = time.time()
                            
                            if len(self.force_history) >= 10:
                                history_list = list(self.force_history)
                                baseline_force = np.mean(history_list[-10:])
                            else:
                                baseline_force = live_force_N
                                
                            impact_force = live_force_N - baseline_force
                            
                            vel_norm = float(np.linalg.norm(self.target_vel))
                            if vel_norm > 0.01:
                                self.bounce_vel = -(self.target_vel / vel_norm) * 0.20 
                            else:
                                self.bounce_vel = np.zeros(6)
                                
                            if self.system_running: 
                                trigger_type = "CỨNG" if self.hard_stuck_frames >= 2 else "MỀM"
                                self.get_logger().warn(f"💥 [KINEMATIC - ĐÂM {trigger_type}] BÁO KẸT! Lực: {impact_force:.1f}N.")
                    
                    self.force_msg.header.stamp = self.get_clock().now().to_msg()

                    if self.is_stuck:
                        self.force_msg.wrench.force.x = 100.0
                        elapsed_stuck = time.time() - self.bounce_start_time
                        
                        if elapsed_stuck < 0.25:
                            # Tăng nhẹ gia tốc lúc giật lùi để xả ngoàm nhanh hơn
                            self.rtde_c.speedJ(self.bounce_vel.tolist(), 10.0, 0.008) 
                        elif elapsed_stuck < 1.5:
                            self.rtde_c.speedJ([0.0]*6, 2.0, 0.008)
                        else:
                            self.is_stuck = False
                            self.hard_stuck_frames = 0
                            self.slow_stuck_frames = 0
                            self.force_history.clear()
                            for _ in range(40):
                                self.force_history.append(live_force_N)
                    else:
                        self.force_msg.wrench.force.x = 0.0
                        # 🔥 TỐI ƯU 2: Đẩy gia tốc lên 15.0 để xóa độ trễ pha
                        self.rtde_c.speedJ(self.target_vel.tolist(), 15.0, 0.008)

                    self.force_pub.publish(self.force_msg)
                    self.rtde_c.waitPeriod(t_start)
                    
                except Exception as e:
                    if self.system_running:
                        self.get_logger().error(f"🔥 LỖI VÒNG LẶP ĐIỀU KHIỂN: {e}")
                    break 
                    
        finally:
            print("\n🚨 [WATCHDOG SAFETY] Luồng kết thúc! Đang phát lệnh speedStop()...")
            try:
                self.rtde_c.speedStop()
                print("✅ Robot đã dừng an toàn (Hợp pháp).")
            except Exception as e:
                print(f"⚠️ Không thể phát lệnh speedStop: {e}")

    def shutdown_sequence(self):
        self.system_running = False
        if hasattr(self, 'control_thread') and self.control_thread.is_alive():
            self.control_thread.join(timeout=1.0)
            
        # --- XẢ DỮ LIỆU ĐỒ THỊ RA FILE CSV TRƯỚC KHI TẮT ---
        try:
            with open('latency_data_stable.csv', mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Time', 'Master_J1_Pos', 'UR3_J1_Pos'])
                writer.writerows(self.log_data)
            print(f"\n📊 Đã xuất file: latency_data_stable.csv ({len(self.log_data)} khung hình)")
        except Exception as e:
            print(f"\n⚠️ Lỗi lưu file CSV: {e}")

        try:
            self.rtde_c.speedStop()
            self.rtde_c.disconnect()
            self.rtde_r.disconnect()
            print("✅ Đã giải phóng hoàn toàn cổng mạng RTDE.")
        except Exception as e: 
            pass

def main(args=None):
    rclpy.init(args=args)
    node = UR3HybridHaptic()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass 
    finally: 
        node.shutdown_sequence()
        node.destroy_node()
        if rclpy.ok(): 
            rclpy.shutdown()

if __name__ == '__main__':
    main()