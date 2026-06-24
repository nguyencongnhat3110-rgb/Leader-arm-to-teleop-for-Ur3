import numpy as np
from typing import Sequence, Optional
from my_ur_teleop.hardware.robot import Robot
from .driver_feetech.scservo_sdk import PortHandler, sms_sts 

class FeetechRobot(Robot):
    def __init__(
        self, 
        joint_ids: Sequence[int],
        joint_offsets: Optional[Sequence[float]] = None,
        joint_signs: Optional[Sequence[int]] = None,
        port: str = "/dev/ttyUSB0", 
        baudrate: int = 1000000, 
        real: bool = False,
    ):
        self._num_joints = len(joint_ids)
        self._joint_ids = joint_ids
        self._real = real 
        
        self._joint_offsets = np.zeros(self._num_joints) if joint_offsets is None else np.array(joint_offsets)
        self._joint_signs = np.ones(self._num_joints) if joint_signs is None else np.array(joint_signs)
        
        self._last_pos = None
        
        # [BẢN VÁ 1]: Nới lỏng bộ lọc nhiễu, nuốt chửng 85% rung giật cơ khí
        self._alpha = 0.15 

        # --- CÁC BIẾN CHO THUẬT TOÁN MULTI-TURN (QUAY VÔ HẠN) ---
        self._rotations = np.zeros(self._num_joints)  
        self._last_raw_ticks = np.zeros(self._num_joints)
        self._first_read = True
        self._tracked_ticks = np.zeros(self._num_joints, dtype=int) 

        # --- CÁC BIẾN PHỤC VỤ SOFT STOP ---
        self._is_stopped = False
        self._stop_reason = ""
        self._safe_pos = None
        
        # Giới hạn UR3: 5 khớp đầu 6.28 rad (360 độ), Khớp cuối vô hạn
        self._ur3_limits = np.array([6.28, 6.28, 6.28, 6.28, 6.28, 999.0])

        if self._real:
            self.portHandler = PortHandler(port)
            self.packetHandler = sms_sts(self.portHandler)
            print(f"[Feetech] Đang khởi tạo kết nối phần cứng tại {port}...")
            if not self.portHandler.openPort() or not self.portHandler.setBaudRate(baudrate):
                raise RuntimeError(f"❌ FATAL: Lỗi kết nối Serial.")
        else:
            self._fake_pos = np.zeros(self._num_joints)

    def num_dofs(self) -> int:
        return self._num_joints

    # ==========================================================
    # 1. HÀM AUTO-CALIBRATION
    # ==========================================================
    def calibrate(self, target_ur_pose: np.ndarray):
        if not self._real:
            return

        print("\n" + "="*55)
        print("🚀 KÍCH HOẠT ĐỒNG BỘ TỌA ĐỘ (AUTO-CALIBRATION)")
        print("1. Kéo tay UR3 về tư thế chuẩn bị làm việc (VD: Trên bàn cờ).")
        print("2. Nắn Master Feetech giống hệt tư thế của UR3.")
        input("3. Giữ chặt tay Master và bấm phím ENTER để chốt điểm Zero...")
        
        raw_positions = []
        for idx, j_id in enumerate(self._joint_ids):
            scs_present_position, scs_result, _ = self.packetHandler.ReadPos(j_id)
            if scs_result != 0: 
                scs_present_position = 2048 
                
            self._last_raw_ticks[idx] = scs_present_position
            self._rotations[idx] = 0

            rad = (scs_present_position - 2048) * (np.pi / 2048)
            raw_positions.append(rad)
        
        raw_rad_array = np.array(raw_positions)
        self._joint_offsets = raw_rad_array - (target_ur_pose * self._joint_signs)
        
        self._last_pos = target_ur_pose.copy()
        self._first_read = False
        
        print(f"✅ Offset mới đã nạp vào RAM: {np.round(self._joint_offsets, 3)}")
        print("="*55 + "\n")

    # ==========================================================
    # 2. HÀM ĐỌC TỌA ĐỘ VÀ MULTI-TURN
    # ==========================================================
    def get_joint_state(self) -> np.ndarray:
        if not self._real:
            return self._fake_pos
            
        if self._is_stopped:
            return self._safe_pos

        raw_positions = []
        for idx, j_id in enumerate(self._joint_ids):
            scs_present_position, scs_result, _ = self.packetHandler.ReadPos(j_id)
            
            if scs_result != 0:
                if self._first_read:
                    return None  # Tránh giật mình khi mới cắm USB
                else:
                    scs_present_position = self._last_raw_ticks[idx]

            if not self._first_read:
                diff = scs_present_position - self._last_raw_ticks[idx]
                if diff < -2048: 
                    self._rotations[idx] += 1
                elif diff > 2048:
                    self._rotations[idx] -= 1

            self._last_raw_ticks[idx] = scs_present_position
            
            continuous_ticks = scs_present_position + (self._rotations[idx] * 4096)
            self._tracked_ticks[idx] = int(continuous_ticks)
            
            rad = (continuous_ticks - 2048) * (np.pi / 2048)
            raw_positions.append(rad)
            
        self._first_read = False
        pos = np.array(raw_positions)
        pos = (pos - self._joint_offsets) * self._joint_signs
        
        if np.any(np.abs(pos) >= self._ur3_limits):
            violated = np.where(np.abs(pos) >= self._ur3_limits)[0]
            self._is_stopped = True
            
            if self._last_pos is not None:
                self._safe_pos = self._last_pos.copy()
            else:
                self._safe_pos = np.clip(pos, -self._ur3_limits + 0.01, self._ur3_limits - 0.01)

            self._stop_reason = f"Khớp {violated} chạm giới hạn của UR3"
            print(f"\n🚨 [Master Arm] SOFT STOP: {self._stop_reason}")
            return self._safe_pos

        if self._last_pos is None:
            self._last_pos = pos
        else:
            pos = self._last_pos * (1 - self._alpha) + pos * self._alpha
            self._last_pos = pos

        return pos

    def resume(self) -> bool:
        if not self._is_stopped: return True
        self._is_stopped = False
        self._stop_reason = ""
        return True

    def command_joint_state(self, joint_state: np.ndarray): pass
    def get_observations(self) -> dict: return {"joint_positions": self.get_joint_state()}

    # ==========================================================
    # 3. HÀM ĐIỀU KHIỂN LỰC (FORCE FEEDBACK / TORQUE LOCK)
    # ==========================================================
    def set_torque_lock(self, joint_id: int, enable: bool):
        if not self._real: return

        # 1 là gồng cứng (khóa), 0 là xìu xuống (nhả)
        lock_val = 1 if enable else 0

        try:
            # [BẢN VÁ 2]: Sử dụng hàm chuẩn từ thư viện sms_sts đã được sửa
            self.packetHandler.EnableTorque(joint_id, lock_val)
        except Exception:
            pass