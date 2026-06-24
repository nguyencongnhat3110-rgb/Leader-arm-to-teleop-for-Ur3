import os
import cv2
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import mujoco
import mujoco.viewer
import numpy as np
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from std_msgs.msg import Float64, Empty

class MujocoSimNode(Node):
    def __init__(self):
        super().__init__('mujoco_sim_node')
        self.get_logger().info("MuJoCo Node: Khởi động hệ thống điều khiển Jacobian OSC...")

        package_share_dir = get_package_share_directory('my_ur_teleop')
        self.xml_path = os.path.join(package_share_dir, 'models', 'scene.xml')

        self.model = mujoco.MjModel.from_xml_path(self.xml_path)
        self.data  = mujoco.MjData(self.model)

        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "home")
        if key_id != -1:
            home_angles = self.model.key_qpos[key_id][:6]
            self.data.qpos[:6] = home_angles
            self.data.ctrl[:6] = home_angles

        mujoco.mj_step(self.model, self.data) 
        self.site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")
        
        self.target_quat = np.zeros(4)
        mujoco.mju_mat2Quat(self.target_quat, self.data.site_xmat[self.site_id])
        self.target_xyz = None 

        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        self.joint_state_pub  = self.create_publisher(JointState, '/joint_states', 10)
        self.image_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.reached_pub = self.create_publisher(Empty, '/target_reached', 10)

        self.cartesian_sub = self.create_subscription(PoseStamped, '/cartesian_target', self.cartesian_callback, 10)
        self.gripper_sub = self.create_subscription(Float64, '/gripper_cmd', self.gripper_callback, 10)

        self.rgb_renderer = mujoco.Renderer(self.model, height=480, width=640)
        self.depth_renderer = mujoco.Renderer(self.model, height=480, width=640)
        self.depth_renderer.enable_depth_rendering()
        
        self.bridge = CvBridge()
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        
        self.viewer.cam.distance, self.viewer.cam.azimuth, self.viewer.cam.elevation = 1.5, 135.0, -25.0
        self.viewer.cam.lookat[:] = [0.45, 0.0, 0.1]

        self.step_counter = 0
        self.timer = self.create_timer(0.01, self.sim_loop)

    def cartesian_callback(self, msg):
        self.target_xyz = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def gripper_callback(self, msg):
        self.data.ctrl[6] = max(0.0, min(msg.data, 0.025))

    def osc_controller(self):
        if self.target_xyz is None: return

        current_pos = self.data.site_xpos[self.site_id]
        current_quat = np.zeros(4)
        mujoco.mju_mat2Quat(current_quat, self.data.site_xmat[self.site_id])

        dx = self.target_xyz - current_pos
        inv_current_quat = np.zeros(4)
        mujoco.mju_negQuat(inv_current_quat, current_quat)
        q_error = np.zeros(4)
        mujoco.mju_mulQuat(q_error, self.target_quat, inv_current_quat)
        
        dr = np.zeros(3)
        mujoco.mju_quat2Vel(dr, q_error, 1.0)
        err = np.hstack((dx, dr)) 

        jacp, jacr = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, jacp, jacr, self.site_id)
        J = np.vstack((jacp[:, :6], jacr[:, :6])) 

        J_pinv = J.T @ np.linalg.inv(J @ J.T + 0.02 * np.eye(6))
        dq = J_pinv @ err
        self.data.ctrl[:6] = self.data.qpos[:6] + dq * 0.25

    def sim_loop(self):
        if not self.viewer.is_running():
            rclpy.shutdown()
            return

        self.osc_controller() 
        mujoco.mj_step(self.model, self.data)
        self.viewer.sync()
        self.step_counter += 1

        if self.target_xyz is not None:
            # Ngưỡng nới lỏng thành 1.5cm để chống bị cấn/treo do vật lý
            if np.linalg.norm(self.target_xyz - self.data.site_xpos[self.site_id]) < 0.015:
                self.reached_pub.publish(Empty())

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name, joint_msg.position = self.joint_names, self.data.qpos[:6].tolist()
        self.joint_state_pub.publish(joint_msg)

        if self.step_counter % 10 == 0:
            self._publish_camera_image()

    def _publish_camera_image(self):
        try:
            self.rgb_renderer.update_scene(self.data, camera="realsense_d435")
            bgr_img = cv2.cvtColor(self.rgb_renderer.render(), cv2.COLOR_RGB2BGR)
            img_msg = self.bridge.cv2_to_imgmsg(bgr_img, encoding="bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "realsense_d435"
            self.image_pub.publish(img_msg)

            self.depth_renderer.update_scene(self.data, camera="realsense_d435")
            depth_msg = self.bridge.cv2_to_imgmsg(self.depth_renderer.render(), encoding="32FC1")
            depth_msg.header.stamp = img_msg.header.stamp
            depth_msg.header.frame_id = "realsense_d435"
            self.depth_pub.publish(depth_msg)
        except Exception: pass

    def close_viewer(self):
        if hasattr(self, 'viewer') and self.viewer.is_running():
            self.viewer.close()

def main(args=None):
    rclpy.init(args=args)
    node = MujocoSimNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.close_viewer()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()