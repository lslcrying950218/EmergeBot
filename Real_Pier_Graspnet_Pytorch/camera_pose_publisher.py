import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
import numpy as np
import time

class CameraPosePublisher(Node):
    def __init__(self):
        super().__init__("camera_pose_publisher")

        # Hand-eye calibration: camera OPTICAL pose relative to gripper
        # NOTE: ArUco uses camera_color_optical_frame (not camera_link)
        # This is T_gripper_to_optical, giving optical frame world pose directly
        self.cam_pos_gripper = np.array([-0.0763, 0.0039, 0.0350])
        self.cam_quat_gripper = np.array([-0.120, 0.124, -0.697, 0.696])
        self.cam_rot_gripper = R.from_quat(self.cam_quat_gripper).as_matrix()

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to gripper pose in world frame
        self.gripper_sub = self.create_subscription(
            PoseStamped, "/end_pose_stamped", self.gripper_cb, 10)

        # Publish camera pose in world frame (for Contact-GraspNet)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.cam_pub = self.create_publisher(PoseStamped, "/piper/end_pose", qos)

        # Log timing control
        self._last_log_time = 0
        self._log_interval = 5.0  # Print every 5 seconds

        self.get_logger().info("Camera pose publisher started")
        self.get_logger().info(f"  Hand-eye calibration (optical frame): pos={self.cam_pos_gripper}, quat={self.cam_quat_gripper}")

    def gripper_cb(self, msg):
        # Get gripper pose in world frame
        gripper_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        gripper_quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                                  msg.pose.orientation.z, msg.pose.orientation.w])
        gripper_rot = R.from_quat(gripper_quat).as_matrix()

        # Compute camera_link pose in world frame: T_link_world = T_gripper_world @ T_link_gripper
        cam_link_pos_world = gripper_pos + gripper_rot @ self.cam_pos_gripper
        cam_link_rot_world = gripper_rot @ self.cam_rot_gripper
        cam_link_quat_world = R.from_matrix(cam_link_rot_world).as_quat()

        # Publish camera_link pose (for Contact-GraspNet)
        cam_msg = PoseStamped()
        cam_msg.header = msg.header
        cam_msg.header.frame_id = "world"
        cam_msg.pose.position.x = float(cam_link_pos_world[0])
        cam_msg.pose.position.y = float(cam_link_pos_world[1])
        cam_msg.pose.position.z = float(cam_link_pos_world[2])
        cam_msg.pose.orientation.x = float(cam_link_quat_world[0])
        cam_msg.pose.orientation.y = float(cam_link_quat_world[1])
        cam_msg.pose.orientation.z = float(cam_link_quat_world[2])
        cam_msg.pose.orientation.w = float(cam_link_quat_world[3])

        self.cam_pub.publish(cam_msg)

        # Publish TF: world -> camera_link
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "world"
        t.child_frame_id = "camera_link"
        t.transform.translation.x = float(cam_link_pos_world[0])
        t.transform.translation.y = float(cam_link_pos_world[1])
        t.transform.translation.z = float(cam_link_pos_world[2])
        t.transform.rotation.x = float(cam_link_quat_world[0])
        t.transform.rotation.y = float(cam_link_quat_world[1])
        t.transform.rotation.z = float(cam_link_quat_world[2])
        t.transform.rotation.w = float(cam_link_quat_world[3])
        self.tf_broadcaster.sendTransform(t)

        # Log gripper/camera pose info every few seconds
        now_time = time.time()
        if now_time - self._last_log_time >= self._log_interval:
            self._last_log_time = now_time
            diff_mm = (cam_link_pos_world - gripper_pos) * 1000
            self.get_logger().info(
                f"[LINK6 POSE]   pos=[{gripper_pos[0]:.4f}, {gripper_pos[1]:.4f}, {gripper_pos[2]:.4f}]"
            )
            self.get_logger().info(
                f"[CAMERA LINK]  pos=[{cam_link_pos_world[0]:.4f}, {cam_link_pos_world[1]:.4f}, {cam_link_pos_world[2]:.4f}]"
            )
            self.get_logger().info(
                f"[DIFF]         offset=[{diff_mm[0]:.1f}, {diff_mm[1]:.1f}, {diff_mm[2]:.1f}]mm"
            )

def main():
    rclpy.init()
    node = CameraPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
