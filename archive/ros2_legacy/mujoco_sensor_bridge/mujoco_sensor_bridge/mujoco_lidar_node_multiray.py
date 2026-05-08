"""MuJoCo LiDAR raycast node (mj_multiRay backend).

Loads a copy of the MuJoCo model, subscribes to /joint_states to sync joint
positions, then casts 11520 rays (720 horizontal x 16 vertical) from the LiDAR
mount position using mj_multiRay(). Publishes sensor_msgs/PointCloud2 at 10 Hz
on /{ns}/registered_scan.

This is a drop-in replacement for mujoco_lidar_node that uses a single batched
mj_multiRay() C call instead of 11,520 individual mj_ray() Python-loop calls.

Uses a MuJoCo site for LiDAR mount position and orientation — no hardcoded
offsets. The site orientation (e.g. 13 deg pitch for Livox MID-360) is
automatically applied to ray directions and used to transform hit points back
into the LiDAR local frame.

Simulates a Livox MID-360:
  - 360 deg horizontal FOV
  - -7 deg to +52 deg vertical FOV
  - 720 horizontal samples x 16 vertical lines = 11520 points
  - Range: 0.05 - 20.0 m
"""

import math
import threading

import mujoco
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState, PointCloud2, PointField
from std_msgs.msg import Header
from ament_index_python.packages import get_package_share_directory


class MujocoLidarMultiRayNode(Node):

    def __init__(self):
        super().__init__('mujoco_lidar_node_multiray',
                         parameter_overrides=[rclpy.Parameter('use_sim_time', value=True)])

        # --- Parameters ---
        self.declare_parameter('mjcf_path', '')
        self.declare_parameter('lidar_site', 'livox_mid360')
        self.declare_parameter('lidar_body', 'base_link')
        self.declare_parameter('hz_samples', 720)
        self.declare_parameter('vt_samples', 16)
        self.declare_parameter('h_fov_deg', 360.0)
        self.declare_parameter('v_min_deg', -7.0)
        self.declare_parameter('v_max_deg', 52.0)
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 20.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('frame_id', 'lidar_link')

        mjcf_path = self.get_parameter('mjcf_path').get_parameter_value().string_value
        if not mjcf_path:
            # Default: look for go2w.xml in go2_gazebo_sim package
            try:
                pkg_dir = get_package_share_directory('go2_gazebo_sim')
                mjcf_path = pkg_dir + '/mujoco/go2w.xml'
            except Exception:
                self.get_logger().fatal('mjcf_path not set and go2_gazebo_sim not found')
                raise RuntimeError('mjcf_path is required')

        lidar_site_name = self.get_parameter('lidar_site').get_parameter_value().string_value
        self.lidar_body_name = self.get_parameter('lidar_body').get_parameter_value().string_value
        self.hz_samples = self.get_parameter('hz_samples').get_parameter_value().integer_value
        self.vt_samples = self.get_parameter('vt_samples').get_parameter_value().integer_value
        h_fov = math.radians(self.get_parameter('h_fov_deg').get_parameter_value().double_value)
        v_min = math.radians(self.get_parameter('v_min_deg').get_parameter_value().double_value)
        v_max = math.radians(self.get_parameter('v_max_deg').get_parameter_value().double_value)
        self.range_min = self.get_parameter('range_min').get_parameter_value().double_value
        self.range_max = self.get_parameter('range_max').get_parameter_value().double_value
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # --- Load MuJoCo model ---
        self.get_logger().info(f'Loading MuJoCo model from: {mjcf_path}')
        self.mj_model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.mj_data = mujoco.MjData(self.mj_model)
        self._mj_lock = threading.Lock()

        # Find lidar site (position + orientation from MJCF, no hardcoded offsets)
        self.lidar_site_id = mujoco.mj_name2id(
            self.mj_model, mujoco.mjtObj.mjOBJ_SITE, lidar_site_name)
        if self.lidar_site_id < 0:
            self.get_logger().fatal(f'Site "{lidar_site_name}" not found in MuJoCo model')
            raise RuntimeError(f'Site not found: {lidar_site_name}')

        # Body id still needed for mj_multiRay bodyexclude
        self.lidar_body_id = mujoco.mj_name2id(
            self.mj_model, mujoco.mjtObj.mjOBJ_BODY, self.lidar_body_name)
        if self.lidar_body_id < 0:
            self.get_logger().fatal(f'Body "{self.lidar_body_name}" not found in MuJoCo model')
            raise RuntimeError(f'Body not found: {self.lidar_body_name}')

        # Build joint name -> qpos index mapping
        self._joint_map = {}
        self._free_jnt_qpos_adr = None
        for i in range(self.mj_model.njnt):
            name = mujoco.mj_id2name(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if self.mj_model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE:
                self._free_jnt_qpos_adr = self.mj_model.jnt_qposadr[i]
            elif name:
                qpos_adr = self.mj_model.jnt_qposadr[i]
                self._joint_map[name] = qpos_adr

        if self._free_jnt_qpos_adr is None:
            self.get_logger().warn('No free joint found — root body position will not track ground truth')

        # --- Pre-compute ray directions in LiDAR local frame ---
        # Horizontal angles: equally spaced over h_fov, centered at 0
        h_angles = np.linspace(-h_fov / 2, h_fov / 2, self.hz_samples, endpoint=False)
        # Vertical angles: equally spaced from v_min to v_max
        v_angles = np.linspace(v_min, v_max, self.vt_samples)

        # Build (N, 3) ray direction array in LiDAR-local frame
        # Convention: x forward, z up
        h_grid, v_grid = np.meshgrid(h_angles, v_angles, indexing='ij')
        h_flat = h_grid.ravel()
        v_flat = v_grid.ravel()
        self.n_rays = len(h_flat)

        cos_v = np.cos(v_flat)
        self._ray_dirs_local = np.stack([
            cos_v * np.cos(h_flat),
            cos_v * np.sin(h_flat),
            np.sin(v_flat),
        ], axis=1).astype(np.float64)  # (N, 3)

        self.get_logger().info(
            f'LiDAR configured: {self.hz_samples}x{self.vt_samples} = {self.n_rays} rays, '
            f'{rate} Hz, range [{self.range_min}, {self.range_max}]m (mj_multiRay backend)')

        # --- Geom group filter: cast against all groups ---
        # geomgroup is a byte array of length mjNGROUP (6); 1 = include group
        self._geomgroup = None  # None means all geoms

        # --- Pre-allocate reusable buffers for mj_multiRay ---
        self._ray_dist = np.full(self.n_rays, self.range_max, dtype=np.float64)
        self._ray_geomid = np.full(self.n_rays, -1, dtype=np.int32)

        # --- ROS pub/sub ---
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.pc_pub = self.create_publisher(PointCloud2, 'registered_scan', qos)

        self.js_sub = self.create_subscription(
            JointState, 'joint_states', self._joint_state_cb, 10)

        self._latest_js = None
        self._js_lock = threading.Lock()
        self._last_stamp_ns = 0  # monotonicity guard for sim-time stamps

        # Subscribe to ground-truth pose to sync free joint (root body position)
        self._latest_pose = None
        self._pose_lock = threading.Lock()
        self.pose_sub = self.create_subscription(
            PoseStamped, 'base_link_site_pose_sensor/pose', self._pose_cb, 10)

        # Timer
        self.timer = self.create_timer(1.0 / rate, self._timer_cb)
        self.get_logger().info('MuJoCo LiDAR node (mj_multiRay) started')

    # ------------------------------------------------------------------
    def _pose_cb(self, msg: PoseStamped):
        with self._pose_lock:
            self._latest_pose = msg

    # ------------------------------------------------------------------
    def _joint_state_cb(self, msg: JointState):
        with self._js_lock:
            self._latest_js = msg

    # ------------------------------------------------------------------
    def _timer_cb(self):
        # 1. Update qpos from latest joint_states
        with self._js_lock:
            js = self._latest_js
        if js is None:
            return

        # 1b. Sync free joint from ground-truth pose
        with self._pose_lock:
            pose = self._latest_pose

        with self._mj_lock:
            if pose is not None and self._free_jnt_qpos_adr is not None:
                a = self._free_jnt_qpos_adr
                p = pose.pose.position
                q = pose.pose.orientation
                # MuJoCo free joint qpos: [x, y, z, qw, qx, qy, qz]
                self.mj_data.qpos[a:a+7] = [p.x, p.y, p.z, q.w, q.x, q.y, q.z]

            for i, name in enumerate(js.name):
                if name in self._joint_map:
                    self.mj_data.qpos[self._joint_map[name]] = js.position[i]

            # 2. Forward kinematics (no dynamics)
            mujoco.mj_forward(self.mj_model, self.mj_data)

            # 3. Get LiDAR origin and orientation from site (no hardcoded offsets)
            lidar_origin = self.mj_data.site_xpos[self.lidar_site_id].copy()   # (3,)
            lidar_mat = self.mj_data.site_xmat[self.lidar_site_id].reshape(3, 3).copy()  # (3,3)

            # 4. Rotate ray directions to world frame using site orientation
            ray_dirs_world = (lidar_mat @ self._ray_dirs_local.T).T  # (N, 3)

            # 5. Cast all rays via single mj_multiRay call
            pnt = lidar_origin.reshape(3, 1)       # (3, 1) column vector
            vec = ray_dirs_world.ravel()            # (n_rays*3,) flat

            self._ray_dist[:] = self.range_max
            self._ray_geomid[:] = -1

            mujoco.mj_multiRay(
                m=self.mj_model,
                d=self.mj_data,
                pnt=pnt,
                vec=vec,
                geomgroup=self._geomgroup,   # None = all groups
                flg_static=1,                # include static geoms
                bodyexclude=self.lidar_body_id,
                geomid=self._ray_geomid,
                dist=self._ray_dist,
                normal=None,
                nray=self.n_rays,
                cutoff=self.range_max,
            )

        # Post-processing outside the lock (pure NumPy, no MjData access)
        valid = ((self._ray_geomid != -1)
                 & (self._ray_dist >= self.range_min)
                 & (self._ray_dist <= self.range_max))
        hit_dirs = ray_dirs_world[valid]
        hit_dist = self._ray_dist[valid, np.newaxis]
        points_world = (hit_dirs * hit_dist).astype(np.float32)

        # 5b. Rotate hit points from world frame back into livox_mid360 frame
        #     so they match frame_id (livox_mid360).
        if len(points_world) > 0:
            points = (lidar_mat.T @ points_world.astype(np.float64).T).T.astype(np.float32)
        else:
            points = points_world

        # 6. Stamp AFTER raycasting — guarantees lidar timestamp >= any IMU
        #    timestamps dispatched during the mj_multiRay() call.
        #    Monotonicity guard drops if sim time hasn't advanced.
        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= self._last_stamp_ns:
            return  # sim clock hasn't advanced
        self._last_stamp_ns = now_ns
        self._pending_stamp = self.get_clock().now().to_msg()

        # 7. Publish PointCloud2
        if len(points) > 0:
            msg = self._make_pointcloud2(points)
            self.pc_pub.publish(msg)

    # ------------------------------------------------------------------
    def _make_pointcloud2(self, points: np.ndarray) -> PointCloud2:
        """Create a PointCloud2 message from Nx3 float32 array."""
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self._pending_stamp
        msg.header.frame_id = self.frame_id

        msg.height = 1
        msg.width = len(points)
        msg.is_bigendian = False
        msg.point_step = 12  # 3 x float32
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        msg.data = points.tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = MujocoLidarMultiRayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
