"""MuJoCo contact sensor bridge node.

Loads a copy of the MuJoCo model, subscribes to /joint_states to sync the
kinematic state, then runs forward dynamics + collision detection (via
mujoco.mj_forward) and inspects the contact list to determine foot
contact.  Publishes champ_msgs/ContactsStamped at 50 Hz on
/{ns}/foot_contacts.

Why contact-list rather than touch sensors:
  The MJCF defines `<touch>` sensors at sites named FL_foot_site, …
  (size 0.01 m, sitting ~0.013 m above the ground contact point under
  the home keyframe).  The contact pos is therefore *outside* the
  touch sphere, so the sensor reads 0 even when the foot is firmly
  planted (verified 2026-05-02 on demo1_go2_real.xml).  Reading
  d.contact[:d.ncon] directly is robust to site geometry and gives
  per-foot booleans without needing constraint forces — mj_forward
  already populates the contact list.

Foot identification:
  Foot collision geoms are unprefixed (FL/FR/RL/RR for Go2,
  FL_wheel_collision/etc for Go2W).  The other side of every
  ground-contact pair is the world `ground` geom.  We match by the
  body that owns the foot geom: FL_calf / FL_wheel — anything on the
  per-leg subtree counts as that leg's contact.
"""

import threading

import mujoco
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from champ_msgs.msg import ContactsStamped
from ament_index_python.packages import get_package_share_directory


# Names of the 4 touch sensors in the MJCF (kept for backward compat with
# the param API; the actual contact detection now runs against the body
# tree below).
_DEFAULT_TOUCH_SENSORS = [
    'FL_foot_contact',
    'FR_foot_contact',
    'RL_foot_contact',
    'RR_foot_contact',
]

# Per-leg root bodies in the MJCF.  Any contact on a geom whose body is
# in (or descended from) one of these counts as that leg's contact.
# Order must match CHAMP convention: [FL, FR, RL, RR].
_LEG_ROOT_BODIES = ['FL_calf', 'FR_calf', 'RL_calf', 'RR_calf']


class MujocoContactNode(Node):

    def __init__(self):
        super().__init__('mujoco_contact_node',
                         parameter_overrides=[rclpy.Parameter('use_sim_time', value=True)])

        # --- Parameters ---
        self.declare_parameter('mjcf_path', '')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('contact_threshold', 0.01)
        self.declare_parameter('touch_sensor_names', _DEFAULT_TOUCH_SENSORS)

        mjcf_path = self.get_parameter('mjcf_path').get_parameter_value().string_value
        if not mjcf_path:
            try:
                pkg_dir = get_package_share_directory('go2_gazebo_sim')
                mjcf_path = pkg_dir + '/mujoco/go2w.xml'
            except Exception:
                self.get_logger().fatal('mjcf_path not set and go2_gazebo_sim not found')
                raise RuntimeError('mjcf_path is required')

        rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.contact_threshold = self.get_parameter(
            'contact_threshold').get_parameter_value().double_value
        sensor_names = self.get_parameter(
            'touch_sensor_names').get_parameter_value().string_array_value

        # --- Load MuJoCo model ---
        self.get_logger().info(f'Loading MuJoCo model from: {mjcf_path}')
        self.mj_model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.mj_data = mujoco.MjData(self.mj_model)
        self._mj_lock = threading.Lock()

        # Resolve per-leg body subtrees. Each entry is the set of body ids
        # in the calf-root subtree; any contact whose geom belongs to a
        # body in this set counts as that leg's contact. Captures both
        # Go2 (foot geom on calf) and Go2W (wheel geom on a child of calf).
        self._leg_body_sets = []
        for root_name in _LEG_ROOT_BODIES:
            root_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, root_name)
            if root_id < 0:
                self.get_logger().warn(
                    f'Leg root body "{root_name}" not found; that leg will report no contact')
                self._leg_body_sets.append(set())
                continue
            subtree = set()
            for bid in range(self.mj_model.nbody):
                # Walk parent chain; if we hit root_id it's in the subtree.
                cur = bid
                while cur > 0:
                    if cur == root_id:
                        subtree.add(bid)
                        break
                    cur = self.mj_model.body_parentid[cur]
            self._leg_body_sets.append(subtree)
            self.get_logger().info(
                f'Leg "{root_name}": subtree size={len(subtree)} body(s)')

        # Pre-build geom→leg-index map: faster than walking each contact.
        self._geom_to_leg = {}
        for geom_id in range(self.mj_model.ngeom):
            body_id = self.mj_model.geom_bodyid[geom_id]
            for leg_idx, body_set in enumerate(self._leg_body_sets):
                if body_id in body_set:
                    self._geom_to_leg[geom_id] = leg_idx
                    break

        # Apply the MJCF's "home" keyframe so the free-joint base starts
        # at the standing pose (z≈0.27) instead of qpos=0 (body at world
        # origin, legs penetrating ground hard). The contact node never
        # subscribes to TF for the base pose, so without this the leg
        # geometry is wildly wrong on startup; even with leg-joint sync
        # from /joint_states, feet wouldn't be near the floor.
        key_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_KEY, 'home')
        if key_id >= 0:
            mujoco.mj_resetDataKeyframe(self.mj_model, self.mj_data, key_id)
            self.get_logger().info('Initialized to "home" keyframe')

        # Build joint name -> qpos index mapping (non-free joints only;
        # the free-root joint is synced separately from base odometry).
        self._joint_map = {}
        self._free_qpos_adr = -1
        for i in range(self.mj_model.njnt):
            name = mujoco.mj_id2name(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if not name:
                continue
            qpos_adr = self.mj_model.jnt_qposadr[i]
            if self.mj_model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE:
                # Free joint occupies 7 qpos entries: [x y z qw qx qy qz]
                self._free_qpos_adr = qpos_adr
            else:
                self._joint_map[name] = qpos_adr

        # --- ROS pub/sub ---
        self.contacts_pub = self.create_publisher(ContactsStamped, 'foot_contacts', 10)

        self.js_sub = self.create_subscription(
            JointState, 'joint_states', self._joint_state_cb, 10)

        # Base odometry sync. Without this the free joint stays at the
        # home-keyframe pose (z=0.27 stand) regardless of where the real
        # body actually is. During gait the body rocks ±2-3 cm vertically
        # and the leg angles by themselves don't tell whether a foot is
        # supposed to be airborne — the base z determines that. Sim gets
        # GT odom at /<ns>/odom/ground_truth; real robot would map to a
        # different topic but currently this node is sim-only anyway.
        self._latest_odom = None
        self._odom_lock = threading.Lock()
        self.odom_sub = self.create_subscription(
            Odometry, 'odom/ground_truth', self._odom_cb, 10)

        self._latest_js = None
        self._js_lock = threading.Lock()

        self.timer = self.create_timer(1.0 / rate, self._timer_cb)
        self.get_logger().info(f'MuJoCo contact node started at {rate} Hz')

    # ------------------------------------------------------------------
    def _joint_state_cb(self, msg: JointState):
        with self._js_lock:
            self._latest_js = msg

    def _odom_cb(self, msg: Odometry):
        with self._odom_lock:
            self._latest_odom = msg

    # ------------------------------------------------------------------
    def _timer_cb(self):
        with self._js_lock:
            js = self._latest_js
        if js is None:
            return

        # Snapshot odom (base pose) — fall back to keyframe pose if not
        # yet received.
        with self._odom_lock:
            odom = self._latest_odom

        # Sync joint state + base pose into MuJoCo model.
        with self._mj_lock:
            for i, name in enumerate(js.name):
                if name in self._joint_map:
                    self.mj_data.qpos[self._joint_map[name]] = js.position[i]
            if odom is not None and self._free_qpos_adr >= 0:
                p = odom.pose.pose.position
                q = odom.pose.pose.orientation
                a = self._free_qpos_adr
                # MuJoCo free joint qpos layout: [x y z qw qx qy qz]
                self.mj_data.qpos[a + 0] = p.x
                self.mj_data.qpos[a + 1] = p.y
                self.mj_data.qpos[a + 2] = p.z
                self.mj_data.qpos[a + 3] = q.w
                self.mj_data.qpos[a + 4] = q.x
                self.mj_data.qpos[a + 5] = q.y
                self.mj_data.qpos[a + 6] = q.z
            # Zero velocity — we don't track qvel and don't want spurious
            # bias terms in the constraint pipeline.
            self.mj_data.qvel[:] = 0

            # mj_forward runs kinematics + collision detection. d.contact
            # is populated with all detected interpenetrations after this
            # call; we don't need the constraint solver's force values
            # because we're using the contact list directly.
            mujoco.mj_forward(self.mj_model, self.mj_data)

            # Per-leg contact: True if any contact pair has one side on
            # this leg's body subtree.
            leg_in_contact = [False, False, False, False]
            for k in range(self.mj_data.ncon):
                c = self.mj_data.contact[k]
                for gid in (c.geom1, c.geom2):
                    leg_idx = self._geom_to_leg.get(gid)
                    if leg_idx is not None:
                        leg_in_contact[leg_idx] = True
            contacts = leg_in_contact

        # Publish
        msg = ContactsStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.contacts = contacts
        self.contacts_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MujocoContactNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
