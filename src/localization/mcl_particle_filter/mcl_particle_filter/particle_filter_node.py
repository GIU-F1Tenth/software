#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import range_libc
import time
from threading import Lock
from tf_transformations import quaternion_from_euler
import tf2_ros
from . import utils as Utils

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseArray, PolygonStamped, PoseWithCovarianceStamped, PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap

VAR_NO_EVAL_SENSOR_MODEL = 0
VAR_CALC_RANGE_MANY_EVAL_SENSOR = 1
VAR_REPEAT_ANGLES_EVAL_SENSOR = 2
VAR_REPEAT_ANGLES_EVAL_SENSOR_ONE_SHOT = 3
VAR_RADIAL_CDDT_OPTIMIZATIONS = 4


class AmclParticleFilter(Node):

    def __init__(self):
        super().__init__('particle_filter')

        self.declare_parameter('angle_step', 18)
        self.declare_parameter('min_particles', 500)
        self.declare_parameter('max_particles', 4000)
        self.declare_parameter('max_viz_particles', 60)
        self.declare_parameter('squash_factor', 2.2)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('theta_discretization', 112)
        self.declare_parameter('range_method', 'cddt')
        self.declare_parameter('rangelib_variant', 3)
        self.declare_parameter('fine_timing', False)
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('viz', True)

        self.declare_parameter('z_short', 0.01)
        self.declare_parameter('z_max', 0.07)
        self.declare_parameter('z_rand', 0.12)
        self.declare_parameter('z_hit', 0.75)
        self.declare_parameter('sigma_hit', 8.0)

        self.declare_parameter('alpha_1', 0.5)
        self.declare_parameter('alpha_2', 0.015)
        self.declare_parameter('alpha_3', 1.0)
        self.declare_parameter('alpha_4', 0.5)
        self.declare_parameter('lam_thresh', 0.1)
        self.declare_parameter('min_trans_update', 0.01)
        self.declare_parameter('reverse_velocity_thresh', -0.05)
        self.declare_parameter('reverse_spread', 1.1)

        self.declare_parameter('kld_err', 0.05)
        self.declare_parameter('kld_z', 0.99)
        self.declare_parameter('kld_bin_size_xy', 0.5)
        self.declare_parameter('kld_bin_size_theta', 0.1745)
        self.declare_parameter('alpha_slow', 0.001)
        self.declare_parameter('alpha_fast', 0.1)
        self.declare_parameter('resample_interval', 1)

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odometry_topic', '/odom')
        self.declare_parameter('static_map', 'static_map')

        self.ANGLE_STEP = int(self.get_parameter('angle_step').value)
        self.MIN_PARTICLES = int(self.get_parameter('min_particles').value)
        self.MAX_PARTICLES = int(self.get_parameter('max_particles').value)
        self.MAX_VIZ_PARTICLES = int(self.get_parameter('max_viz_particles').value)
        self.INV_SQUASH_FACTOR = 1.0 / float(self.get_parameter('squash_factor').value)
        self.MAX_RANGE_METERS = float(self.get_parameter('max_range').value)
        self.THETA_DISCRETIZATION = int(self.get_parameter('theta_discretization').value)
        self.WHICH_RM = str(self.get_parameter('range_method').value).lower()
        self.RANGELIB_VAR = int(self.get_parameter('rangelib_variant').value)
        self.SHOW_FINE_TIMING = bool(self.get_parameter('fine_timing').value)
        self.PUBLISH_ODOM = bool(self.get_parameter('publish_odom').value)
        self.DO_VIZ = bool(self.get_parameter('viz').value)

        self.Z_SHORT = float(self.get_parameter('z_short').value)
        self.Z_MAX = float(self.get_parameter('z_max').value)
        self.Z_RAND = float(self.get_parameter('z_rand').value)
        self.Z_HIT = float(self.get_parameter('z_hit').value)
        self.SIGMA_HIT = float(self.get_parameter('sigma_hit').value)

        self.ALPHA_1 = float(self.get_parameter('alpha_1').value)
        self.ALPHA_2 = float(self.get_parameter('alpha_2').value)
        self.ALPHA_3 = float(self.get_parameter('alpha_3').value)
        self.ALPHA_4 = float(self.get_parameter('alpha_4').value)
        self.LAM_THRESH = float(self.get_parameter('lam_thresh').value)
        self.MIN_TRANS_UPDATE = float(self.get_parameter('min_trans_update').value)
        self.REVERSE_VEL_THRESH = float(self.get_parameter('reverse_velocity_thresh').value)
        self.REVERSE_SPREAD = float(self.get_parameter('reverse_spread').value)

        self.KLD_ERR = float(self.get_parameter('kld_err').value)
        self.KLD_Z = float(self.get_parameter('kld_z').value)
        self.KLD_BIN_SIZE_XY = float(self.get_parameter('kld_bin_size_xy').value)
        self.KLD_BIN_SIZE_THETA = float(self.get_parameter('kld_bin_size_theta').value)
        self.ALPHA_SLOW = float(self.get_parameter('alpha_slow').value)
        self.ALPHA_FAST = float(self.get_parameter('alpha_fast').value)
        self.RESAMPLE_INTERVAL = int(self.get_parameter('resample_interval').value)

        self.MAX_RANGE_PX = None
        self.iters = 0
        self.resample_counter = 0
        self.map_info = None
        self.map_initialized = False
        self.lidar_initialized = False
        self.odom_initialized = False
        self.last_pose = None
        self.curr_pose = None
        self.last_linear_x = 0.0
        self.laser_angles = None
        self.downsampled_angles = None
        self.downsampled_ranges = None
        self.range_method = None
        self.last_stamp = None
        self.state_lock = Lock()

        self.queries = None
        self.ranges = None
        self.tiled_angles = None
        self.sensor_model_table = None
        self._buffer_n = 0

        self.w_slow = 0.0
        self.w_fast = 0.0

        self.inferred_pose = None
        self.particles = np.zeros((self.MAX_PARTICLES, 3))
        self.weights = np.ones(self.MAX_PARTICLES) / float(self.MAX_PARTICLES)

        self.smoothing = Utils.CircularArray(10)
        self.timer = Utils.Timer(10)
        self.get_omap()
        self.precompute_sensor_model()
        self.initialize_global()

        self.pose_pub = self.create_publisher(PoseStamped, '/pf/viz/inferred_pose', 1)
        self.particle_pub = self.create_publisher(PoseArray, '/pf/viz/particles', 1)
        self.pub_fake_scan = self.create_publisher(LaserScan, '/pf/viz/fake_scan', 1)
        self.rect_pub = self.create_publisher(PolygonStamped, '/pf/viz/poly1', 1)

        if self.PUBLISH_ODOM:
            self.odom_pub = self.create_publisher(Odometry, '/pf/pose/odom', 1)

        self.pub_tf = tf2_ros.TransformBroadcaster(self)

        scan_topic = self.get_parameter('scan_topic').value
        odom_topic = self.get_parameter('odometry_topic').value

        self.laser_sub = self.create_subscription(LaserScan, scan_topic, self.lidarCB, 1)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odomCB, 1)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.clicked_pose, 1)
        self.click_sub = self.create_subscription(PointStamped, '/clicked_point', self.clicked_pose, 1)

        self.get_logger().info("AMCL particle filter initialized, waiting on messages...")

    def get_omap(self):
        map_service_name = self.get_parameter('static_map').value
        map_client = self.create_client(GetMap, map_service_name)
        while not map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for map service...')
        future = map_client.call_async(GetMap.Request())
        rclpy.spin_until_future_complete(self, future)
        map_msg = future.result().map

        self.map_info = map_msg.info
        oMap = range_libc.PyOMap(map_msg)
        self.MAX_RANGE_PX = int(self.MAX_RANGE_METERS / self.map_info.resolution)

        self.get_logger().info(f"Initializing range method: {self.WHICH_RM}")
        if self.WHICH_RM == "bl":
            self.range_method = range_libc.PyBresenhamsLine(oMap, self.MAX_RANGE_PX)
        elif "cddt" in self.WHICH_RM:
            self.range_method = range_libc.PyCDDTCast(oMap, self.MAX_RANGE_PX, self.THETA_DISCRETIZATION)
            if self.WHICH_RM == "pcddt":
                self.get_logger().info("Pruning...")
                self.range_method.prune()
        elif self.WHICH_RM == "rm":
            self.range_method = range_libc.PyRayMarching(oMap, self.MAX_RANGE_PX)
        elif self.WHICH_RM == "rmgpu":
            self.range_method = range_libc.PyRayMarchingGPU(oMap, self.MAX_RANGE_PX)
        elif self.WHICH_RM == "glt":
            self.range_method = range_libc.PyGiantLUTCast(oMap, self.MAX_RANGE_PX, self.THETA_DISCRETIZATION)

        array_255 = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))

        self.permissible_region = np.zeros_like(array_255, dtype=bool)
        self.permissible_region[array_255 == 0] = 1
        self._perm_x, self._perm_y = np.where(self.permissible_region == 1)
        self.map_initialized = True

    def publish_tf(self, pose, stamp=None):
        if stamp is None:
            stamp = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = float(pose[0])
        t.transform.translation.y = float(pose[1])
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, float(pose[2]))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.pub_tf.sendTransform(t)

        if self.PUBLISH_ODOM:
            odom = Odometry()
            odom.header = Utils.make_header('map', stamp)
            odom.pose.pose.position.x = float(pose[0])
            odom.pose.pose.position.y = float(pose[1])
            odom.pose.pose.orientation = Utils.angle_to_quaternion(float(pose[2]))
            self.odom_pub.publish(odom)

    def visualize(self):
        if not self.DO_VIZ:
            return

        if self.pose_pub.get_subscription_count() > 0 and isinstance(self.inferred_pose, np.ndarray):
            ps = PoseStamped()
            ps.header = Utils.make_header('map', self.get_clock().now().to_msg())
            ps.pose.position.x = float(self.inferred_pose[0])
            ps.pose.position.y = float(self.inferred_pose[1])
            ps.pose.orientation = Utils.angle_to_quaternion(float(self.inferred_pose[2]))
            self.pose_pub.publish(ps)

        if self.particle_pub.get_subscription_count() > 0:
            n = self.particles.shape[0]
            if n > self.MAX_VIZ_PARTICLES:
                idx = np.random.choice(np.arange(n), self.MAX_VIZ_PARTICLES, p=self.weights)
                self.publish_particles(self.particles[idx, :])
            else:
                self.publish_particles(self.particles)

        if self.pub_fake_scan.get_subscription_count() > 0 and isinstance(self.ranges, np.ndarray):
            self.viz_queries[:, 0] = float(self.inferred_pose[0])
            self.viz_queries[:, 1] = float(self.inferred_pose[1])
            self.viz_queries[:, 2] = self.downsampled_angles + float(self.inferred_pose[2])
            self.range_method.calc_range_many(self.viz_queries, self.viz_ranges)
            self.publish_scan(self.downsampled_angles, self.viz_ranges)

    def publish_particles(self, particles):
        pa = PoseArray()
        pa.header = Utils.make_header('map', self.get_clock().now().to_msg())
        pa.poses = Utils.particles_to_poses(particles)
        self.particle_pub.publish(pa)

    def publish_scan(self, angles, ranges):
        ls = LaserScan()
        ls.header = Utils.make_header('laser', stamp=self.last_stamp)
        ls.angle_min = float(np.min(angles))
        ls.angle_max = float(np.max(angles))
        ls.angle_increment = float(np.abs(angles[0] - angles[1]))
        ls.range_min = 0.0
        ls.range_max = float(np.max(ranges))
        ls.ranges = list(ranges)
        self.pub_fake_scan.publish(ls)

    def lidarCB(self, msg):
        if not isinstance(self.laser_angles, np.ndarray):
            self.laser_angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
            self.downsampled_angles = np.copy(self.laser_angles[0::self.ANGLE_STEP]).astype(np.float32)
            self.viz_queries = np.zeros((self.downsampled_angles.shape[0], 3), dtype=np.float32)
            self.viz_ranges = np.zeros(self.downsampled_angles.shape[0], dtype=np.float32)

        self.downsampled_ranges = np.array(msg.ranges[::self.ANGLE_STEP])
        self.lidar_initialized = True

    def odomCB(self, msg):
        position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        orientation = Utils.quaternion_to_angle(msg.pose.pose.orientation)
        pose = np.array([position[0], position[1], orientation])
        self.last_linear_x = float(msg.twist.twist.linear.x)
        self.last_stamp = msg.header.stamp

        if isinstance(self.last_pose, np.ndarray):
            self.curr_pose = pose
            self.odom_initialized = True
            self.update()
            self.last_pose = pose
        else:
            self.last_pose = pose

    def clicked_pose(self, msg):
        if isinstance(msg, PointStamped):
            self.initialize_global()
        elif isinstance(msg, PoseWithCovarianceStamped):
            self.initialize_particles_pose(msg.pose.pose)

    def initialize_particles_pose(self, pose):
        with self.state_lock:
            n = self.MAX_PARTICLES
            self.particles = np.zeros((n, 3))
            self.particles[:, 0] = pose.position.x + np.random.normal(loc=0.0, scale=0.5, size=n)
            self.particles[:, 1] = pose.position.y + np.random.normal(loc=0.0, scale=0.5, size=n)
            self.particles[:, 2] = Utils.quaternion_to_angle(pose.orientation) + np.random.normal(loc=0.0, scale=0.4, size=n)
            self.weights = np.ones(n) / float(n)
            self.w_slow = 0.0
            self.w_fast = 0.0

    def initialize_global(self):
        with self.state_lock:
            n = self.MAX_PARTICLES
            indices = np.random.randint(0, len(self._perm_x), size=n)
            states = np.zeros((n, 3))
            states[:, 0] = self._perm_y[indices]
            states[:, 1] = self._perm_x[indices]
            states[:, 2] = np.random.random(n) * np.pi * 2.0
            Utils.map_to_world(states, self.map_info)
            self.particles = states
            self.weights = np.ones(n) / float(n)
            self.w_slow = 0.0
            self.w_fast = 0.0

    def random_pose_world(self):
        idx = np.random.randint(0, len(self._perm_x))
        state = np.zeros((1, 3))
        state[0, 0] = self._perm_y[idx]
        state[0, 1] = self._perm_x[idx]
        state[0, 2] = np.random.random() * 2.0 * np.pi
        Utils.map_to_world(state, self.map_info)
        return state[0]

    def precompute_sensor_model(self):
        z_short, z_max, z_rand, z_hit, sigma_hit = self.Z_SHORT, self.Z_MAX, self.Z_RAND, self.Z_HIT, self.SIGMA_HIT
        table_width = int(self.MAX_RANGE_PX) + 1
        self.sensor_model_table = np.zeros((table_width, table_width))

        for d in range(table_width):
            norm = 0.0
            for r in range(table_width):
                prob = 0.0
                z = float(r - d)
                prob += z_hit * np.exp(-(z * z) / (2.0 * sigma_hit * sigma_hit)) / (sigma_hit * np.sqrt(2.0 * np.pi))
                if r < d:
                    prob += 2.0 * z_short * (d - r) / float(d)
                if int(r) == int(self.MAX_RANGE_PX):
                    prob += z_max
                if r < int(self.MAX_RANGE_PX):
                    prob += z_rand * 1.0 / float(self.MAX_RANGE_PX)
                norm += prob
                self.sensor_model_table[int(r), int(d)] = prob
            self.sensor_model_table[:, int(d)] /= norm

        if self.RANGELIB_VAR > 0:
            self.range_method.set_sensor_model(self.sensor_model_table)

    def motion_model_tum(self, proposal_dist):
        a1, a2, a3, a4 = self.ALPHA_1, self.ALPHA_2, self.ALPHA_3, self.ALPHA_4

        dx = self.curr_pose[0] - self.last_pose[0]
        dy = self.curr_pose[1] - self.last_pose[1]
        dtheta = Utils.angle_diff(self.curr_pose[2], self.last_pose[2])
        d_trans = float(np.sqrt(dx * dx + dy * dy))

        if d_trans < self.MIN_TRANS_UPDATE:
            return

        d_rot1 = Utils.angle_diff(np.arctan2(dy, dx), self.last_pose[2])

        reverse_offset = 0.0
        reverse_spread = 1.0
        if self.last_linear_x < self.REVERSE_VEL_THRESH:
            reverse_offset = np.pi
            reverse_spread = self.REVERSE_SPREAD
            d_rot1 += np.pi if d_rot1 < -np.pi / 2 else -np.pi

        d_rot2 = Utils.angle_diff(dtheta, d_rot1)

        d_rot1 = min(abs(Utils.angle_diff(d_rot1, 0.0)), abs(Utils.angle_diff(d_rot1, np.pi)))
        d_rot2 = min(abs(Utils.angle_diff(d_rot2, 0.0)), abs(Utils.angle_diff(d_rot2, np.pi)))

        scale_rot1 = (a1 * d_rot1 + a2 / max(d_trans, self.LAM_THRESH)) * reverse_spread
        scale_rot2 = (a1 * d_rot2 + a2 / max(d_trans, self.LAM_THRESH)) * reverse_spread
        scale_trans = (a3 * d_trans + a4 * (d_rot1 + d_rot2)) * reverse_spread

        n = proposal_dist.shape[0]
        sampled_rot1 = d_rot1 + np.random.normal(scale=scale_rot1, size=n)
        sampled_trans = d_trans + np.random.normal(loc=scale_trans / 2.0, scale=scale_trans, size=n)
        sampled_rot2 = d_rot2 + np.random.normal(scale=scale_rot2, size=n)

        eff_hdg = proposal_dist[:, 2] + sampled_rot1 + reverse_offset
        proposal_dist[:, 0] += sampled_trans * np.cos(eff_hdg)
        proposal_dist[:, 1] += sampled_trans * np.sin(eff_hdg)
        proposal_dist[:, 2] = np.arctan2(
            np.sin(proposal_dist[:, 2] + sampled_rot1 + sampled_rot2),
            np.cos(proposal_dist[:, 2] + sampled_rot1 + sampled_rot2))

    def _ensure_buffers(self, n):
        num_rays = self.downsampled_angles.shape[0]
        if self._buffer_n == n and self.queries is not None:
            return num_rays
        if self.RANGELIB_VAR <= 1:
            self.queries = np.zeros((num_rays * n, 3), dtype=np.float32)
        else:
            self.queries = np.zeros((n, 3), dtype=np.float32)
        self.ranges = np.zeros(num_rays * n, dtype=np.float32)
        self.tiled_angles = np.tile(self.downsampled_angles, n)
        self._buffer_n = n
        return num_rays

    def sensor_model(self, proposal_dist, obs, weights):
        n = proposal_dist.shape[0]
        num_rays = self._ensure_buffers(n)

        if self.RANGELIB_VAR == VAR_RADIAL_CDDT_OPTIMIZATIONS and "cddt" in self.WHICH_RM:
            self.queries[:, :] = proposal_dist[:, :]
            self.range_method.calc_range_many_radial_optimized(
                num_rays, self.downsampled_angles[0], self.downsampled_angles[-1], self.queries, self.ranges)
            self.range_method.eval_sensor_model(obs, self.ranges, weights, num_rays, n)
            np.power(weights, self.INV_SQUASH_FACTOR, weights)
        elif self.RANGELIB_VAR == VAR_REPEAT_ANGLES_EVAL_SENSOR_ONE_SHOT:
            self.queries[:, :] = proposal_dist[:, :]
            self.range_method.calc_range_repeat_angles_eval_sensor_model(
                self.queries, self.downsampled_angles, obs, weights)
            np.power(weights, self.INV_SQUASH_FACTOR, weights)
        elif self.RANGELIB_VAR == VAR_REPEAT_ANGLES_EVAL_SENSOR:
            self.queries[:, :] = proposal_dist[:, :]
            self.range_method.calc_range_repeat_angles(self.queries, self.downsampled_angles, self.ranges)
            self.range_method.eval_sensor_model(obs, self.ranges, weights, num_rays, n)
            np.power(weights, self.INV_SQUASH_FACTOR, weights)
        elif self.RANGELIB_VAR == VAR_CALC_RANGE_MANY_EVAL_SENSOR:
            self.queries[:, 0] = np.repeat(proposal_dist[:, 0], num_rays)
            self.queries[:, 1] = np.repeat(proposal_dist[:, 1], num_rays)
            self.queries[:, 2] = np.repeat(proposal_dist[:, 2], num_rays) + self.tiled_angles
            self.range_method.calc_range_many(self.queries, self.ranges)
            self.range_method.eval_sensor_model(obs, self.ranges, weights, num_rays, n)
            np.power(weights, self.INV_SQUASH_FACTOR, weights)
        elif self.RANGELIB_VAR == VAR_NO_EVAL_SENSOR_MODEL:
            self.queries[:, 0] = np.repeat(proposal_dist[:, 0], num_rays)
            self.queries[:, 1] = np.repeat(proposal_dist[:, 1], num_rays)
            self.queries[:, 2] = np.repeat(proposal_dist[:, 2], num_rays) + self.tiled_angles
            self.range_method.calc_range_many(self.queries, self.ranges)

            obs_px = obs / float(self.map_info.resolution)
            ranges_px = self.ranges / float(self.map_info.resolution)
            obs_px[obs_px > self.MAX_RANGE_PX] = self.MAX_RANGE_PX
            ranges_px[ranges_px > self.MAX_RANGE_PX] = self.MAX_RANGE_PX
            intobs = np.rint(obs_px).astype(np.uint16)
            intrng = np.rint(ranges_px).astype(np.uint16)
            for i in range(n):
                w = np.product(self.sensor_model_table[intobs, intrng[i * num_rays:(i + 1) * num_rays]])
                weights[i] = np.power(w, self.INV_SQUASH_FACTOR)
        else:
            self.get_logger().warn("PLEASE SET rangelib_variant PARAM to 0-4")

    def kld_sample_size(self, k):
        if k <= 1:
            return self.MIN_PARTICLES
        b = 2.0 / (9.0 * (k - 1))
        n = ((k - 1) / (2.0 * self.KLD_ERR)) * (1.0 - b + np.sqrt(b) * self.KLD_Z) ** 3
        return int(np.clip(n, self.MIN_PARTICLES, self.MAX_PARTICLES))

    def kld_resample(self):
        w_diff = 0.0
        if self.w_slow > 1e-12:
            w_diff = max(0.0, 1.0 - self.w_fast / self.w_slow)

        cumulative = np.cumsum(self.weights)
        if cumulative[-1] <= 0.0:
            self.weights[:] = 1.0 / len(self.weights)
            cumulative = np.cumsum(self.weights)
        cumulative[-1] = 1.0

        new_particles = np.empty((self.MAX_PARTICLES, 3))
        occupied = set()
        n = 0
        target = self.MIN_PARTICLES

        while n < target and n < self.MAX_PARTICLES:
            if np.random.random() < w_diff:
                new_particles[n] = self.random_pose_world()
            else:
                idx = int(np.searchsorted(cumulative, np.random.random()))
                idx = min(idx, len(self.particles) - 1)
                new_particles[n] = self.particles[idx]

            bin_x = int(np.floor(new_particles[n, 0] / self.KLD_BIN_SIZE_XY))
            bin_y = int(np.floor(new_particles[n, 1] / self.KLD_BIN_SIZE_XY))
            bin_t = int(np.floor(new_particles[n, 2] / self.KLD_BIN_SIZE_THETA))
            key = (bin_x, bin_y, bin_t)
            if key not in occupied:
                occupied.add(key)
                if len(occupied) > 1:
                    target = self.kld_sample_size(len(occupied))
            n += 1

        if w_diff > 0.0:
            self.w_slow = 0.0
            self.w_fast = 0.0

        self.particles = new_particles[:n]
        self.weights = np.ones(n) / float(n)

    def AMCL(self):
        self.motion_model_tum(self.particles)
        observation = np.copy(self.downsampled_ranges).astype(np.float32)
        self.sensor_model(self.particles, observation, self.weights)

        w_avg = float(np.mean(self.weights)) if self.weights.size > 0 else 0.0
        if self.w_slow == 0.0 and self.w_fast == 0.0:
            self.w_slow = w_avg
            self.w_fast = w_avg
        else:
            self.w_slow += self.ALPHA_SLOW * (w_avg - self.w_slow)
            self.w_fast += self.ALPHA_FAST * (w_avg - self.w_fast)

        w_sum = float(np.sum(self.weights))
        if w_sum > 0.0:
            self.weights /= w_sum
        else:
            self.weights[:] = 1.0 / float(self.weights.size)

        self.resample_counter += 1
        if self.resample_counter >= self.RESAMPLE_INTERVAL:
            self.kld_resample()
            self.resample_counter = 0

    def expected_pose(self):
        x = float(np.dot(self.particles[:, 0], self.weights))
        y = float(np.dot(self.particles[:, 1], self.weights))
        sin_theta = float(np.dot(np.sin(self.particles[:, 2]), self.weights))
        cos_theta = float(np.dot(np.cos(self.particles[:, 2]), self.weights))
        theta = float(np.arctan2(sin_theta, cos_theta))
        return np.array([x, y, theta])

    def update(self):
        if not (self.lidar_initialized and self.odom_initialized and self.map_initialized):
            return
        if self.state_lock.locked():
            return

        with self.state_lock:
            self.timer.tick()
            self.iters += 1
            t1 = time.time()

            self.AMCL()
            self.inferred_pose = self.expected_pose()

            t2 = time.time()

        self.publish_tf(self.inferred_pose, self.last_stamp)
        self.smoothing.append(1.0 / max(t2 - t1, 1e-9))
        self.visualize()


def main():
    rclpy.init()
    pf = AmclParticleFilter()
    try:
        rclpy.spin(pf)
    except KeyboardInterrupt:
        pass
    finally:
        pf.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
