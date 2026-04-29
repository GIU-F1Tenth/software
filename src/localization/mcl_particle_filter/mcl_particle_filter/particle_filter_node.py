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


class ParticleFiler(Node):

    def __init__(self):
        super().__init__('particle_filter')

        self.declare_parameter('angle_step', 18)
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
        self.declare_parameter('motion_dispersion_x', 0.05)
        self.declare_parameter('motion_dispersion_y', 0.025)
        self.declare_parameter('motion_dispersion_theta', 0.25)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odometry_topic', '/odom')
        self.declare_parameter('static_map', 'static_map')

        self.ANGLE_STEP = int(self.get_parameter('angle_step').value)
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

        self.MOTION_DISPERSION_X = float(self.get_parameter('motion_dispersion_x').value)
        self.MOTION_DISPERSION_Y = float(self.get_parameter('motion_dispersion_y').value)
        self.MOTION_DISPERSION_THETA = float(self.get_parameter('motion_dispersion_theta').value)

        self.MAX_RANGE_PX = None
        self.odometry_data = np.array([0.0, 0.0, 0.0])
        self.laser = None
        self.iters = 0
        self.map_info = None
        self.map_initialized = False
        self.lidar_initialized = False
        self.odom_initialized = False
        self.last_pose = None
        self.laser_angles = None
        self.downsampled_angles = None
        self.range_method = None
        self.last_time = None
        self.last_stamp = None
        self.first_sensor_update = True
        self.state_lock = Lock()

        self.local_deltas = np.zeros((self.MAX_PARTICLES, 3))

        self.queries = None
        self.ranges = None
        self.tiled_angles = None
        self.sensor_model_table = None

        self.inferred_pose = None
        self.particle_indices = np.arange(self.MAX_PARTICLES)
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

        self.get_logger().info("Finished initializing, waiting on messages...")

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
        self.map_initialized = True

    def publish_tf(self, pose, stamp=None):
        if stamp is None:
            stamp = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = pose[0]
        t.transform.translation.y = pose[1]
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, pose[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.pub_tf.sendTransform(t)

        if self.PUBLISH_ODOM:
            odom = Odometry()
            odom.header = Utils.make_header('map', stamp)
            odom.pose.pose.position.x = pose[0]
            odom.pose.pose.position.y = pose[1]
            odom.pose.pose.orientation = Utils.angle_to_quaternion(pose[2])
            self.odom_pub.publish(odom)

        return

    def visualize(self):
        if not self.DO_VIZ:
            return

        if self.pose_pub.get_subscription_count() > 0 and isinstance(self.inferred_pose, np.ndarray):
            ps = PoseStamped()
            ps.header = Utils.make_header('map', self.get_clock().now().to_msg())
            ps.pose.position.x = self.inferred_pose[0]
            ps.pose.position.y = self.inferred_pose[1]
            ps.pose.orientation = Utils.angle_to_quaternion(self.inferred_pose[2])
            self.pose_pub.publish(ps)

        if self.particle_pub.get_subscription_count() > 0:
            if self.MAX_PARTICLES > self.MAX_VIZ_PARTICLES:
                proposal_indices = np.random.choice(self.particle_indices, self.MAX_VIZ_PARTICLES, p=self.weights)
                self.publish_particles(self.particles[proposal_indices, :])
            else:
                self.publish_particles(self.particles)

        if self.pub_fake_scan.get_subscription_count() > 0 and isinstance(self.ranges, np.ndarray):
            self.viz_queries[:, 0] = self.inferred_pose[0]
            self.viz_queries[:, 1] = self.inferred_pose[1]
            self.viz_queries[:, 2] = self.downsampled_angles + self.inferred_pose[2]
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
        ls.angle_min = np.min(angles)
        ls.angle_max = np.max(angles)
        ls.angle_increment = np.abs(angles[0] - angles[1])
        ls.range_min = 0
        ls.range_max = np.max(ranges)
        ls.ranges = ranges
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
        position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y])

        orientation = Utils.quaternion_to_angle(msg.pose.pose.orientation)
        pose = np.array([position[0], position[1], orientation])

        if isinstance(self.last_pose, np.ndarray):
            rot = Utils.rotation_matrix(-self.last_pose[2])
            delta = np.array([position - self.last_pose[0:2]]).transpose()
            local_delta = (rot * delta).transpose()

            self.odometry_data = np.array([local_delta[0, 0], local_delta[0, 1], orientation - self.last_pose[2]])
            self.last_pose = pose
            self.last_stamp = msg.header.stamp
            self.odom_initialized = True
        else:
            self.last_pose = pose

        self.update()

    def clicked_pose(self, msg):
        if isinstance(msg, PointStamped):
            self.initialize_global()
        elif isinstance(msg, PoseWithCovarianceStamped):
            self.initialize_particles_pose(msg.pose.pose)

    def initialize_particles_pose(self, pose):
        self.state_lock.acquire()
        self.weights = np.ones(self.MAX_PARTICLES) / float(self.MAX_PARTICLES)
        self.particles[:, 0] = pose.position.x + np.random.normal(loc=0.0, scale=0.5, size=self.MAX_PARTICLES)
        self.particles[:, 1] = pose.position.y + np.random.normal(loc=0.0, scale=0.5, size=self.MAX_PARTICLES)
        self.particles[:, 2] = Utils.quaternion_to_angle(pose.orientation) + np.random.normal(loc=0.0, scale=0.4, size=self.MAX_PARTICLES)
        self.state_lock.release()

    def initialize_global(self):
        self.state_lock.acquire()
        permissible_x, permissible_y = np.where(self.permissible_region == 1)
        indices = np.random.randint(0, len(permissible_x), size=self.MAX_PARTICLES)

        permissible_states = np.zeros((self.MAX_PARTICLES, 3))
        permissible_states[:, 0] = permissible_y[indices]
        permissible_states[:, 1] = permissible_x[indices]
        permissible_states[:, 2] = np.random.random(self.MAX_PARTICLES) * np.pi * 2.0

        Utils.map_to_world(permissible_states, self.map_info)
        self.particles = permissible_states
        self.weights[:] = 1.0 / self.MAX_PARTICLES
        self.state_lock.release()

    def precompute_sensor_model(self):
        z_short = self.Z_SHORT
        z_max = self.Z_MAX
        z_rand = self.Z_RAND
        z_hit = self.Z_HIT
        sigma_hit = self.SIGMA_HIT

        table_width = int(self.MAX_RANGE_PX) + 1
        self.sensor_model_table = np.zeros((table_width, table_width))

        t = time.time()
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


    def motion_model(self, proposal_dist, action):
        cosines = np.cos(proposal_dist[:, 2])
        sines = np.sin(proposal_dist[:, 2])

        self.local_deltas[:, 0] = cosines * action[0] - sines * action[1]
        self.local_deltas[:, 1] = sines * action[0] + cosines * action[1]
        self.local_deltas[:, 2] = action[2]

        proposal_dist[:, :] += self.local_deltas
        proposal_dist[:, 0] += np.random.normal(loc=0.0, scale=self.MOTION_DISPERSION_X, size=self.MAX_PARTICLES)
        proposal_dist[:, 1] += np.random.normal(loc=0.0, scale=self.MOTION_DISPERSION_Y, size=self.MAX_PARTICLES)
        proposal_dist[:, 2] += np.random.normal(loc=0.0, scale=self.MOTION_DISPERSION_THETA, size=self.MAX_PARTICLES)

    def sensor_model(self, proposal_dist, obs, weights):
        num_rays = self.downsampled_angles.shape[0]
        if self.first_sensor_update:
            if self.RANGELIB_VAR <= 1:
                self.queries = np.zeros((num_rays * self.MAX_PARTICLES, 3), dtype=np.float32)
            else:
                self.queries = np.zeros((self.MAX_PARTICLES, 3), dtype=np.float32)

            self.ranges = np.zeros(num_rays * self.MAX_PARTICLES, dtype=np.float32)
            self.tiled_angles = np.tile(self.downsampled_angles, self.MAX_PARTICLES)
            self.first_sensor_update = False

        if self.RANGELIB_VAR == VAR_RADIAL_CDDT_OPTIMIZATIONS:
            if "cddt" in self.WHICH_RM:
                self.queries[:, :] = proposal_dist[:, :]
                self.range_method.calc_range_many_radial_optimized(num_rays, self.downsampled_angles[0], self.downsampled_angles[-1], self.queries, self.ranges)
                self.range_method.eval_sensor_model(obs, self.ranges, self.weights, num_rays, self.MAX_PARTICLES)
                self.weights = np.power(self.weights, self.INV_SQUASH_FACTOR)
        elif self.RANGELIB_VAR == VAR_REPEAT_ANGLES_EVAL_SENSOR_ONE_SHOT:
            self.queries[:, :] = proposal_dist[:, :]
            self.range_method.calc_range_repeat_angles_eval_sensor_model(self.queries, self.downsampled_angles, obs, self.weights)
            np.power(self.weights, self.INV_SQUASH_FACTOR, self.weights)
        elif self.RANGELIB_VAR == VAR_REPEAT_ANGLES_EVAL_SENSOR:
            if self.SHOW_FINE_TIMING:
                t_start = time.time()
            self.queries[:, :] = proposal_dist[:, :]
            if self.SHOW_FINE_TIMING:
                t_init = time.time()
            self.range_method.calc_range_repeat_angles(self.queries, self.downsampled_angles, self.ranges)
            if self.SHOW_FINE_TIMING:
                t_range = time.time()
            self.range_method.eval_sensor_model(obs, self.ranges, self.weights, num_rays, self.MAX_PARTICLES)
            if self.SHOW_FINE_TIMING:
                t_eval = time.time()
            np.power(self.weights, self.INV_SQUASH_FACTOR, self.weights)
            if self.SHOW_FINE_TIMING:
                t_squash = time.time()
                t_total = (t_squash - t_start) / 100.0

            if self.SHOW_FINE_TIMING and self.iters % 10 == 0:
                self.get_logger().info("sensor_model: init: ", np.round((t_init - t_start) / t_total, 2), "range:", np.round((t_range - t_init) / t_total, 2),
                      "eval:", np.round((t_eval - t_range) / t_total, 2), "squash:", np.round((t_squash - t_eval) / t_total, 2))
        elif self.RANGELIB_VAR == VAR_CALC_RANGE_MANY_EVAL_SENSOR:
            self.queries[:, 0] = np.repeat(proposal_dist[:, 0], num_rays)
            self.queries[:, 1] = np.repeat(proposal_dist[:, 1], num_rays)
            self.queries[:, 2] = np.repeat(proposal_dist[:, 2], num_rays)
            self.queries[:, 2] += self.tiled_angles

            self.range_method.calc_range_many(self.queries, self.ranges)

            self.range_method.eval_sensor_model(obs, self.ranges, self.weights, num_rays, self.MAX_PARTICLES)
            np.power(self.weights, self.INV_SQUASH_FACTOR, self.weights)
        elif self.RANGELIB_VAR == VAR_NO_EVAL_SENSOR_MODEL:
            self.queries[:, 0] = np.repeat(proposal_dist[:, 0], num_rays)
            self.queries[:, 1] = np.repeat(proposal_dist[:, 1], num_rays)
            self.queries[:, 2] = np.repeat(proposal_dist[:, 2], num_rays)
            self.queries[:, 2] += self.tiled_angles

            self.range_method.calc_range_many(self.queries, self.ranges)

            obs /= float(self.map_info.resolution)
            ranges = self.ranges / float(self.map_info.resolution)
            obs[obs > self.MAX_RANGE_PX] = self.MAX_RANGE_PX
            ranges[ranges > self.MAX_RANGE_PX] = self.MAX_RANGE_PX

            intobs = np.rint(obs).astype(np.uint16)
            intrng = np.rint(ranges).astype(np.uint16)

            for i in range(self.MAX_PARTICLES):
                weight = np.product(self.sensor_model_table[intobs, intrng[i * num_rays:(i + 1) * num_rays]])
                weight = np.power(weight, self.INV_SQUASH_FACTOR)
                weights[i] = weight
        else:
            self.get_logger().warn("PLEASE SET rangelib_variant PARAM to 0-4")

    def MCL(self, a, o):
        if self.SHOW_FINE_TIMING:
            t = time.time()
        proposal_indices = np.random.choice(self.particle_indices, self.MAX_PARTICLES, p=self.weights)
        proposal_distribution = self.particles[proposal_indices, :]
        if self.SHOW_FINE_TIMING:
            t_propose = time.time()

        self.motion_model(proposal_distribution, a)
        if self.SHOW_FINE_TIMING:
            t_motion = time.time()

        self.sensor_model(proposal_distribution, o, self.weights)
        if self.SHOW_FINE_TIMING:
            t_sensor = time.time()

        self.weights /= np.sum(self.weights)
        if self.SHOW_FINE_TIMING:
            t_norm = time.time()
            t_total = (t_norm - t) / 100.0

        self.particles = proposal_distribution

    def expected_pose(self):
        return np.dot(self.particles.transpose(), self.weights)

    def update(self):
        if self.lidar_initialized and self.odom_initialized and self.map_initialized:
            if not self.state_lock.locked():
                self.state_lock.acquire()
                self.timer.tick()
                self.iters += 1

                t1 = time.time()
                observation = np.copy(self.downsampled_ranges).astype(np.float32)
                action = np.copy(self.odometry_data)
                self.odometry_data = np.zeros(3)

                self.MCL(action, observation)

                self.inferred_pose = self.expected_pose()
                self.state_lock.release()
                t2 = time.time()

                self.publish_tf(self.inferred_pose, self.last_stamp)

                ips = 1.0 / (t2 - t1)
                self.smoothing.append(ips)
                self.visualize()



def main(): 
    rclpy.init()
    try:
        pf = ParticleFiler()
        rclpy.spin(pf)
    except:
        pf.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
