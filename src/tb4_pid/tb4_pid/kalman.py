import math
import rclpy
from rclpy.node import Node
import numpy as np
from sympy import *

from sensor_msgs.msg import NavSatFix, Imu
from custom_interfaces.msg import KalmanPose

def geodetic_to_enu(lat, lon, lat0, lon0):

    lat_r, lon_r = math.radians(lat), math.radians(lon)
    lat0_r, lon0_r = math.radians(lat0), math.radians(lon0)
    x = 6_378_137.0 * (lon_r - lon0_r) * math.cos(lat0_r)
    y = 6_378_137.0 * (lat_r - lat0_r)
    return x, y


def quaternion_to_yaw(x, y, z, w):

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class EKF:

    def __init__(
        self,
        process_noise_pos: float = 0.5,
        process_noise_yaw: float = 0.01,
        gps_noise_xy: float = 1.5,
    ):
        #state x y yaw 
        self.x = np.zeros(3)
        self.P = np.eye(3) * 1.0

        #noise gps için
        self.R = np.diag([gps_noise_xy**2, gps_noise_xy**2])

        #x y ölçüm matrisi
        self.H = np.zeros((2, 3))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0

        self._q_pos = process_noise_pos
        self._q_yaw = process_noise_yaw

    def predict(self, ax_body: float, ay_body: float, yaw_rate: float, dt: float):

        if dt <= 0.0:
            return

        x, y, yaw = self.x

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        #rotasyon body frameden world e
        ax_w = ax_body * cos_yaw - ay_body * sin_yaw
        ay_w = ax_body * sin_yaw + ay_body * cos_yaw

        dx = 0.5 * ax_w * dt**2
        dy = 0.5 * ay_w * dt**2

        #tahmin
        x_new = x + dx
        y_new = y + dy
        yaw_new = yaw + yaw_rate * dt
        yaw_new = math.atan2(math.sin(yaw_new), math.cos(yaw_new))

        self.x = np.array([x_new, y_new, yaw_new])

        #Jakobiyen matrisi F için
        F = np.eye(3)
        F[0, 2] = 0.5 * dt**2 * (-ax_body * sin_yaw - ay_body * cos_yaw)
        F[1, 2] = 0.5 * dt**2 * ( ax_body * cos_yaw - ay_body * sin_yaw)

        #gürültü
        Q = np.diag([self._q_pos * dt, self._q_pos * dt, self._q_yaw * dt,])

        self.P = F @ self.P @ F.T + Q

    def update_gps(self, x_meas: float, y_meas: float):
        z = np.array([x_meas, y_meas])

        #adım 3 için
        y_residue = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y_residue
        I = np.eye(3)
        self.P = (I - K @ self.H) @ self.P
   
        self.x[2] = math.atan2(math.sin(self.x[2]), math.cos(self.x[2]))

class KalmanNode(Node):
    def __init__(self):
        super().__init__("kalman_node")
    
        self.last_t = self.get_clock().now()

        self.ekf = EKF()

        self._ref_lat = None
        self._ref_lon = None

        self._last_imu_time = None

        self.create_subscription(NavSatFix, "/tb4_navsat", self.callback_gps, 10)
        self.create_subscription(Imu, "/tb4_imu", self.callback_imu, 20)

        self.kalman_pose_pub = self.create_publisher(KalmanPose, "/kalman_pose", 10)

        self.get_logger().info('ekf nodeu başladı')

    def callback_imu(self, msg: Imu):
        now = self._stamp_to_sec(msg.header.stamp)

        if self._last_imu_time is not None:
            dt = now - self._last_imu_time
            if dt > 0.0:
                ax = msg.linear_acceleration.x
                ay = msg.linear_acceleration.y
                yaw_rate = msg.angular_velocity.z

                self.ekf.predict(ax, ay, yaw_rate, dt)
                self._publish_pose(msg.header.stamp)

        self._last_imu_time = now

    def callback_gps(self, msg: NavSatFix):
        # ignore invalid fixes
        if msg.status.status < 0:
            return

        lat = msg.latitude
        lon = msg.longitude

        # set the ENU reference on the very first fix
        if self._ref_lat is None:
            self._ref_lat = lat
            self._ref_lon = lon
            self.get_logger().info(
                f'ENU origin set to ({lat:.7f}, {lon:.7f})'
            )

        x_gps, y_gps = geodetic_to_enu(lat, lon, self._ref_lat, self._ref_lon)
        self.ekf.update_gps(x_gps, y_gps)
        self._publish_pose(msg.header.stamp)

    def _publish_pose(self, stamp):
        out = KalmanPose()
        out.x_kalman_est = float(self.ekf.x[0])
        out.y_kalman_est = float(self.ekf.x[1])
        out.yaw_kalman_est = float(self.ekf.x[2])
        self.kalman_pose_pub.publish(out)

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

def main(args=None):
    rclpy.init(args=args)
    node = KalmanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
