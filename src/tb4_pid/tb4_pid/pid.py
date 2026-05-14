import math
import rclpy
from rclpy.node import Node

from custom_interfaces.msg import GoalPose
from geometry_msgs.msg import TwistStamped

def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

class PID:
    def __init__(self, kp, ki, kd, i_limit=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit = abs(i_limit)
        self.i = 0.0
        self.prev_e = None

    def reset(self):
        self.i = 0.0
        self.prev_e = None

    def step(self, e, dt):
        if dt <= 0.0:
            return self.kp * e
        self.i += e * dt
        self.i = max(-self.i_limit, min(self.i_limit, self.i))

        if self.prev_e is None:
            de = 0.0
        else:
            de = (e - self.prev_e) / dt
        self.prev_e = e

        return self.kp * e + self.ki * self.i + self.kd * de

class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")

        # ---- Parameters you can change quickly ----
        self.target_x = 0.0  # meters (local frame)
        self.target_y = 0.0   # meters (local frame)
        
        self.target_yaw = 0.0   # radians (local frame)


        self.pos_tol = 0.25   # meters
        self.yaw_tol = 0.15   # rad
    
        self.max_lin = 0.30   # m/s
        self.max_ang = 1.20   # rad/s

        # distance PID and heading PID
        self.pid_dist = PID(kp=0.6, ki=0.0, kd=0.1, i_limit=1.0)
        self.pid_yaw  = PID(kp=2.2, ki=0.0, kd=0.2, i_limit=1.0)

        # ---- State ----
        self.have_ref = False
        self.have_target = False
        self.lat0 = None
        self.lon0 = None

        self.x = None
        self.y = None
        self.yaw = None

        self.last_t = self.get_clock().now()
    
        self.create_subscription(GoalPose, "/goal_pose", self.goalPoseCallback,10)
        
        self.get_logger().info("Waiting for goal_pose to be published")


        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        self.create_timer(0.05, self.control_loop)

        #self.get_logger().info(f"Target set to local (x,y)=({self.target_x:.2f},{self.target_y:.2f}) meters.")

    def goalPoseCallback(self, msg: GoalPose):
        
        if not self.have_target:
            self.target_x = msg.x
            self.target_y = msg.y

            self.target_yaw = msg.theta

            self.have_target= True
        
            self.get_logger().info(f'goal_pose set to: x={self.target_x}, y={self.target_y}, theta={self.target_yaw}') 
        else:
            return    

    def publish_stop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        # need both position and yaw
    #   if self.x is None or self.y is None or self.yaw is None or not self.have_ref:
    #        return
        if not self.have_ref:
            self.x=0
            self.y=0
            self.yaw=0
            self.have_ref= True

        self.get_logger().info(f"x={self.x}, y={self.y}, yaw={self.yaw}")

        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now

        dx = self.target_x - self.x
        dy = self.target_y - self.y
        dist = math.hypot(dx, dy)

        desired_yaw = math.atan2(dy, dx)
        yaw_err = wrap_pi(desired_yaw - self.yaw)

        # Stop condition
        if dist < self.pos_tol:
            self.publish_stop()
            return

        # PID outputs
        v = self.pid_dist.step(dist, dt)
        w = self.pid_yaw.step(yaw_err, dt)

        # Gate forward speed if facing away
        # (helps “turn then go” behavior)
        heading_scale = max(0.0, math.cos(yaw_err))
        v *= heading_scale

        # Clamp
        v = max(-self.max_lin, min(self.max_lin, v))
        w = max(-self.max_ang, min(self.max_ang, w))

        cmd = TwistStamped()
        cmd.header.stamp = now.to_msg()
        cmd.twist.linear.x = v
        cmd.twist.angular.z = w
        self.cmd_pub.publish(cmd)
        self.get_logger().info('publishing : "%s"' % cmd)


class Test (Node):
    
    def __init__(self):
        super().__init__('msg_tester')

        self.publisher = self.create_publisher(GoalPose, 'topic', 10)

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = GoalPose()
        msg.x = 5.0
        msg.y = 5.0
        msg.theta = 2.0

        self.publisher.publish(msg=msg)
        self.get_logger().info('Publishing "%s"' % msg)
        

def main(args=None):
    rclpy.init(args=args)

    #testPublisher = Test()
    #rclpy.spin(testPublisher)
    #testPublisher.destroy_node()

    controlNode = ControlNode()
    rclpy.spin(controlNode)
    controlNode.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
