import rclpy
from rclpy.node import Node

from custom_interfaces.msg import GoalPose

class GoalPoseNode (Node):
    
    def __init__(self):
        super().__init__('msg_tester')

        self.publisher = self.create_publisher(GoalPose, 'goal_pose', 10)

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

    goalPoseNode = GoalPoseNode()
    rclpy.spin(goalPoseNode)

    goalPoseNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
