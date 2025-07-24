import rclpy, sys, yaml, time
from rclpy.node import Node
from berry_good_bot_interfaces.action import (
    MoveToPose, MoveToNamedPose, ServoTwist)
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.action import ActionClient

class DebugNode(Node):
    def __init__(self):
        super().__init__("berry_debugger")
        self.pose_ac  = ActionClient(self, MoveToPose,  "move_to_pose")
        self.named_ac = ActionClient(self, MoveToNamedPose, "move_to_named_pose")
        self.servo_ac = ActionClient(self, ServoTwist, "servo_twist")
        self.tw_pub   = self.create_publisher(TwistStamped, "eef_twist", 10)

    def send_mode1(self):
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose.position.x = 0.3; ps.pose.position.y = 0.0; ps.pose.position.z = 0.4
        ps.pose.orientation.w = 1.0
        goal = MoveToPose.Goal(target_pose=ps)
        self.pose_ac.wait_for_server()
        self.pose_ac.send_goal_async(goal)

    def send_mode2(self, name="home"):
        goal = MoveToNamedPose.Goal(pose_name=name)
        self.named_ac.wait_for_server()
        self.named_ac.send_goal_async(goal)

    def send_mode3(self):
        goal = ServoTwist.Goal(duration=rclpy.duration.Duration(seconds=3).to_msg())
        self.servo_ac.wait_for_server()
        future = self.servo_ac.send_goal_async(goal)
        # 3초 동안 Twist 발행
        start = time.time()
        while time.time() - start < 3.0:
            tw = TwistStamped()
            tw.twist.linear.x = 0.02
            self.tw_pub.publish(tw)
            time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    dbg = DebugNode()
    mode = sys.argv[1] if len(sys.argv) > 1 else "1"
    if mode == "1":
        dbg.send_mode1()
    elif mode == "2":
        pose = sys.argv[2] if len(sys.argv) > 2 else "home"
        dbg.send_mode2(pose)
    else:
        dbg.send_mode3()
    rclpy.spin(dbg)
    rclpy.shutdown()
