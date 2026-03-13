#!/usr/bin/env python3
import rclpy
import math

from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool

from tf_transformations import quaternion_from_euler, euler_from_quaternion

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class NavToPoseClient(Node):
    def __init__(self):
        super().__init__('nav_to_pose_client')

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.pub_ready = self.create_publisher(Bool, '/ready_to_pick', 10)

        # ✅ QoS compatível com AMCL (geralmente TRANSIENT_LOCAL)
        amcl_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            amcl_qos
        )

        self.current_pose = None

        self.awaiting_final_print = False
        self.print_timer = None
        self.print_deadline_ns = 0

    # -------------------------------------------------
    def amcl_callback(self, msg):
        self.current_pose = msg.pose.pose
        # (opcional) confirma que está chegando
        # self.get_logger().info("✅ Recebi /amcl_pose")

    # -------------------------------------------------
    def send_goal(self, x, y, yaw):
        self.client.wait_for_server()

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, float(yaw))
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info('🚗 Navegando até a pose...')
        self.client.send_goal_async(goal).add_done_callback(self.goal_response_callback)

    # -------------------------------------------------
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejeitado')
            rclpy.try_shutdown()
            return

        goal_handle.get_result_async().add_done_callback(self.result_callback)

    # -------------------------------------------------
    def result_callback(self, future):
        status = future.result().status

        if status == 4:
            self.get_logger().info('🎯 Cheguei na pose!')

            # aguarda pose chegar via timer (sem bloquear executor)
            self.awaiting_final_print = True
            self.print_deadline_ns = self.get_clock().now().nanoseconds + int(3.0e9)  # 3s

            if self.print_timer is None:
                self.print_timer = self.create_timer(0.05, self.try_print_and_finish)

        else:
            self.get_logger().warn(f'⚠️ Navegação não concluída (status={status})')
            rclpy.try_shutdown()

    # -------------------------------------------------
    def try_print_and_finish(self):
        if not self.awaiting_final_print:
            return

        now_ns = self.get_clock().now().nanoseconds

        if self.current_pose is not None:
            pos = self.current_pose.position
            ori = self.current_pose.orientation

            quat = [ori.x, ori.y, ori.z, ori.w]
            _, _, yaw = euler_from_quaternion(quat)

            self.get_logger().info("📍 Pose FINAL alcançada (AMCL):")
            self.get_logger().info(f"   Posição -> x: {pos.x:.3f}, y: {pos.y:.3f}, z: {pos.z:.3f}")
            self.get_logger().info(
                f"   Orientação (quat) -> x: {ori.x:.3f}, y: {ori.y:.3f}, z: {ori.z:.3f}, w: {ori.w:.3f}"
            )
            self.get_logger().info(f"   Yaw final -> {math.degrees(yaw):.2f}°")

            self.pub_ready.publish(Bool(data=True))
            self.get_logger().info('📤 /ready_to_pick = True enviado para o braço')

            self.awaiting_final_print = False
            self.print_timer.cancel()
            self.print_timer = None
            rclpy.try_shutdown()
            return

        if now_ns > self.print_deadline_ns:
            self.get_logger().warn("⚠️ Timeout esperando /amcl_pose para imprimir a pose final.")
            self.pub_ready.publish(Bool(data=True))
            self.get_logger().info('📤 /ready_to_pick = True enviado para o braço')

            self.awaiting_final_print = False
            self.print_timer.cancel()
            self.print_timer = None
            rclpy.try_shutdown()


def main():
    rclpy.init()
    node = NavToPoseClient()

    x = 0.91
    y = 1.20
    yaw = 0.0

    node.send_goal(x, y, yaw)
    rclpy.spin(node)


if __name__ == '__main__':
    main()

