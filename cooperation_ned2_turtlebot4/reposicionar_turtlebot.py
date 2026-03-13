#!/usr/bin/env python3
import math
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool
from action_msgs.msg import GoalStatus

from tf_transformations import quaternion_from_euler, euler_from_quaternion


class NavToPoseClient(Node):
    def __init__(self):
        super().__init__('nav_to_pose_client')

        # ====== AJUSTE AQUI ======
        self.X_FINAL = 0.91
        self.Y_FINAL = 1.20
        self.GOAL_FRAME = "map"
        self.AMCL_TIMEOUT_S = 8.0
        # =========================

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.pub_robot_moveu = self.create_publisher(Bool, '/robot_moveu', 10)

        # QoS compatível com AMCL (frequentemente TRANSIENT_LOCAL)
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

        self.sub_turtle_pose = self.create_subscription(
            PoseStamped,
            '/turtle_pose',
            self.turtle_pose_callback,
            10
        )

        self.current_pose_amcl = None
        self.goal_yaw = None
        self.goal_in_progress = False

        self.start_ns = self.get_clock().now().nanoseconds
        self.guard_timer = self.create_timer(0.2, self.check_amcl_timeout)

        self.get_logger().info("✅ Aguardando /turtle_pose (yaw desejado) e /amcl_pose...")

    # -------------------------------------------------
    def check_amcl_timeout(self):
        if self.current_pose_amcl is not None:
            return
        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self.start_ns) > int(self.AMCL_TIMEOUT_S * 1e9):
            self.get_logger().error(
                f"❌ /amcl_pose não chegou em {self.AMCL_TIMEOUT_S:.1f}s. "
                "AMCL/Localization provavelmente não está ativo."
            )
            self.shutdown(False)

    # -------------------------------------------------
    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose_amcl = msg.pose.pose

    # -------------------------------------------------
    def turtle_pose_callback(self, msg: PoseStamped):
        # só pega o yaw uma vez e navega uma vez
        if self.goal_in_progress or self.goal_yaw is not None:
            return

        q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.goal_yaw = yaw

        self.get_logger().info(f"🟢 yaw desejado recebido = {math.degrees(self.goal_yaw):.2f}°")

        # precisa do AMCL ativo antes de mandar goal (pra Nav2)
        if self.current_pose_amcl is None:
            self.get_logger().warn("⚠️ Ainda sem /amcl_pose. Vou esperar AMCL antes de enviar o goal...")
            return

        self.send_goal(self.X_FINAL, self.Y_FINAL, self.goal_yaw)

    # -------------------------------------------------
    def send_goal(self, x: float, y: float, yaw: float):
        if self.goal_in_progress:
            return

        if not self.client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("❌ Action server 'navigate_to_pose' não disponível. Nav2 está rodando?")
            self.shutdown(False)
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.GOAL_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, float(yaw))
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.goal_in_progress = True
        self.get_logger().info(f"🚗 Indo para (x,y)=({x:.2f},{y:.2f}) com yaw={math.degrees(yaw):.2f}°")

        self.client.send_goal_async(goal).add_done_callback(self.goal_response_cb)

    # -------------------------------------------------
    def goal_response_cb(self, future):
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejeitado.")
            self.shutdown(False)
            return

        self.get_logger().info("✅ Goal aceito. Aguardando resultado...")
        goal_handle.get_result_async().add_done_callback(self.result_cb)

    # -------------------------------------------------
    def result_cb(self, future):
        result = future.result()
        status = result.status
        self.goal_in_progress = False

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f"⚠️ Navegação falhou (status={status}).")
            self.shutdown(False)
            return

        self.get_logger().info("🎯 Cheguei na pose!")

        # imprime yaw final via AMCL (se disponível)
        if self.current_pose_amcl is not None:
            o = self.current_pose_amcl.orientation
            _, _, yaw_final = euler_from_quaternion([o.x, o.y, o.z, o.w])
            self.get_logger().info(f"📍 yaw final (AMCL) = {math.degrees(yaw_final):.2f}°")

        self.pub_robot_moveu.publish(Bool(data=True))
        self.get_logger().info("📤 /robot_moveu = True")

        self.shutdown(True)

    # -------------------------------------------------
    def shutdown(self, ok: bool):
        try:
            self.guard_timer.cancel()
        except Exception:
            pass
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    rclpy.init()
    node = NavToPoseClient()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

'''
    
    
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

        # ✅ QoS do AMCL
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

        self.sub_turtle_pose = self.create_subscription(
            PoseStamped,
            '/turtle_pose',
            self.turtle_pose_callback,
            10
        )

        self.current_amcl_pose = None      # geometry_msgs/Pose
        self.current_turtle_pose = None    # geometry_msgs/Pose

        self.goal_sent = False

        self.awaiting_final_print = False
        self.print_timer = None
        self.print_deadline_ns = 0

        # tenta mandar o goal automaticamente quando tiver as duas fontes
        self.kick_timer = self.create_timer(0.05, self.try_send_goal_from_topics)

    # -------------------------------------------------
    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.current_amcl_pose = msg.pose.pose

    def turtle_pose_callback(self, msg: PoseStamped):
        self.current_turtle_pose = msg.pose

    # -------------------------------------------------
    def try_send_goal_from_topics(self):
        if self.goal_sent:
            return

        if self.current_amcl_pose is None or self.current_turtle_pose is None:
            return

        # x,y do AMCL
        x = float(self.current_amcl_pose.position.x)
        y = float(self.current_amcl_pose.position.y)

        # yaw do /turtle_pose
        ori = self.current_turtle_pose.orientation
        quat = [ori.x, ori.y, ori.z, ori.w]
        _, _, yaw = euler_from_quaternion(quat)

        self.get_logger().info("🎯 Enviando GOAL com:")
        self.get_logger().info(f"   x,y (AMCL) -> x: {x:.3f}, y: {y:.3f}")
        self.get_logger().info(f"   yaw (/turtle_pose) -> {math.degrees(yaw):.2f}°")

        self.send_goal(x, y, yaw)
        self.goal_sent = True

        # não precisa mais ficar tentando
        self.kick_timer.cancel()
        self.kick_timer = None

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

        # imprime x,y do AMCL e yaw do turtle_pose (igual você pediu)
        if (self.current_amcl_pose is not None) and (self.current_turtle_pose is not None):
            pos = self.current_amcl_pose.position

            ori_yaw = self.current_turtle_pose.orientation
            quat = [ori_yaw.x, ori_yaw.y, ori_yaw.z, ori_yaw.w]
            _, _, yaw = euler_from_quaternion(quat)

            self.get_logger().info("📍 Pose FINAL (misturada):")
            self.get_logger().info(f"   Posição (AMCL) -> x: {pos.x:.3f}, y: {pos.y:.3f}, z: {pos.z:.3f}")
            self.get_logger().info(f"   Yaw (/turtle_pose) -> {math.degrees(yaw):.2f}°")

            self.pub_ready.publish(Bool(data=True))
            self.get_logger().info('📤 /ready_to_pick = True enviado para o braço')

            self.awaiting_final_print = False
            self.print_timer.cancel()
            self.print_timer = None
            rclpy.try_shutdown()
            return

        if now_ns > self.print_deadline_ns:
            self.get_logger().warn("⚠️ Timeout esperando /amcl_pose e/ou /turtle_pose para imprimir a pose final.")
            self.pub_ready.publish(Bool(data=True))
            self.get_logger().info('📤 /ready_to_pick = True enviado para o braço')

            self.awaiting_final_print = False
            self.print_timer.cancel()
            self.print_timer = None
            rclpy.try_shutdown()


def main():
    rclpy.init()
    node = NavToPoseClient()
    rclpy.spin(node)


if __name__ == '__main__':
    main()'''


