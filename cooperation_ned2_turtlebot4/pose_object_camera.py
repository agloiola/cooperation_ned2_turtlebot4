#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class AprilTagTFSubscriber(Node):
    def __init__(self):
        super().__init__('april_tag_tf_subscriber')

        # -------------------------------
        # Publisher
        # -------------------------------
        self.pub_pose = self.create_publisher(PoseStamped, '/object_pose', 10)

        # -------------------------------
        # Subscribers
        # -------------------------------
        self.tf_sub = self.create_subscription(
            TFMessage,
            '/apriltag_tf',
            self.tf_callback,
            10
        )

        # ✅ gatilho 1: nav2 chegou
        self.move_sub = self.create_subscription(
            Bool,
            '/robot_moveu',
            self.trigger_callback_robot_moveu,
            10
        )

        # ✅ gatilho 2: pronto pra pegar
        self.ready_sub = self.create_subscription(
            Bool,
            '/ready_to_pick',
            self.trigger_callback_ready_to_pick,
            10
        )

        # -------------------------------
        # Flags
        # -------------------------------
        self.armed = False      # libera a leitura
        self.published = False  # 1 publicação por armamento

        self.get_logger().info("📷 Nó da câmera iniciado.")
        self.get_logger().info("⏳ Aguardando /robot_moveu=True OU /ready_to_pick=True para ler a tag...")

    # ------------------------------------------------
    # Função única: arma a leitura
    # ------------------------------------------------
    def arm_once(self, source: str):
        self.armed = True
        self.published = False
        self.get_logger().info(f"✅ Gatilho recebido ({source}). Armado para publicar 1 pose quando tag_0 aparecer.")

    # ------------------------------------------------
    # Callbacks dos gatilhos
    # ------------------------------------------------
    def trigger_callback_robot_moveu(self, msg: Bool):
        if msg.data:
            self.arm_once("/robot_moveu")

    def trigger_callback_ready_to_pick(self, msg: Bool):
        if msg.data:
            self.arm_once("/ready_to_pick")

    # ------------------------------------------------
    # Callback do TF do AprilTag
    # ------------------------------------------------
    def tf_callback(self, msg: TFMessage):
        # 🔒 Gate principal
        if not self.armed or self.published:
            return

        for transform in msg.transforms:
            if transform.child_frame_id == 'tag_0':

                pose = PoseStamped()
                pose.header = transform.header
                pose.header.frame_id = transform.header.frame_id

                pose.pose.position.x = transform.transform.translation.x
                pose.pose.position.y = transform.transform.translation.y
                pose.pose.position.z = transform.transform.translation.z
                pose.pose.orientation = transform.transform.rotation

                self.pub_pose.publish(pose)

                # ✅ consome: 1 leitura por gatilho
                self.published = True
                self.armed = False

                self.get_logger().info(
                    "🎯 Pose do objeto publicada (/object_pose) após gatilho!\n"
                    f"x={pose.pose.position.x:.3f}, "
                    f"y={pose.pose.position.y:.3f}, "
                    f"z={pose.pose.position.z:.3f}"
                )
                break


def main():
    rclpy.init()
    node = AprilTagTFSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
