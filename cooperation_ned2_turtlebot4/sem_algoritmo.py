#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_configs_utils import MoveItConfigsBuilder
import time
import math
from rclpy.node import Node
from niryo_ned_ros2_interfaces.srv import ToolCommand
from moveit.planning import MoveItPy
from moveit.planning import PlanningSceneMonitor
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import AttachedCollisionObject
from moveit_msgs.msg import Grasp
import sys
import numpy as np
import math

from tf_transformations import euler_from_quaternion

from tf_transformations import quaternion_from_euler, quaternion_multiply
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose 
import rclpy
import time 

from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


from copy import deepcopy
import numpy as np
from shape_msgs.msg import Mesh, MeshTriangle
import trimesh

# Importações das mensagens necessárias
from moveit_msgs.msg import Grasp, GripperTranslation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Vector3

from moveit_msgs.msg import CollisionObject, RobotTrajectory
from shape_msgs.msg import Mesh, MeshTriangle

from geometry_msgs.msg import PoseStamped, Pose, Point, Vector3
from std_msgs.msg import Bool
from std_msgs.msg import Float32



# plan and execute
def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    logger.info("Planning trajectory")

    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    if not plan_result:
        logger.error("Planning failed")
        time.sleep(sleep_time)
        return False  # ⬅️ FALHOU

    logger.info("Executing plan")
    robot_trajectory = plan_result.trajectory
    robot.execute(robot_trajectory, controllers=[])

    time.sleep(sleep_time)
    return True  # ⬅️ DEU CERTO


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place_node')
        self.logger = get_logger('pick_place_node')

        self.logger.info("🚀 Nó Pick & Place inicializado!")
        self.logger.info("🔄 Tentando iniciar MoveItPy...")

        try:
            moveit_config = (
                MoveItConfigsBuilder("niryo_ned2")
                .robot_description(
                    file_path="/home/aline/ros2_drivers_ws/src/ned-ros2-driver/niryo_ned_description/urdf/ned2/niryo_ned2.urdf.xacro"
                )
                .robot_description_semantic(
                    file_path="/home/aline/ros2_drivers_ws/src/ned-ros2-driver/niryo_ned_moveit_configs/niryo_ned2_moveit_config/config/niryo_ned2.srdf"
                )
                .robot_description_kinematics(
                    file_path="/home/aline/ros2_drivers_ws/src/ned-ros2-driver/niryo_ned_moveit_configs/niryo_ned2_moveit_config/config/kinematics.yaml"
                )
                .joint_limits(
                    file_path="/home/aline/ros2_drivers_ws/src/ned-ros2-driver/niryo_ned_moveit_configs/niryo_ned2_moveit_config/config/joint_limits.yaml"
                )
                .trajectory_execution(
                    file_path="/home/aline/ros2_drivers_ws/src/ned-ros2-driver/niryo_ned_moveit_configs/niryo_ned2_moveit_config/config/moveit_controllers.yaml"
                )
                .moveit_cpp(file_path="/home/aline/ros2_drivers_ws/src/ned-ros2-driver/niryo_ned_moveit_configs/niryo_ned2_moveit_config/config/planning_pipelines.yaml")
                .to_moveit_configs()
            )


            self.niryo_ned2 = MoveItPy(
                node_name="pick_place",
                config_dict=moveit_config.to_dict()
            )

            self.logger.info("✅ MoveItPy conectado com sucesso!")
        except Exception as e:
            self.logger.error(f"💥 Falha ao conectar com MoveItPy: {e}")
        
        
        self.open_cli = self.create_client(ToolCommand, '/niryo_robot/tools/open_gripper')
        self.close_cli = self.create_client(ToolCommand, '/niryo_robot/tools/close_gripper')

        self.open_cli.wait_for_service()
        self.close_cli.wait_for_service()
        self.get_logger().info("🦾 Serviços do gripper prontos!")

        self.arm = self.niryo_ned2.get_planning_component("arm")
        self.planning_scene_monitor = self.niryo_ned2.get_planning_scene_monitor() 
        
        # ✅ Define parâmetros do planejamento
        self.arm_plan_request_params = PlanRequestParameters(self.niryo_ned2, "ompl_rrtc")
        self.arm_plan_request_params.planning_time = 0.03
        self.arm_plan_request_params.max_velocity_scaling_factor = 0.3
        self.arm_plan_request_params.max_acceleration_scaling_factor = 0.3

        self.gripper_pose_pub = self.create_publisher(PoseStamped, 'gripper_pose', 10)

        self.logger.info("✅ Parametros passados com sucesso!")

        self.GRIPPER_FRAME = 'mors_2'
        self.GRIPPER_JOINT_NAMES = ["gripper_finger_joint_1", "gripper_finger_joint_2"]

        self.object_pose: PoseStamped = None

        self.object_pose_sub = self.create_subscription(
            PoseStamped,
            '/object_pose',
            self.object_pose_callback,
            10
        )

        self.get_logger().info("Esperando pose do objeto em /object_pose...")

        # === Comunicação com nó do TurtleBot ===
        self.turtle_pose_pub = self.create_publisher(PoseStamped, '/turtle_pose', 10)
        self.robot_moveu_pub = self.create_publisher(Bool, '/robot_moveu_cmd', 10) 
        self.robot_moveu = False

        self.create_subscription(Bool, '/robot_moveu', self.robot_moveu_callback, 10)

        self.pose_atualizada = False
        self.object_pose_cam: PoseStamped = None   # sempre a pose crua da câmera
        self.object_pose: PoseStamped = None       # sempre a pose convertida (base_link) usada no grasp
        self.last_object_stamp = None


        # ===== Pose do TurtleBot via AMCL =====
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

        self.tb_x = None
        self.tb_y = None
        self.tb_yaw = None


    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # extrai yaw (theta) do quaternion
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.tb_x = float(p.x)
        self.tb_y = float(p.y)
        self.tb_yaw = float(yaw)


    def robot_moveu_callback(self, msg: Bool):
        self.robot_moveu = bool(msg.data)

    def object_pose_callback(self, msg: PoseStamped):
        self.object_pose_cam = msg
        self.pose_atualizada = True
        self.last_object_stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        p = msg.pose.position
        self.get_logger().info(
            f"Pose do objeto recebida de /object_pose:\n"
            f"  frame_id: {msg.header.frame_id}\n"
            f"  x={p.x:.4f}, y={p.y:.4f}, z={p.z:.4f}"
        )
    def allow_gripper_object_collision(self, allow=True):
        gripper_links = ["base_gripper_1", "mors_1", "mors_2"]  # ajuste se tiver mais links
        with self.planning_scene_monitor.read_write() as scene:
            acm = scene.allowed_collision_matrix
            for link in gripper_links:
                acm.set_entry("object", link, allow)
            scene.allowed_collision_matrix = acm
            scene.current_state.update()


    def gripper_open(self):
        req = ToolCommand.Request()
        req.id = 11
        req.position = 1900
        req.speed = 100
        req.hold_torque = 100
        req.max_torque = 100
        future = self.open_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("🔓 Garra aberta!")

    def gripper_close(self):
        req = ToolCommand.Request()
        req.id = 11
        req.position = 200
        req.speed = 30
        req.hold_torque = 70
        req.max_torque = 70
        future = self.close_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("🤏 Garra fechada!")

        


    def go_to_target_pose(self):
        
        self.logger.info("INICIANDO...")
        self.create_scene()
        time.sleep(2.0)

        # ============================================================
        #                 VOLTA PARA posição home (sleep)
        # ============================================================
        with self.planning_scene_monitor.read_only() as scene:
            scene.current_state.update()
            ok = self.arm.set_start_state(robot_state=scene.current_state)

        # 1) Ir para a pose de inicial (sleep)
        self.arm.set_goal_state(configuration_name="sleep")
        plan_wait = self.arm.plan(single_plan_parameters=self.arm_plan_request_params)           

        exec_ok = self.niryo_ned2.execute(plan_wait.trajectory, controllers=[])

        if not exec_ok:
            self.logger.error("❌ Execução para sleep")

        time.sleep(1.0)


        self.logger.info("ROBÔ NA POSIÇÃO INICIAL...")
        self.gripper_open()
        self.logger.info("GRIPPER ABERTO")

        # ============================================================
        #            LOOP DE TENTATIVAS DE PICK  
        # ============================================================

        max_pick_attempts = 10
        pick_success = False
        best_pose = None
        attempt = 0

        yaw = 0.0
        valor = 22.5
        
        while not pick_success and attempt < max_pick_attempts:
            attempt += 1
            self.logger.info(f"🔁 Tentativa de PICK: {attempt}")

            initial_pose = PoseStamped()
            initial_pose.header.frame_id = self.object_pose.header.frame_id
            initial_pose.pose = deepcopy(self.object_pose.pose)
            
            # initial_pose.pose.position.x -= 0.0015
            # initial_pose.pose.position.z += 0.035
            
            initial_pose.pose.position.z += 0.035

            initial_pose.pose.orientation = deepcopy(self.object_pose.pose.orientation)

            grasp_candidates = self.make_grasps(initial_pose, ['object'])

            for grasp in grasp_candidates:
                self.gripper_pose_pub.publish(grasp.grasp_pose)
                time.sleep(0.0001)

            self.allow_gripper_object_collision(False)


            for i, candidate in enumerate(grasp_candidates):
                self.logger.info(f"🧪 Testando grasp candidato {i+1}/{len(grasp_candidates)}")

                with self.planning_scene_monitor.read_only() as scene:
                    scene.current_state.update()
                    ok = self.arm.set_start_state(robot_state=scene.current_state)

                if not ok:
                    self.logger.warning("Falha ao setar start_state via RobotState")


                # 3) Define goal e planeja
                self.arm.set_goal_state(
                    pose_stamped_msg=candidate.grasp_pose,
                    pose_link="gripper_tcp"
                )


                plan_result = self.arm.plan(
                    single_plan_parameters=self.arm_plan_request_params
                )

                if not plan_result:
                    self.logger.warning("❌ Falha no planejamento")
                    continue
                

                if plan_result:

                    pre_grasp_pose = deepcopy(candidate.grasp_pose)

                    pre_grasp_pose.pose.position.x -= 0.02  # 3 cm para trás

                    self.logger.info("▶ Indo para PRE-GRASP (-3 cm em X)")

                    self.allow_gripper_object_collision(False)

                    self.arm.set_goal_state(
                        pose_stamped_msg=pre_grasp_pose,
                        pose_link="gripper_tcp"
                    )

                    plan_pre = self.arm.plan(
                        single_plan_parameters=self.arm_plan_request_params
                    )

                    if not plan_pre:
                        self.logger.warning("❌ Falha no planejamento do PRE-GRASP")
                        continue

                    exec_ok = self.niryo_ned2.execute(plan_pre.trajectory, controllers=[])
                    time.sleep(1.0)

                    if not exec_ok:
                        self.logger.warning("❌ Execução do PRE-GRASP falhou")
                        continue


                    # ⏸ espera ANTES do grasp
                    self.logger.info("⏸Aguardando antes do GRASP FINAL...")
                    time.sleep(1.0)

                    # 2) GRASP FINAL: agora vai para a pose "melhor" (candidate.grasp_pose)
                    self.logger.info("▶ Executando GRASP FINAL (pose planejada)...")
                

                    with self.planning_scene_monitor.read_only() as scene:
                        scene.current_state.update()
                        ok = self.arm.set_start_state(robot_state=scene.current_state)
                    if not ok:
                        self.logger.warning("Falha ao setar start_state (grasp final)")

                    self.arm.set_goal_state(pose_stamped_msg=candidate.grasp_pose, pose_link="gripper_tcp")
                    plan_final = self.arm.plan(single_plan_parameters=self.arm_plan_request_params)
                    if not plan_final:
                        self.logger.warning("❌ Falha no planejamento do GRASP FINAL")
                        continue

                    exec_ok = self.niryo_ned2.execute(plan_final.trajectory, controllers=[])
                    time.sleep(2.0)

                    if not exec_ok:
                        self.logger.error("❌ Execução do GRASP FINAL falhou")
                        continue

                    # ✅ Só entra aqui se executou as duas
                    self.logger.info("✅ Execução concluída com sucesso.")
                    best_pose = candidate
                    pick_success = True

                    # logs
                    self.logger.info(
                        f"Posição do objeto: x={self.object_pose.pose.position.x}, "
                        f"y={self.object_pose.pose.position.y}, "
                        f"z={self.object_pose.pose.position.z}"
                    )
                    grasp_pose = candidate.grasp_pose.pose
                    self.logger.info(
                        f"Posição do grasp selecionado: x={grasp_pose.position.x:.3f}, "
                        f"y={grasp_pose.position.y:.3f}, "
                        f"z={grasp_pose.position.z:.3f}"
                    )

                    break


            if not pick_success:
                self.logger.warning("⚠️ Nenhum grasp executável nesta tentativa de PICK.")
                rclpy.shutdown()
                raise SystemExit(1)

            
        if not pick_success:
            self.logger.error(f"💥 PICK falhou após {max_pick_attempts} tentativas. Abortando.")
            return

        self.logger.info("✅ ✅ ✅ ✅ ✅ ✅ ✅ ✅ ")
        self.logger.info("PICK CONCLUÍDO – ROBÔ NO MELHOR GRASP")

        # ============================================================
        #                 FECHA GARRA E ANEXA OBJETO
        # ============================================================
       # 1) Permite contato gripper <-> object (evita CheckStartStateCollision)
        self.allow_gripper_object_collision(True)
    
        with self.planning_scene_monitor.read_only() as scene:
                scene.current_state.update()
                ok = self.arm.set_start_state(robot_state=scene.current_state)
        self.gripper_close()
        self.sleep_time = 2.0
        self.logger.info("FECHOU O GRIPPER ")


        attached_object = AttachedCollisionObject()
        attached_object.link_name = "mors_2"
        attached_object.object.id = "object"
        attached_object.object.operation = CollisionObject.ADD
        attached_object.touch_links = ["base_gripper_1", "mors_1", "mors_2", "tool_link"]


        with self.planning_scene_monitor.read_write() as scene:
            scene.process_attached_collision_object(attached_object)

            # Permite colisão do objeto 'object' com os dedos da garra
            acm = scene.allowed_collision_matrix
            for link_name in ["mors_1", "mors_2", "tool_link"]:
                acm.set_entry("object", link_name, True)
            scene.allowed_collision_matrix = acm

            scene.current_state.update()

        self.logger.info("📦 Objeto anexado ao gripper e colisão com mors_1/mors_2 liberada.")
        time.sleep(0.3)

        self.arm_plan_request_params.planning_time = 1.0

        


        # # 🔄 Garante que MoveIt está usando o estado atual do robô real
        # with self.planning_scene_monitor.read_only() as scene:
        #     scene.current_state.update()
        #     ok = self.arm.set_start_state(robot_state=scene.current_state)

        # if not ok:
        #     self.logger.warning("⚠️ Não consegui setar start_state via scene.current_state (LIFT)")

        # # ============================================================
        # #                LIFT: afasta o objeto da mesa
        # # ============================================================
        # lift_pose = PoseStamped()
        # lift_pose.header.frame_id = "base_link"
        # lift_pose.pose = deepcopy(best_pose.grasp_pose.pose)
        # lift_pose.pose.position.z += 0.10  # sobe 10 cm

        # self.arm.set_goal_state(pose_stamped_msg=lift_pose, pose_link="gripper_tcp")

        # self.logger.info("🔼 Fazendo movimento de lift (afastando object da mesa)...")

        # plan_result = self.arm.plan(single_plan_parameters=self.arm_plan_request_params)
        # if not plan_result:
        #     self.logger.error("❌ Falha no planejamento do LIFT")
        #     return

        # exec_ok = self.niryo_ned2.execute(plan_result.trajectory, controllers=[])
        # time.sleep(2.0)

        # if not exec_ok:
        #     self.logger.error("❌ Execução do LIFT falhou (controller abortou)")
        #     return
        

        # with self.planning_scene_monitor.read_only() as scene:
        #     scene.current_state.update()
        #     self.arm.set_start_state(robot_state=scene.current_state)


        # ============================================================
        #                 BASE DO PLACE
        # ============================================================
        x, y, z, roll, pitch, yaw = 0.30,0.30, 0.05, 0.0, 0.0, 1.57

        self.logger.info("📍 POSIÇÃO FINAL (pose base de place)...")

        place_base = PoseStamped()
        place_base.header.frame_id = "base_link"
        place_base.pose.position.x = x
        place_base.pose.position.y = y
        place_base.pose.position.z = z

        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        place_base.pose.orientation.x = qx
        place_base.pose.orientation.y = qy
        place_base.pose.orientation.z = qz
        place_base.pose.orientation.w = qw

        # Gera candidatos de place ao redor da pose base
        place_candidates = self.make_places(place_base)
        self.logger.info(f"📦 Geradas {len(place_candidates)} poses candidatas para PLACE.")

        # ============================================================
        #            LOOP DE TENTATIVAS DE PLACE
        # ============================================================
        max_place_attempts = 10
        place_attempt = 0
        place_success = False

        self.arm_plan_request_params.planning_time = 3.0

        for place_pose in place_candidates:
            if place_attempt >= max_place_attempts:
                break

            place_attempt += 1
            self.logger.info(f"📦 Tentativa de PLACE: {place_attempt}/{max_place_attempts}")

            with self.planning_scene_monitor.read_only() as scene:
                scene.current_state.update()
                ok = self.arm.set_start_state(robot_state=scene.current_state)
            self.arm.set_goal_state(pose_stamped_msg=place_pose, pose_link="gripper_tcp")

            plan_result = self.arm.plan(single_plan_parameters=self.arm_plan_request_params)
            if not plan_result:
                self.logger.warning("❌ Falha no planejamento do PLACE, tentando próximo candidato...")
                continue

            exec_ok = self.niryo_ned2.execute(plan_result.trajectory, controllers=[])
            time.sleep(2.0)

            if not exec_ok:
                self.logger.warning("❌ Execução do PLACE falhou (controller abortou), tentando próximo...")
                continue

            self.logger.info("✅ PLACE DEU CERTO!")
            best_place = place_pose
            
            place_success = True
            break

        if not place_success:
            self.logger.warning(f"⚠️ PLACE falhou após {place_attempt} tentativas.")
            return

        self.logger.info("✅ ✅ ✅ ✅ ✅ ✅ ✅ ✅ ")
        self.logger.info("MOVEU PARA POSIÇÃO DE PLACE")

        # ============================================================
        #            ABRE GARRA E DESANEXA + REMOVE OBJETO DA CENA
        # ============================================================
        self.gripper_open()
        time.sleep(1.0)
        self.logger.info("🔓 ABRIU O GRIPPER")

        detach = AttachedCollisionObject()
        detach.link_name = "mors_2"          # mesmo link usado no attach
        detach.object.id = "object"
        detach.object.operation = CollisionObject.REMOVE

        with self.planning_scene_monitor.read_write() as scene:
            scene.process_attached_collision_object(detach)

            # (opcional, mas recomendado) desfaz permissões de colisão que você liberou
            acm = scene.allowed_collision_matrix
            for link_name in ["mors_1", "mors_2", "tool_link", "base_gripper_1"]:
                acm.set_entry("object", link_name, False)
            scene.allowed_collision_matrix = acm

            scene.current_state.update()

        self.logger.info("🖐️ Objeto desanexado da garra (não removi do mundo).")
        time.sleep(2.0)



        # ============================================================
        #                 VOLTA PARA SLEEP
        # ============================================================
        with self.planning_scene_monitor.read_only() as scene:
                scene.current_state.update()
                ok = self.arm.set_start_state(robot_state=scene.current_state)
        self.arm.set_goal_state(configuration_name="pose_cam")

        plan_result = self.arm.plan(single_plan_parameters=self.arm_plan_request_params)
        if not plan_result:
            self.logger.error("❌ Falha ao planejar para 'pose_cam'")
            return

        exec_ok = self.niryo_ned2.execute(plan_result.trajectory, controllers=[])
        time.sleep(2.0)

        if not exec_ok:
            self.logger.error("❌ Falha ao executar trajetória para 'pose_cam'")
            return
        

        with self.planning_scene_monitor.read_write() as scene:

            # # 2) Remove do mundo (mesmo se já não existir, não tem problema)
            rm_obj = CollisionObject()
            rm_obj.header.frame_id = "base_link"
            rm_obj.id = "object"
            rm_obj.operation = CollisionObject.REMOVE
            scene.apply_collision_object(rm_obj)

            scene.current_state.update()


        self.logger.info("🧹 Finalizado: robô em na posição inicial e cena limpa.")

    
    
    def make_gripper_posture(self, joint_positions, effort=0.0):
        """
        Cria uma trajetória para abrir ou fechar a garra.
        """
        t = JointTrajectory()
        t.joint_names = self.GRIPPER_JOINT_NAMES

        tp = JointTrajectoryPoint()
        tp.positions = joint_positions
        
    
        tp.time_from_start = Duration(seconds=1.0).to_msg()

        t.points.append(tp)

        return t
    # Gerar a translação do gripper na direção dada pelo vetor
    def make_gripper_translation(self, min_dist, desired, vector, frame_id=None):
        """
        Define a aproximação ou retirada da garra.
        """
        gt = GripperTranslation()
        
        # Define o vetor de direção (x, y, z)
        gt.direction.vector.x = float(vector[0])
        gt.direction.vector.y = float(vector[1])
        gt.direction.vector.z = float(vector[2])

        # Define o frame de referência (se não passar, usa o padrão da classe)
        if frame_id is None:
            gt.direction.header.frame_id = self.GRIPPER_FRAME
        else:
            gt.direction.header.frame_id = frame_id

        gt.min_distance = float(min_dist)
        gt.desired_distance = float(desired)

        return gt

    def make_grasps(self, initial_pose_stamped, allowed_touch_objects):
        """
        Gera uma lista de poses (PoseStamped) em torno de uma pose inicial,
        usando a mesma lógica de varrer x, y, z, pitch e yaw que você usou no ROS1.
        """
    
        g = Grasp()
        
        # Usar diretamente g.pre_grasp_posture e g.grasp_posture
        g.pre_grasp_posture = self.make_gripper_posture([0.1, 0.1])  # Usar a função para criar a postura
        g.grasp_posture = self.make_gripper_posture([-0.10, -0.10])  # Definir a postura de grip

        # Aproximação (Pre-Grasp Approach)
        g.pre_grasp_approach = self.make_gripper_translation(
            0.02, 0.03, [1.0, 0.0, 0.0], frame_id="gripper_tcp"
        )

        # Retirada (Post-Grasp Retreat)
        g.post_grasp_retreat = self.make_gripper_translation(
            0.015, 0.02, [0.0, 0.0, 1.0], frame_id="gripper_tcp"
        )

  
        

        yaw_vals = [0.0, 0.125, -0.125, 0.25, 0.50, 0.75, -0.25, -0.50,-0.75]
        #yaw_vals = [0.0,  0.125, -0.125]
        

        #x_vals = [0.0, -0.01, -0.02, -0.03]
  
        z_vals = [-0.01, -0.02, 0.00, 0.01, 0.02, 0.01, 0.02, 0.03, 0.04]

        x_vals = [0.0, -0.005, -0.01, -0.015 -0.018, -0.02]

        #y_vals = [0.0, 0.005, -0.005, 0.01, -0.01]

        

        grasps = []

        for yaw in yaw_vals:
            for dx in x_vals:
                    #for dy in y_vals:
                
                        for dz in z_vals:
                            g.grasp_pose = deepcopy(initial_pose_stamped)
                                    
                            g.grasp_pose.pose.position.x += dx
                            #g.grasp_pose.pose.position.y += dy
                        
                            g.grasp_pose.pose.position.z += dz
            
                            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
                            g.grasp_pose.pose.orientation.x = qx
                            g.grasp_pose.pose.orientation.y = qy
                            g.grasp_pose.pose.orientation.z = qz
                            g.grasp_pose.pose.orientation.w = qw

                            

                            g.id = str(len(grasps))

                            g.grasp_quality = 1.0 - abs(yaw)
                            
                            g.allowed_touch_objects = allowed_touch_objects

                            grasps.append(deepcopy(g)) 

        self.logger.info(f"🔎 Geradas {len(grasps)} poses candidatas de grasp.")
        return grasps

    

    def make_places(self, init_pose_stamped):
        """
        Gera várias poses candidatas para 'place' (colocação do objeto),
        variando posição e orientação ao redor da pose inicial.
        """

        x_vals = [ 0.01, -0.005, -0.01]
        y_vals = [0.01, -0.005, -0.01]
        z_vals = [0.01, 0.02, 0.03, 0.04]

       
        pitch_vals = [0.0]
        yaw_vals = [0.0, 0.125, -0.125, 0.25, 0.50, 0.75, -0.25, -0.50,-0.75]

        place_poses = []

        for yaw in yaw_vals:
            for pitch in pitch_vals:
                for dx in x_vals:
                    for dy in y_vals:
                        for dz in z_vals:

                            place_pose = deepcopy(init_pose_stamped)

                            place_pose.pose.position.x += dx
                            place_pose.pose.position.y += dy
                            place_pose.pose.position.z += dz

                            qx, qy, qz, qw = quaternion_from_euler(0.0, pitch, yaw)
                            place_pose.pose.orientation.x = qx
                            place_pose.pose.orientation.y = qy
                            place_pose.pose.orientation.z = qz
                            place_pose.pose.orientation.w = qw

                            place_poses.append(place_pose)

        self.logger.info(f"📦 Geradas {len(place_poses)} poses candidatas para place.")

        return place_poses
    
    
      

    def add_mesh_to_scene(self, object_name, object_pose_st, mesh_path, scale=(1.0, 1.0, 1.0)):
        """
        Carrega um STL com trimesh e converte para shape_msgs/Mesh,
        depois adiciona como CollisionObject na planning scene (ROS2/MoveIt2).
        """

        mesh = trimesh.load(mesh_path)

       
        s_x, s_y, s_z = scale
       
        s_uniform = (s_x + s_y + s_z) / 3.0
        mesh.apply_scale(s_uniform)

      
        ros_mesh = Mesh()

        for v in mesh.vertices:
            p = Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))
            ros_mesh.vertices.append(p)

        for f in mesh.faces:
            tri = MeshTriangle()
            tri.vertex_indices = [int(f[0]), int(f[1]), int(f[2])]
            ros_mesh.triangles.append(tri)

        mesh_co = CollisionObject()
        mesh_co.id = object_name
        mesh_co.header.frame_id = object_pose_st.header.frame_id

        mesh_co.meshes.append(ros_mesh)
        mesh_co.mesh_poses.append(object_pose_st.pose)

        mesh_co.operation = CollisionObject.ADD


        with self.planning_scene_monitor.read_write() as scene:
            scene.apply_collision_object(mesh_co)
            scene.current_state.update()

        self.logger.info(f"📦 Mesh '{object_name}' adicionado com sucesso!")
        self.logger.info(
            f"Pose: x={object_pose_st.pose.position.x:.3f}, "
            f"y={object_pose_st.pose.position.y:.3f}, "
            f"z={object_pose_st.pose.position.z:.3f}"
        )
    
    def wait_new_object_pose(self, prev_stamp):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last_object_stamp is not None and self.last_object_stamp != prev_stamp:
                return True

    




    def create_scene(self):
        self.logger.info("CRIANDO CENA...")

        if self.object_pose_cam is None:
            self.logger.error("❌ self.object_pose ainda é None! Nenhuma pose recebida de /object_pose.")
            return None

        # Adiciona objeto para pick
        object_pose_st = PoseStamped()
        object_pose_st.header.frame_id = "base_link"
        

        object_pose_st.pose.position.x= -self.object_pose_cam.pose.position.y + 0.374 +0.015 #0.3681
        object_pose_st.pose.position.y = -self.object_pose_cam.pose.position.x + 0.0152 +0.003#+ 0.025 #+  0.03
        object_pose_st.pose.position.z = -0.017
        
        
        roll, pitch, yaw = euler_from_quaternion([
            self.object_pose_cam.pose.orientation.x,
            self.object_pose_cam.pose.orientation.y,
            self.object_pose_cam.pose.orientation.z,
            self.object_pose_cam.pose.orientation.w
        ])

       
        self.logger.info(f"YAW (em radianos) ={yaw:.3f}")

        
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)

       
        object_pose_st.pose.orientation.x = qx
        object_pose_st.pose.orientation.y = qy
        object_pose_st.pose.orientation.z = -qz
        object_pose_st.pose.orientation.w = qw

       
        object_name = "object"
        mesh_path = "/home/aline/ros2_drivers_ws/src/pick_place_ned2/teste_certo.stl"
        scale = (0.001, 0.001, 0.001)

        self.add_mesh_to_scene(object_name, object_pose_st, mesh_path, scale)

        self.logger.info(f"Objeto '{object_name}' adicionado com sucesso!")
        self.logger.info(f"Pose: x={object_pose_st.pose.position.x:.3f}, "
                        f"y={object_pose_st.pose.position.y:.3f}, "
                        f"z={object_pose_st.pose.position.z:.3f}, "
                        f"orientação z={object_pose_st.pose.orientation.z:.3f}")

        # guarda para o planner de grasp
        self.object_pose = deepcopy(object_pose_st)


        # Caixa para place

        box = CollisionObject()
        box.id = "caixa_obstaculo"
        box.header.frame_id = "base_link"

       
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.25, 0.25, 0.37]  

      
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = 0.2656
        pose.pose.position.y = 0.4038
        pose.pose.position.z = -0.37/2  
        pose.pose.orientation.w = 1.0

        box.primitives = [primitive]
        box.primitive_poses = [pose.pose]
        box.operation = CollisionObject.ADD

        with self.planning_scene_monitor.read_write() as scene:
            scene.apply_collision_object(box)
            scene.current_state.update()

        self.logger.info("📦 Caixa 'caixa_obstaculo' adicionada na cena!")


    
      
        #======== TurtleBot real (AMCL) como cilindro =========
        if self.tb_x is None:
            self.logger.warn("⚠️ Ainda não recebi /amcl_pose. Vou usar valores padrão (0,0,0).")
            #tb_x, tb_y, tb_yaw = 0.0, 0.0, 0.0
        else:
            tb_x, tb_y, tb_yaw = self.tb_x, self.tb_y, self.tb_yaw

        turtlebot4 = SolidPrimitive()
        turtlebot4.type = SolidPrimitive.CYLINDER
        turtlebot4.dimensions = [0.351, 0.15]  # altura, raio

        turtlebot4_pose = Pose()
        turtlebot4_pose.position.x = tb_x + 0.09
        turtlebot4_pose.position.y = tb_y
        turtlebot4_pose.position.z = 0.351/2  # AMCL é no chão; se quiser visual, pode pôr -0.20
        # tb_yaw = 0.0
        # turtlebot4_pose = Pose()
        # turtlebot4_pose.position.x = object_pose_st.pose.position.x
        # turtlebot4_pose.position.y = object_pose_st.pose.position.y
        # turtlebot4_pose.position.z = -0.195 

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, tb_yaw)
        turtlebot4_pose.orientation.x = qx
        turtlebot4_pose.orientation.y = -qy
        turtlebot4_pose.orientation.z = qz
        turtlebot4_pose.orientation.w = qw

        turtlebot4_obj = CollisionObject()
        turtlebot4_obj.id = "turtlebot4"
        turtlebot4_obj.header.frame_id = "base_link"   
        turtlebot4_obj.primitives = [turtlebot4]
        turtlebot4_obj.primitive_poses = [turtlebot4_pose]
        turtlebot4_obj.operation = CollisionObject.ADD

        with self.planning_scene_monitor.read_write() as scene:
            scene.apply_collision_object(turtlebot4_obj)
            scene.current_state.update()

        # (opcional) guardar pra usar depois
        self.turtlebot_pose = PoseStamped()
        self.turtlebot_pose.header.frame_id = "base_link"
        self.turtlebot_pose.pose = deepcopy(turtlebot4_pose)

        self.logger.info(
            f"🤖 TurtleBot na cena (AMCL): x={tb_x:.3f}, y={tb_y:.3f}, yaw={math.degrees(tb_yaw):.1f}°"
        )

                #======== TurtleBot real (AMCL) como stl =========
        # if self.tb_x is None:
        #     self.logger.warn("⚠️ Ainda não recebi /amcl_pose. Vou usar valores padrão (0,0,0).")
        #     #tb_x, tb_y, tb_yaw = 0.0, 0.0, 0.0
        # else:
        #     tb_x, tb_y, tb_yaw = self.tb_x, self.tb_y, self.tb_yaw

        
 

        # turtlebot4 = PoseStamped()
        # turtlebot4.header.frame_id = "base_link"
        # turtlebot4.pose.position.x = tb_x
        # turtlebot4.pose.position.y = tb_y 
        # turtlebot4.pose.position.z = -0.37  # AMCL é no chão; se quiser visual, pode pôr -0.20

        # qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, tb_yaw + 1.57)
        # turtlebot4.pose.orientation.x = qx
        # turtlebot4.pose.orientation.y = -qy
        # turtlebot4.pose.orientation.z = qz 
        # turtlebot4.pose.orientation.w = qw
        
        # turtlebot_name = "turtlebot4"
        # mesh_path   = "/home/aline/ros2_drivers_ws/src/pick_place_ned2/Totobot.stl"
        # scale       = (0.001, 0.001, 0.001)

        # self.add_mesh_to_scene(turtlebot_name , turtlebot4, mesh_path, scale)

        # # (opcional) guardar pra usar depois
        # self.turtlebot_pose = PoseStamped()
        # self.turtlebot_pose.header.frame_id = "map"
        # self.turtlebot_pose.pose = deepcopy(turtlebot4.pose)

         
        expected_scene_objects = ["object", "turtlebot4", "caixa_obstaculo"]

        
    
        scene_manager = self.planning_scene_monitor

        try:
          
            self.wait_for_scene_update(
                scene_manager,
                expected_scene_objects=expected_scene_objects,
                timeout_sec=5.0, 
            )

            self.logger.info("🎨 Objeto adicionado com sucesso à cena!")
            return expected_scene_objects

        except Exception as error:
            self.logger.error("FALHAAAA")
            self.logger.error(f'{error.args[0]} Objects not created.')
            
            
            with scene_manager.read_write() as scene:
                scene.remove_all_collision_objects() 
                scene.current_state.update()

            self.logger.error("🚨 Falha na criação da cena. Processo interrompido.")
            rclpy.shutdown()
            raise SystemExit(1)
           
    def wait_for_scene_update(self, scene_manager, expected_scene_objects, expected_attached_objects=None, timeout_sec=10.0):
        start_time = time.time()
        while (time.time() - start_time) < timeout_sec:
            with scene_manager.read_only() as scene:
                ps_msg = scene.planning_scene_message
                current_scene_objects = [obj.id for obj in ps_msg.world.collision_objects]
                current_attached_objects = [
                    ac.object.id for ac in ps_msg.robot_state.attached_collision_objects
                ]
            if set(expected_scene_objects).issubset(current_scene_objects) and \
            set(expected_attached_objects or []).issubset(current_attached_objects):
                return 
            time.sleep(0.1)
        raise Exception(f"Scene was not updated before timeout of {timeout_sec} seconds.")

        
    
        

    
def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    rclpy.spin_once(node, timeout_sec=2.0)

    
    # Espera até receber a pose do objeto
    node.get_logger().info("⏳ Aguardando pose do objeto em /object_pose...")
    while rclpy.ok() and node.object_pose_cam is None:
        rclpy.spin_once(node, timeout_sec=0.1)


    node.get_logger().info("✅ Pose do objeto recebida, iniciando go_to_target_pose()")

    # Começar o pick & place
    node.go_to_target_pose()

   
    if hasattr(node, "niryo_ned2"):
        node.niryo_ned2.shutdown()
        node.logger.info("🧩 MoveItPy encerrado com segurança!")

    node.destroy_node()
    time.sleep(1.0)
    rclpy.shutdown()



if __name__ == "__main__":
    main()