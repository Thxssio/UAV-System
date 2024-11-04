#!/usr/bin/env python3
import threading
import time

import actionlib
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from interface_package.srv import MoveCommand, MoveCommandResponse
from interface_package.msg import (
    ArmDesarmAction,
    ArmDesarmFeedback,
    ArmDesarmResult,
    LandAction,
    LandFeedback,
    LandResult,
    SetModeAction,
    SetModeFeedback,
    SetModeResult,
    TakeoffAction,
    TakeoffFeedback,
    TakeoffResult,
)


class InterfaceManager:
    def __init__(self):
        rospy.init_node("interface_manager")
        self.move_command_active = False

        # Inicializa os servidores de ação
        self.arm_server = actionlib.SimpleActionServer(
            "arm_desarm_action", ArmDesarmAction, self.execute_arm, False
        )
        self.mode_server = actionlib.SimpleActionServer(
            "set_mode_action", SetModeAction, self.execute_mode, False
        )
        self.takeoff_server = actionlib.SimpleActionServer(
            "takeoff_action", TakeoffAction, self.execute_takeoff, False
        )
        self.land_server = actionlib.SimpleActionServer(
            "land_action", LandAction, self.execute_land, False
        )

        self.arm_server.start()
        self.mode_server.start()
        self.takeoff_server.start()
        self.land_server.start()

        # Inicializa publisher e subscritor
        self.position_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10
        )
        self.current_state = State()
        rospy.Subscriber("/mavros/state", State, self.state_callback)

        # Subscritor para a altitude atual do drone
        self.current_altitude = 0.0
        self.altitude_lock = threading.Lock()
        rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.altitude_callback
        )

        # Serviço MoveCommand
        self.move_command_service = rospy.Service(
            "/move_command", MoveCommand, self.handle_move_command
        )

        # Serviços MAVROS
        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.wait_for_service("/mavros/set_mode")
        self.arm_service = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        # Configuração inicial do PoseStamped para o modo OFFBOARD
        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = 0.0
        self.pose_lock = threading.Lock()  


        self.last_client_pub_time = time.time()

        # Timer para gerenciar o move_command
        self.move_timer = None

        # Inicia a publicação contínua usando rospy.Timer
        self.position_timer = rospy.Timer(
            rospy.Duration(1.0 / 120), self.publish_position
        )

        # Tenta manter o drone no modo OFFBOARD usando rospy.Timer
        self.offboard_timer = rospy.Timer(
            rospy.Duration(1.0 / 120), self.ensure_offboard_mode
        )

        # Define um subscritor para detectar quando o cliente envia posições
        rospy.Subscriber(
            "/mavros/setpoint_position/local",
            PoseStamped,
            self.client_position_callback,
        )

    def state_callback(self, msg):
        """Callback para atualizar o estado atual do drone."""
        self.current_state = msg

    def altitude_callback(self, msg):
        """Callback para atualizar a altitude atual do drone."""
        with self.altitude_lock:
            self.current_altitude = msg.pose.position.z

    def client_position_callback(self, msg):
        """Callback para monitorar a última vez que o cliente publicou uma posição."""
        self.last_client_pub_time = time.time()

    def handle_move_command(self, req):
        """Manipulador para o serviço MoveCommand, ajusta a posição alvo do drone."""
        rospy.loginfo(
            f"Recebendo comando de movimento: x={req.x}, y={req.y}, z={req.z}, yaw={req.yaw}"
        )
        self.move_command_active = True
        self.last_client_pub_time = time.time()  # Atualiza o tempo da última publicação

        with self.pose_lock:
            self.target_pose.pose.position.x = req.x
            self.target_pose.pose.position.y = req.y
            self.target_pose.pose.position.z = req.z

        return MoveCommandResponse(success=True)

    def deactivate_move_command(self, event):
        """Desativa a flag move_command_active após um período de inatividade do move_command."""
        self.move_command_active = False

    def publish_position(self, event):
        """Publica continuamente a posição alvo para manter o modo OFFBOARD."""
        try:
            with self.pose_lock:
                target_pose = self.target_pose
            self.position_pub.publish(target_pose)
        except Exception as e:
            rospy.logerr(f"Erro ao publicar posição: {e}")

    def ensure_offboard_mode(self, event):
        """Garante que o drone esteja no modo OFFBOARD."""
        try:
            if self.current_state.mode != "OFFBOARD":
                rospy.loginfo("Tentando alternar para o modo OFFBOARD...")
                response = self.set_mode_service(custom_mode="OFFBOARD")
                if response.mode_sent:
                    rospy.loginfo("Solicitação para modo OFFBOARD enviada com sucesso.")
                else:
                    rospy.logwarn("Falha ao enviar solicitação para modo OFFBOARD.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Erro ao tentar mudar para o modo OFFBOARD: {e}")
        except Exception as e:
            rospy.logerr(f"Erro na função ensure_offboard_mode: {e}")

    def execute_arm(self, goal):
        feedback = ArmDesarmFeedback()
        result = ArmDesarmResult()

        try:
            response = self.arm_service(goal.arm)
            result.success = response.success
            feedback.message = "Drone armado" if goal.arm else "Drone desarmado"
            self.arm_server.publish_feedback(feedback)
            rospy.loginfo(feedback.message)
            if response.success:
                self.arm_server.set_succeeded(result)
            else:
                self.arm_server.set_aborted(result)
        except rospy.ServiceException as e:
            feedback.message = f"Erro ao chamar o serviço de armamento/desarmamento: {e}"
            rospy.logerr(feedback.message)
            result.success = False
            self.arm_server.set_aborted(result)

    def execute_mode(self, goal):
        feedback = SetModeFeedback()
        result = SetModeResult(success=False)

        rospy.loginfo(f"Solicitando mudança para o modo: {goal.mode}")
        try:
            response = self.set_mode_service(custom_mode=goal.mode)
            result.success = response.mode_sent
            feedback.message = (
                "Modo alterado com sucesso."
                if response.mode_sent
                else "Falha ao alterar o modo."
            )
            self.mode_server.publish_feedback(feedback)
            rospy.loginfo(feedback.message)
            if response.mode_sent:
                self.mode_server.set_succeeded(result)
            else:
                self.mode_server.set_aborted(result)
        except rospy.ServiceException as e:
            feedback.message = f"Erro ao chamar o serviço de modo: {e}"
            rospy.logerr(feedback.message)
            result.success = False
            self.mode_server.set_aborted(result)

    def execute_takeoff(self, goal):
        feedback = TakeoffFeedback()
        result = TakeoffResult()
        target_altitude = goal.altitude

        with self.pose_lock:
            self.target_pose.pose.position.z = target_altitude

        rospy.loginfo(f"Iniciando decolagem para altitude: {target_altitude} metros")
        rate = rospy.Rate(120)

        while not rospy.is_shutdown():
            if self.takeoff_server.is_preempt_requested():
                rospy.loginfo("Decolagem preemptada pelo cliente.")
                result.success = False
                self.takeoff_server.set_preempted(result)
                return

            with self.pose_lock:
                self.position_pub.publish(self.target_pose)

            with self.altitude_lock:
                current_altitude = self.current_altitude

            feedback.current_altitude = current_altitude
            self.takeoff_server.publish_feedback(feedback)
            rospy.loginfo(f"Altitude atual: {feedback.current_altitude:.2f} metros")

            if current_altitude >= target_altitude - 0.1:
                rospy.loginfo("Altitude de decolagem alcançada.")
                result.success = True
                self.takeoff_server.set_succeeded(result)
                return

            rate.sleep()

    def execute_land(self, goal):
        feedback = LandFeedback()
        result = LandResult()

        rospy.loginfo("Iniciando o pouso do drone...")
        rate = rospy.Rate(120)

        while not rospy.is_shutdown():
            if self.land_server.is_preempt_requested():
                rospy.loginfo("Pouso preemptado pelo cliente.")
                result.success = False
                self.land_server.set_preempted(result)
                return

            with self.pose_lock:
                if self.target_pose.pose.position.z > 0.1:
                    self.target_pose.pose.position.z -= 0.1
                    self.position_pub.publish(self.target_pose)
                    feedback.current_altitude = self.target_pose.pose.position.z
                else:
                    rospy.loginfo("Altitude mínima alcançada. Concluindo pouso.")
                    result.success = True
                    self.land_server.set_succeeded(result)
                    break

            self.land_server.publish_feedback(feedback)
            rospy.loginfo(
                f"Altitude atual durante pouso: {feedback.current_altitude:.2f} metros"
            )

            rate.sleep()


if __name__ == "__main__":
    manager = InterfaceManager()
    rospy.spin()
