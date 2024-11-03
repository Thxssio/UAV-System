#!/usr/bin/env python3
import threading
import time

import actionlib
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

from interface_package.msg import (ArmDesarmAction, ArmDesarmFeedback,
                                   ArmDesarmResult, LandAction, LandFeedback,
                                   LandResult, SetModeAction, SetModeFeedback,
                                   SetModeResult, TakeoffAction,
                                   TakeoffFeedback, TakeoffResult)


class InterfaceManager:
    def __init__(self):
        rospy.init_node("interface_manager")

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
        rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.altitude_callback
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

        # Variável de controle para monitorar o último tempo de publicação do cliente
        self.last_client_pub_time = time.time()

        # Inicia a publicação contínua em uma thread separada
        self.position_thread = threading.Thread(target=self.publish_position)
        self.position_thread.start()

        # Tenta manter o drone no modo OFFBOARD
        self.offboard_thread = threading.Thread(target=self.ensure_offboard_mode)
        self.offboard_thread.start()

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
        self.current_altitude = msg.pose.position.z

    def client_position_callback(self, msg):
        """Callback para monitorar a última vez que o cliente publicou uma posição."""
        self.last_client_pub_time = time.time()

    def publish_position(self):
        """Função que publica continuamente a posição alvo para manter o modo OFFBOARD."""
        rate = rospy.Rate(120)
        rospy.loginfo(
            "Iniciando publicação contínua de posição para manter o modo OFFBOARD..."
        )

        while not rospy.is_shutdown():
            # Se o cliente não publicar por mais de 2 segundos e o drone estiver voando, inicia pouso seguro
            if (
                time.time() - self.last_client_pub_time > 2.0
                and self.current_state.armed
            ):
                rospy.logwarn("Cliente inativo. Iniciando pouso seguro.")
                self.execute_land(None)
                break

            # Publica a posição alvo continuamente
            self.position_pub.publish(self.target_pose)
            rate.sleep()

    def ensure_offboard_mode(self):
        """Função para garantir que o drone esteja no modo OFFBOARD."""
        rate = rospy.Rate(120)
        while not rospy.is_shutdown():
            if self.current_state.mode != "OFFBOARD":
                rospy.loginfo("Tentando alternar para o modo OFFBOARD...")
                try:
                    response = self.set_mode_service(custom_mode="OFFBOARD")
                    if response.mode_sent:
                        rospy.loginfo(
                            "Solicitação para modo OFFBOARD enviada com sucesso. Aguardando confirmação..."
                        )
                        timeout = rospy.Time.now() + rospy.Duration(5)
                        while rospy.Time.now() < timeout:
                            if self.current_state.mode == "OFFBOARD":
                                rospy.loginfo(
                                    "Modo OFFBOARD confirmado pelo estado do drone."
                                )
                                break
                            rospy.sleep(0.1)
                        else:
                            rospy.logwarn(
                                "Não foi possível confirmar o modo OFFBOARD pelo estado do drone."
                            )
                    else:
                        rospy.logwarn("Falha ao enviar solicitação para modo OFFBOARD.")
                except rospy.ServiceException as e:
                    rospy.logerr(f"Erro ao tentar mudar para o modo OFFBOARD: {e}")
            rate.sleep()

    def execute_arm(self, goal):
        feedback = ArmDesarmFeedback()
        result = ArmDesarmResult()

        try:
            response = self.arm_service(goal.arm)
            result.success = response.success
            feedback.message = "Drone armado" if goal.arm else "Drone desarmado"
            self.arm_server.publish_feedback(feedback)
            rospy.loginfo(feedback.message)
            self.arm_server.set_succeeded(result)

        except rospy.ServiceException as e:
            feedback.message = (
                f"Erro ao chamar o serviço de armamento/desarmamento: {e}"
            )
            rospy.logerr(feedback.message)
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
            self.mode_server.set_succeeded(
                result if response.mode_sent else self.mode_server.set_aborted(result)
            )
        except rospy.ServiceException as e:
            feedback.message = f"Erro ao chamar o serviço de modo: {e}"
            rospy.logerr(feedback.message)
            result.success = False
            self.mode_server.set_aborted(result)

    def execute_takeoff(self, goal):
        feedback = TakeoffFeedback()
        result = TakeoffResult()
        target_altitude = goal.altitude
        self.target_pose.pose.position.z = target_altitude

        rospy.loginfo(f"Iniciando decolagem para altitude: {target_altitude} metros")
        rate = rospy.Rate(120)

        while not rospy.is_shutdown():
            if self.takeoff_server.is_preempt_requested():
                rospy.loginfo("Decolagem preemptada pelo cliente.")
                result.success = False
                self.takeoff_server.set_preempted(result)
                return

            self.position_pub.publish(self.target_pose)

            feedback.current_altitude = self.current_altitude
            self.takeoff_server.publish_feedback(feedback)
            rospy.loginfo(f"Altitude atual: {feedback.current_altitude:.2f} metros")

            if self.current_altitude >= target_altitude - 0.2:
                rospy.loginfo("Altitude de decolagem alcançada.")
                result.success = True
                self.takeoff_server.set_succeeded(result)
                return

            rate.sleep()

    def execute_land(self, goal=None):
        feedback = LandFeedback()
        result = LandResult()

        rospy.loginfo("Iniciando o pouso do drone...")
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.target_pose.pose.position.z > 0.1:
                self.target_pose.pose.position.z -= 0.1
                self.position_pub.publish(self.target_pose)
                feedback.current_altitude = self.target_pose.pose.position.z
                if goal:
                    self.land_server.publish_feedback(feedback)
                rospy.loginfo(
                    f"Altitude atual durante pouso: {feedback.current_altitude:.2f} metros"
                )
            else:
                rospy.loginfo("Altitude mínima alcançada. Concluindo pouso.")
                result.success = True
                if goal:
                    self.land_server.set_succeeded(result)
                break

            rate.sleep()


if __name__ == "__main__":
    manager = InterfaceManager()
    rospy.spin()
