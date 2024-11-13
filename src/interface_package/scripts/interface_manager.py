#!/usr/bin/env python3
import actionlib
import rospy
import threading


from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from interface_package.msg import (
    ArmDesarmAction,
    ArmDesarmFeedback,
    ArmDesarmResult,
    SetModeAction,
    SetModeFeedback,
    SetModeResult,
    TakeoffAction,
    TakeoffFeedback,
    TakeoffResult,
    LandAction,
    LandFeedback,
    LandResult,
)
from interface_package.srv import MoveCommand, MoveCommandResponse


class InterfaceManager:
    def __init__(self):
        rospy.init_node("interface_manager")
        self.current_state = State()
        self.current_altitude = 0.0

        self.target_pose = PoseStamped()
        self.received_move_command = False

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

        # Publisher e Subscriber
        self.position_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10
        )

        self.vision_pose_pub = rospy.Publisher(
            "/mavros/vision_pose/pose", PoseStamped, queue_size=10
        )

        rospy.Subscriber("/mavros/state", State, self.state_callback)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.altitude_callback)    
        rospy.Subscriber("/vision_pose", PoseStamped, self.vision_pose_callback)
        rospy.Subscriber("/vision_pose/pose_conv", PoseStamped, self.vision_pose_callback)


        # Serviço MoveCommand
        self.move_command_service = rospy.Service(
            "/move_command", MoveCommand, self.handle_move_command
        )

        # Serviços MAVROS
        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.wait_for_service("/mavros/set_mode")
        self.arm_service = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        # Configuração inicial da posição alvo
        self.target_pose = PoseStamped()
        rospy.loginfo("Aguardando conexão com o drone...")
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.sleep(1)
        rospy.loginfo("Drone conectado.")

        # Define o modo inicial como AUTO.LOITER uma vez, na inicialização
        self.set_initial_auto_loiter_mode()

        self.keep_publishing = True
        self.publisher_thread = threading.Thread(target=self.publish_position)
        self.publisher_thread.start()


    def state_callback(self, msg):
        """Callback para atualizar o estado atual do drone."""
        self.current_state = msg

    def altitude_callback(self, msg):
        """Atualiza a altitude atual do drone."""
        self.current_altitude = msg.pose.position.z
        
    def vision_pose_callback(self, msg):
        """Callback que redireciona vision_pose e vision_pose/pose_conv para /mavros/vision_pose/pose."""
        rospy.loginfo(f"Redirecionando pose de visão para /mavros/vision_pose/pose: posição=({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z})")
        self.vision_pose_pub.publish(msg)

    def set_initial_auto_loiter_mode(self):
        """Define o modo inicial como AUTO.LOITER uma vez ao iniciar o nó."""
        try:
            response = self.set_mode_service(custom_mode="AUTO.LOITER")
            if response.mode_sent:
                rospy.loginfo("Modo inicial definido para AUTO.LOITER.")
            else:
                rospy.logwarn("Falha ao definir o modo inicial AUTO.LOITER.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Erro ao definir o modo AUTO.LOITER: {e}")

    def publish_position(self):
        rate = rospy.Rate(120)  # Publicação a cada 0,1s
        while not rospy.is_shutdown() and self.keep_publishing:
            # Publica a posição atual do target_pose, que será zero se nenhum comando for recebido
            self.position_pub.publish(self.target_pose)
            rate.sleep()

    def switch_to_offboard_mode(self):
        """Alterna para o modo OFFBOARD somente se o drone estiver armado e verifica se a mudança foi bem-sucedida."""
        if not self.current_state.armed:
            rospy.logwarn("O drone precisa estar armado para mudar para o modo OFFBOARD.")
            return False

        if self.current_state.mode != "OFFBOARD":
            try:
                for attempt in range(3):  # Tenta até 3 vezes
                    response = self.set_mode_service(custom_mode="OFFBOARD")
                    if response.mode_sent:
                        rospy.loginfo("Tentativa de ativar o modo OFFBOARD.")

                        rate = rospy.Rate(10)  # Frequência de verificação de 10 Hz
                        for _ in range(10):  # Verifica por até 1 segundo (10 tentativas a 10 Hz)
                            if self.current_state.mode == "OFFBOARD":
                                rospy.loginfo("Modo OFFBOARD ativado com sucesso.")
                                return True
                            rate.sleep()
                    
                    rospy.logwarn("Tentativa de ativar o modo OFFBOARD falhou. Retentando...")
                    rospy.sleep(0.5)  # Pausa entre as tentativas

                rospy.logwarn("Falha ao ativar o modo OFFBOARD após várias tentativas.")
                return False
            except rospy.ServiceException as e:
                rospy.logerr(f"Erro ao tentar mudar para o modo OFFBOARD: {e}")
                return False
        return True


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
        result = SetModeResult()

        try:
            response = self.set_mode_service(custom_mode=goal.mode)
            result.success = response.mode_sent
            feedback.message = "Modo alterado com sucesso." if response.mode_sent else "Falha ao alterar o modo."
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

        rospy.loginfo("Iniciando sequência de decolagem...")

        # Chama a ação de armar o drone
        rospy.loginfo("Tentando armar o drone...")
        try:
            arm_response = self.arm_service(True)
            if not arm_response.success:
                rospy.logerr("Falha ao armar o drone.")
                result.success = False
                self.takeoff_server.set_aborted(result)
                return
            rospy.loginfo("Drone armado com sucesso.")

            # Zera os valores de posição ao armar o drone
            self.target_pose.pose.position.x = 0.0
            self.target_pose.pose.position.y = 0.0
            self.target_pose.pose.position.z = 0.0
            rospy.loginfo("Posição alvo resetada após armar o drone.")

            # Espera até que o drone esteja efetivamente armado no estado atual
            rate = rospy.Rate(10)  # Frequência de verificação de 10 Hz
            for _ in range(10):  # Verifica por até 1 segundo
                if self.current_state.armed:
                    break
                rate.sleep()

            if not self.current_state.armed:
                rospy.logwarn("Falha ao confirmar o estado de armamento.")
                result.success = False
                self.takeoff_server.set_aborted(result)
                return
        except rospy.ServiceException as e:
            rospy.logerr(f"Erro ao armar o drone: {e}")
            result.success = False
            self.takeoff_server.set_aborted(result)
            return

        # Alterna para o modo OFFBOARD
        if not self.switch_to_offboard_mode():
            result.success = False
            self.takeoff_server.set_aborted(result)
            return

        # Define a altitude alvo para decolagem
        self.target_pose.pose.position.z = target_altitude
        rospy.loginfo(f"Subindo até {target_altitude} metros...")

        rate = rospy.Rate(120)  # Frequência de 120 Hz para o loop de decolagem

        while not rospy.is_shutdown():
            # Publica o setpoint para manter a altitude desejada
            self.position_pub.publish(self.target_pose)

            # Atualiza o feedback com a altitude atual
            feedback.current_altitude = self.current_altitude
            self.takeoff_server.publish_feedback(feedback)

            # Verifica se o drone atingiu a altitude alvo com uma margem de 0.1 metros
            if self.current_altitude >= target_altitude - 0.1:
                rospy.loginfo("Altitude alvo alcançada.")
                result.success = True
                self.takeoff_server.set_succeeded(result)
                rospy.loginfo("Mantendo a altitude de decolagem.")
                break

            # Verifica se a ação foi preemptada
            if self.takeoff_server.is_preempt_requested():
                rospy.loginfo("Decolagem preemptada pelo cliente.")
                result.success = False
                self.takeoff_server.set_preempted(result)
                return

            rate.sleep()
            
    def execute_land(self, goal):
        feedback = LandFeedback()
        result = LandResult()

        rospy.loginfo("Iniciando o pouso do drone...")

        # Tenta mudar para o modo AUTO.LAND para iniciar o pouso automático
        try:
            response = self.set_mode_service(custom_mode="AUTO.LAND")
            if not response.mode_sent:
                rospy.logwarn("Falha ao ativar o modo AUTO.LAND.")
                result.success = False
                self.land_server.set_aborted(result)
                return
        except rospy.ServiceException as e:
            rospy.logerr(f"Erro ao tentar mudar para o modo AUTO.LAND: {e}")
            result.success = False
            self.land_server.set_aborted(result)
            return

        rate = rospy.Rate(120)  # Define uma frequência de verificação de 120 Hz durante o pouso
        while not rospy.is_shutdown():
            feedback.current_altitude = self.current_altitude
            self.land_server.publish_feedback(feedback)

            # Verifica se a altitude está próxima de zero para indicar que o pouso foi concluído
            if self.current_altitude <= 0.3:
                rospy.loginfo("Altitude mínima alcançada. Verificando estado de armamento para concluir pouso.")

                # Espera até que o drone esteja desarmado após o pouso
                if not self.current_state.armed:
                    rospy.loginfo("Drone desarmado após pouso. Concluindo sequência de pouso.")
                    result.success = True
                    self.land_server.set_succeeded(result)

                    # Muda para o modo AUTO.LOITER após o pouso e desarmamento
                    self.set_initial_auto_loiter_mode()

                    # Ajusta a posição atual como a nova origem
                    self.target_pose.pose.position.x = 0.0
                    self.target_pose.pose.position.y = 0.0
                    self.target_pose.pose.position.z = 0.0
                    rospy.loginfo("Posição alvo resetada após pouso.")
                    self.update_origin()
                    break

            # Verifica se a ação foi preemptada (interrompida pelo cliente)
            if self.land_server.is_preempt_requested():
                rospy.loginfo("Pouso preemptado pelo cliente.")
                result.success = False
                self.land_server.set_preempted(result)
                return

            rate.sleep()

    def update_origin(self):
        """Atualiza a origem da posição para as coordenadas atuais."""
        current_position = self.get_current_position()
        self.origin_x = current_position.x
        self.origin_y = current_position.y
        rospy.loginfo(f"Nova origem definida em ({self.origin_x}, {self.origin_y}).")

    def get_current_position(self):
        """Retorna a posição atual do drone."""
        return self.target_pose.pose.position

    def shutdown(self):
        self.keep_publishing = False
        self.publisher_thread.join()


    def handle_move_command(self, req):
            rospy.loginfo(f"Recebendo comando de movimento: x={req.x}, y={req.y}, z={req.z}, yaw={req.yaw}")
            self.received_move_command = True  # Comando recebido

            # Atualiza o target_pose com os valores recebidos
            self.target_pose.pose.position.x = req.x
            self.target_pose.pose.position.y = req.y
            self.target_pose.pose.position.z = req.z
            self.target_pose.pose.orientation.z = req.yaw  # Supondo que o yaw seja representado assim

            return MoveCommandResponse(success=True)


if __name__ == "__main__":
    manager = InterfaceManager()
    rospy.on_shutdown(manager.shutdown)
    rospy.spin()
