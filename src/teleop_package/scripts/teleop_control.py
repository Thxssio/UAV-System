#!/usr/bin/env python3
import sys
import termios
import tty
import select
import actionlib
import rospy
from interface_package.msg import (
    TakeoffAction,
    TakeoffGoal,
    LandAction,
    LandGoal,
)
from interface_package.srv import MoveCommand


class TeleopControl:
    def __init__(self):
        rospy.init_node("teleop_control_node")

        # Inicializa o cliente do serviço de movimento
        rospy.wait_for_service("move_command")
        self.move_command_client = rospy.ServiceProxy("move_command", MoveCommand)

        # Inicializa os clientes de ação
        self.takeoff_client = actionlib.SimpleActionClient("takeoff_action", TakeoffAction)
        self.takeoff_client.wait_for_server()

        self.land_client = actionlib.SimpleActionClient("land_action", LandAction)
        self.land_client.wait_for_server()

        # Define a altitude inicial desejada
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 2.0  # Altitude inicial após decolagem ajustada para 2.0 metros
        self.running = True

        rospy.loginfo("Conectado aos serviços e ações.")

    def takeoff(self, altitude):
        """Envia o comando de decolagem."""
        try:
            rospy.loginfo(f"Iniciando decolagem para altitude de {altitude} metros...")
            goal = TakeoffGoal()
            goal.altitude = altitude
            self.takeoff_client.send_goal(goal)
            self.takeoff_client.wait_for_result()
            result = self.takeoff_client.get_result()
            if result.success:
                rospy.loginfo("Decolagem bem-sucedida.")
                self.current_z = altitude  # Atualiza a altitude atual para o valor desejado
            else:
                rospy.logwarn("Falha na decolagem.")
        except Exception as e:
            rospy.logerr(f"Erro na decolagem: {e}")

    def land(self):
        """Envia o comando para pousar."""
        try:
            rospy.loginfo("Iniciando o pouso...")
            goal = LandGoal()
            self.land_client.send_goal(goal)
            self.land_client.wait_for_result()
            result = self.land_client.get_result()
            if result.success:
                rospy.loginfo("Pouso bem-sucedido.")
                self.current_z = 0.0
            else:
                rospy.logwarn("Falha no pouso.")
        except Exception as e:
            rospy.logerr(f"Erro ao pousar: {e}")

    def get_key(self):
        """Captura a tecla pressionada usando termios e tty."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                return key
        except Exception as e:
            rospy.logerr(f"Erro ao ler a tecla: {e}")
            return ""
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def process_keyboard_input(self, char):
        """Mapeia teclas para movimentos."""
        command_map = {
            "w": (0.2, 0.0, 0.0),
            "s": (-0.2, 0.0, 0.0),
            "a": (0.0, 0.2, 0.0),
            "d": (0.0, -0.2, 0.0),
            "q": (0.0, 0.0, 0.2),
            "e": (0.0, 0.0, -0.2),
        }

        if char.lower() in command_map:
            dx, dy, dz = command_map[char.lower()]
            self.current_x += dx
            self.current_y += dy
            self.current_z = max(self.current_z + dz, 0.1)  # Limita a altitude mínima a 0.1

            self.send_move_command(self.current_x, self.current_y, self.current_z)
        elif char.lower() == "x":
            rospy.loginfo("Comando de pouso acionado.")
            self.land()
            self.running = False
        elif char == "\x03":  # Ctrl+C para interromper
            rospy.loginfo("Interrompido pelo usuário, iniciando pouso...")
            self.land()
            self.running = False

    def send_move_command(self, x, y, z):
        """Envia o comando de movimento ao serviço MoveCommand."""
        try:
            response = self.move_command_client(x=x, y=y, z=z, yaw=0.0)
            if response.success:
                rospy.loginfo(f"Movimento enviado: x={x:.2f}, y={y:.2f}, z={z:.2f}")
            else:
                rospy.logwarn("Falha ao enviar comando de movimento.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Erro no serviço MoveCommand: {e}")

    def run(self):
        """Loop principal para captura e envio dos comandos de teleoperação."""
        rospy.loginfo("Use as teclas WASDQE para mover e 'X' para pousar. Ctrl+C para sair.")

        # Executa a decolagem antes de iniciar o controle
        self.takeoff(self.current_z)  # Decolagem para a altitude definida

        rate = rospy.Rate(120)  # Frequência de execução para acompanhar o InterfaceManager
        while not rospy.is_shutdown() and self.running:
            try:
                char = self.get_key()
                if char:
                    self.process_keyboard_input(char)
            except Exception as e:
                rospy.logerr(f"Erro no loop principal: {e}")
            rate.sleep()

        # Garante que o drone pouse ao encerrar
        if self.running:
            self.land()


if __name__ == "__main__":
    teleop = TeleopControl()
    teleop.run()
