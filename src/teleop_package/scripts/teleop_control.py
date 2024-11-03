#!/usr/bin/env python3
import sys
import termios
import tty

import actionlib
import rospy

from interface_package.msg import (
    ArmDesarmAction,
    ArmDesarmGoal,
    LandAction,
    LandGoal,
    TakeoffAction,
    TakeoffGoal,
)
from interface_package.srv import MoveCommand


class TeleopControl:
    def __init__(self):
        rospy.init_node("teleop_control_node")

        # Configura o cliente para o serviço MoveCommand
        rospy.wait_for_service("move_command")
        self.move_command_client = rospy.ServiceProxy("move_command", MoveCommand)

        # Configura o cliente de ação para ArmDesarm
        self.arm_client = actionlib.SimpleActionClient(
            "arm_desarm_action", ArmDesarmAction
        )
        self.arm_client.wait_for_server()

        # Configura o cliente de ação para Takeoff
        self.takeoff_client = actionlib.SimpleActionClient(
            "takeoff_action", TakeoffAction
        )
        self.takeoff_client.wait_for_server()

        # Configura o cliente de ação para Land
        self.land_client = actionlib.SimpleActionClient("land_action", LandAction)
        self.land_client.wait_for_server()

        # Variável para controlar o loop principal
        self.running = True

        # Posição atual do drone (iniciando em zero)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        rospy.loginfo("Conectado aos serviços e ações.")

    def arm_drone(self):
        """Arma o drone antes de decolar."""
        try:
            rospy.loginfo("Enviando comando para armar o drone...")
            goal = ArmDesarmGoal()
            goal.arm = True
            self.arm_client.send_goal(goal)
            self.arm_client.wait_for_result()
            result = self.arm_client.get_result()
            if result.success:
                rospy.loginfo("Drone armado com sucesso.")
            else:
                rospy.logwarn("Falha ao armar o drone.")
        except Exception as e:
            rospy.logerr(f"Erro ao armar o drone: {e}")

    def takeoff(self, altitude):
        """Envia o comando de decolagem."""
        try:
            rospy.loginfo(f"Enviando comando de decolagem para {altitude} metros...")
            goal = TakeoffGoal()
            goal.altitude = altitude
            self.takeoff_client.send_goal(goal)
            self.takeoff_client.wait_for_result()
            result = self.takeoff_client.get_result()
            if result.success:
                rospy.loginfo("Decolagem bem-sucedida.")
                self.current_z = altitude
            else:
                rospy.logwarn("Falha na decolagem.")
        except Exception as e:
            rospy.logerr(f"Erro na decolagem: {e}")

    def land(self):
        """Envia o comando para pousar."""
        try:
            rospy.loginfo("Comando de pouso iniciado...")
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
        """Captura a tecla pressionada."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            key = sys.stdin.read(1)
        except Exception as e:
            rospy.logerr(f"Erro ao ler a tecla: {e}")
            key = ""
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def process_keyboard_input(self, char):
        """Mapeia teclas para movimentos."""
        command_map = {
            "w": (0.2, 0.0, 0.0),  # Frente
            "s": (-0.2, 0.0, 0.0),  # Trás
            "a": (0.0, 0.2, 0.0),  # Esquerda (corrigido)
            "d": (0.0, -0.2, 0.0),  # Direita (corrigido)
            "q": (0.0, 0.0, 0.2),  # Subir
            "e": (0.0, 0.0, -0.2),  # Descer
        }

        if char.lower() in command_map:
            dx, dy, dz = command_map[char.lower()]
            # Atualiza a posição atual
            self.current_x += dx
            self.current_y += dy
            self.current_z += dz

            # Limita a altitude mínima a 0.1 para evitar valores negativos
            if self.current_z < 0.1:
                self.current_z = 0.1
                rospy.logwarn("Altitude mínima atingida.")

            self.send_move_command(self.current_x, self.current_y, self.current_z)
        elif char.lower() == "x":
            rospy.loginfo("Pousando...")
            self.land()
            self.running = False  # Encerra o loop principal
        elif char == "\x03":  # Ctrl+C
            rospy.loginfo("Interrompido pelo usuário, pousando...")
            self.land()
            self.running = False  # Encerra o loop principal

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
        rospy.loginfo(
            "Use as teclas WASDQE para mover e 'X' para pousar. Ctrl+C para sair."
        )

        # Arma e decola o drone antes de iniciar o controle
        self.arm_drone()
        self.takeoff(1.0)

        rate = rospy.Rate(120)
        while not rospy.is_shutdown() and self.running:
            try:
                char = self.get_key()
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
