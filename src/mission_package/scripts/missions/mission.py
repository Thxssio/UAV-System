# missions/mission.py
from abc import ABC, abstractmethod
import rospy
from geometry_msgs.msg import PoseStamped
from interface_package.srv import MoveCommand
import numpy as np

class Mission(ABC):
    def __init__(self):
        rospy.wait_for_service("move_command")
        self.move_command_client = rospy.ServiceProxy("move_command", MoveCommand)
        self.takeoff_altitude = 1.5
        self.current_position = None  # Armazena a posição atual do drone

        # Inscrição no tópico de posição
        rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped, self.position_callback)

    def position_callback(self, msg):
        """Callback para atualizar a posição atual do drone."""
        self.current_position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def get_current_position(self):
        """Retorna a posição atual (x, y, z) ou uma posição padrão se não houver dados."""
        if self.current_position:
            return self.current_position
        else:
            rospy.logwarn("Posição atual não disponível, retornando posição padrão.")
            return (0.0, 0.0, self.takeoff_altitude)  # Retorno padrão até que a posição esteja disponível

    @abstractmethod
    def run_mission(self):
        """Método abstrato para definir a missão específica."""
        pass

    def move_to_smooth(self, x, y, z, step=0.01):
        """Move o drone para uma posição alvo (x, y, z) de forma suave, em incrementos menores."""
        current_x, current_y, current_z = self.get_current_position()
        
        direction = np.array([x - current_x, y - current_y, z - current_z])
        distance = np.linalg.norm(direction)
        if distance == 0:
            rospy.loginfo("O drone já está na posição desejada.")
            return
        direction = direction / distance

        # Divide o movimento em pequenos incrementos para suavizar
        num_steps = int(distance / step)
        for i in range(num_steps):
            new_position = np.array([current_x, current_y, current_z]) + direction * step * (i + 1)
            self.send_move_command(new_position[0], new_position[1], new_position[2])
            rospy.sleep(0.1)  # Tempo entre cada movimento para suavidade

        # Envia o comando final para garantir que atinge a posição alvo
        self.send_move_command(x, y, z)
        rospy.loginfo(f"Movimento suave concluído para a posição (x={x}, y={y}, z={z})")

    def send_move_command(self, x, y, z):
        """Envia o comando de movimento ao serviço MoveCommand."""
        try:
            response = self.move_command_client(x=x, y=y, z=z, yaw=0.0)
            if not response.success:
                rospy.logwarn("Falha ao mover o drone para a posição desejada.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Erro no serviço MoveCommand: {e}")
