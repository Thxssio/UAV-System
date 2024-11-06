# missions/mission.py
from abc import ABC, abstractmethod
import rospy
from interface_package.srv import MoveCommand

class Mission(ABC):
    def __init__(self):
        rospy.wait_for_service("move_command")
        self.move_command_client = rospy.ServiceProxy("move_command", MoveCommand)
        self.takeoff_altitude = 1.5 

    @abstractmethod
    def run_mission(self):
        """Método abstrato que define o fluxo da missão."""
        pass

    def move_to(self, x, y, z):
        """Envia um comando para mover o drone para uma posição específica."""
        try:
            response = self.move_command_client(x=x, y=y, z=z, yaw=0.0)
            if response.success:
                rospy.loginfo(f"Movimento bem-sucedido para (x={x}, y={y}, z={z})")
            else:
                rospy.logwarn("Falha ao mover o drone para a posição desejada.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Erro no serviço MoveCommand: {e}")
