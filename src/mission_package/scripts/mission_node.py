#!/usr/bin/env python3
import rospy
import actionlib
from interface_package.msg import TakeoffAction, TakeoffGoal, LandAction, LandGoal
from interface_package.srv import ExecuteMission, ExecuteMissionResponse
from missions.task1 import Task1

class MissionNode:
    def __init__(self):
        rospy.init_node("mission_node")

        self.takeoff_client = actionlib.SimpleActionClient("takeoff_action", TakeoffAction)
        self.takeoff_client.wait_for_server()

        self.land_client = actionlib.SimpleActionClient("land_action", LandAction)
        self.land_client.wait_for_server()

        # Dicionário de missões: associe o nome da missão à sua classe
        self.missions = {
            "Task1": Task1()
        }

        # Serviço para executar a missão escolhida
        self.mission_service = rospy.Service(
            "/execute_mission", ExecuteMission, self.execute_mission_callback
        )

        rospy.loginfo("MissionNode inicializado e aguardando comandos de missão.")

    def takeoff(self, altitude):
        """Envia o comando de decolagem para a altitude especificada."""
        rospy.loginfo(f"Iniciando decolagem para altitude de {altitude} metros...")
        goal = TakeoffGoal()
        goal.altitude = altitude
        self.takeoff_client.send_goal(goal)
        self.takeoff_client.wait_for_result()
        result = self.takeoff_client.get_result()
        return result.success

    def land(self):
        """Envia o comando para pousar."""
        rospy.loginfo("Iniciando pouso...")
        goal = LandGoal()
        self.land_client.send_goal(goal)
        self.land_client.wait_for_result()
        result = self.land_client.get_result()
        return result.success

    def execute_mission_callback(self, req):
        """Callback para o serviço de execução de missão."""
        mission_name = req.mission_name

        if mission_name not in self.missions:
            message = f"Missão '{mission_name}' não encontrada."
            rospy.logwarn(message)
            return ExecuteMissionResponse(success=False, message=message)

        # Inicia a missão
        rospy.loginfo(f"Iniciando a missão '{mission_name}'.")
        mission = self.missions[mission_name]

        # Executa a decolagem
        if not self.takeoff(mission.takeoff_altitude):
            message = "Falha na decolagem. Abortando missão."
            rospy.logwarn(message)
            return ExecuteMissionResponse(success=False, message=message)

        # Executa a missão
        mission.run_mission()
        rospy.loginfo(f"Missão '{mission_name}' concluída. Iniciando pouso.")

        # Executa o pouso
        if not self.land():
            message = "Falha no pouso."
            rospy.logwarn(message)
            return ExecuteMissionResponse(success=False, message=message)

        # Retorna sucesso
        message = f"Missão '{mission_name}' executada com sucesso."
        rospy.loginfo(message)
        return ExecuteMissionResponse(success=True, message=message)

if __name__ == "__main__":
    mission_node = MissionNode()
    rospy.spin()
