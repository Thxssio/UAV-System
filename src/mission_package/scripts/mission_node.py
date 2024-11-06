#!/usr/bin/env python3
import rospy
import actionlib

from interface_package.msg import TakeoffAction, TakeoffGoal, LandAction, LandGoal
from missions.task1 import Task1

class MissionNode:
    def __init__(self, mission):
        rospy.init_node("mission_node")

        self.takeoff_client = actionlib.SimpleActionClient("takeoff_action", TakeoffAction)
        self.takeoff_client.wait_for_server()

        self.land_client = actionlib.SimpleActionClient("land_action", LandAction)
        self.land_client.wait_for_server()

        self.mission = mission
        rospy.loginfo("MissionNode inicializado e pronto para iniciar a missão.")

    def takeoff(self):
        """Envia o comando de decolagem para a altitude especificada na missão."""
        rospy.loginfo(f"Iniciando decolagem para altitude de {self.mission.takeoff_altitude} metros...")
        goal = TakeoffGoal()
        goal.altitude = self.mission.takeoff_altitude
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

    def run_mission(self):
        """Executa a missão automaticamente."""
        if not self.takeoff():
            rospy.logwarn("Falha na decolagem. Abortando missão.")
            return

        rospy.loginfo("Iniciando a missão.")
        self.mission.run_mission()
        
        rospy.loginfo("Missão concluída. Iniciando pouso.")
        self.land()

if __name__ == "__main__":
    mission = Task1()
    mission_node = MissionNode(mission)
    mission_node.run_mission()
