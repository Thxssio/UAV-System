# missions/task1.py
from missions.mission import Mission
import rospy

class Task1(Mission):
    def __init__(self):
        super().__init__()

    def run_mission(self):
        rospy.loginfo("Iniciando Task1.")
        
        # Define a sequência de movimentos da missão
        self.move_to(x=0, y=0, z=2)
        rospy.sleep(10)
        self.move_to(x=2, y=0, z=2)
        rospy.sleep(10)
        self.move_to(x=0, y=2, z=2)
        rospy.sleep(10)
        self.move_to(x=-2, y=0, z=2)
        rospy.sleep(1)
        
        rospy.loginfo("Task1 concluída.")
