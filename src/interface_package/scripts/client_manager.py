#!/usr/bin/env python3
import actionlib
import rospy

from interface_package.msg import (ArmDesarmAction, ArmDesarmGoal,
                                   TakeoffAction, TakeoffGoal)


def arm_drone():
    client = actionlib.SimpleActionClient("arm_desarm_action", ArmDesarmAction)
    client.wait_for_server()

    goal = ArmDesarmGoal()
    goal.arm = True

    rospy.loginfo("Enviando comando para armar o drone...")
    client.send_goal(goal)
    client.wait_for_result()

    result = client.get_result()
    if result.success:
        rospy.loginfo("Drone armado com sucesso.")
    else:
        rospy.logwarn("Falha ao armar o drone.")
    return result.success


def takeoff_to_altitude(altitude):
    client = actionlib.SimpleActionClient("takeoff_action", TakeoffAction)
    client.wait_for_server()

    goal = TakeoffGoal()
    goal.altitude = altitude

    rospy.loginfo(f"Iniciando decolagem para {altitude} metros...")
    client.send_goal(goal, feedback_cb=takeoff_feedback_cb)

    # Aguarda o resultado final da decolagem
    client.wait_for_result()
    result = client.get_result()

    if result.success:
        rospy.loginfo("Decolagem bem-sucedida.")
    else:
        rospy.logwarn("Falha ao realizar a decolagem.")
    return result.success


def takeoff_feedback_cb(feedback):
    rospy.loginfo(
        f"Altitude atual durante a decolagem: {feedback.current_altitude:.2f} metros"
    )


def main():
    rospy.init_node("client_manager")

    # Arma o drone
    if not arm_drone():
        rospy.logerr("Não foi possível armar o drone. Abortando teste.")
        return

    # Inicia a decolagem para a altitude desejada
    takeoff_to_altitude(2.0)


if __name__ == "__main__":
    main()
