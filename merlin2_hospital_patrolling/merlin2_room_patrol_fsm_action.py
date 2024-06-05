# TODO: write the patrol FSM action

from .pddl import room_type, room_patrolled, room_at
from merlin2_basic_actions.merlin2_basic_types import wp_type
from merlin2_basic_actions.merlin2_basic_predicates import robot_at
from merlin2_fsm_action import Merlin2BasicStates
from merlin2_fsm_action import Merlin2FsmAction

from yasmin import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED
from yasmin import CbState

from kant_dto import PddlObjectDto, PddlConditionEffectDto
from geometry_msgs.msg import Twist

import rclpy
import time


class Merlin2RoomPatrolFsmAction(Merlin2FsmAction):
    def __init__(self) -> None:

        self._room = PddlObjectDto(room_type, "room")
        self._wp = PddlObjectDto(wp_type, "wp")

        self.rotate_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        super().__init__("room_patrol")

        tts_state = self.create_state(Merlin2BasicStates.TTS)

        self.add_state(
            "ROTATE", CbState([SUCCEED], self.rotate), transitions={SUCCEED: "TEXT"}
        )

        self.add_state(
            "TEXT", CbState([SUCCEED], self.text), transitions={SUCCEED: "SPEAK"}
        )

        self.add_state("SPEAK", tts_state)

    def rotate(self, blackboard: Blackboard) -> str:
        # Create a Twist message
        twist_msg = Twist()
        twist_msg.angular.z = 1.0  # Set the angular velocity to rotate the robot

        while time.time() < blackboard.start_time + 10:
            self.rotate_pub.publish(twist_msg)
            time.sleep(0.1)

        twist_msg.angular.z = 0.0  # Stop the robot
        self.rotate_pub.publish(twist_msg)

        return SUCCEED

    def text(self, blackboard: Blackboard) -> str:
        room_name = blackboard.merlin2_action_goal.objetcs[0][-1]
        blackboard.text = f"Patrolling room {room_name}."
        return SUCCEED

    def create_parameters(self) -> List[PddlObjectDto]:
        return [self._room, self._wp]

    def create_conditions(self) -> List[PddlConditionEffectDto]:

        cond_1 = PddlConditionEffectDto(
            room_patrolled,
            [self._room],
            PddlConditionEffectDto.AT_START,
            is_negative=False,
        )
        cond_2 = PddlConditionEffectDto(
            robot_at, [self._wp], PddlConditionEffectDto.AT_START
        )
        cond_3 = PddlConditionEffectDto(
            room_at, [self._room, self._wp], PddlConditionEffectDto.AT_START
        )

        return [cond_1, cond_2, cond_3]

    def create_effects(self) -> List[PddlConditionEffectDto]:

        cond_1 = PddlConditionEffectDto(
            room_patrolled, [self._room], PddlConditionEffectDto.AT_END
        )

        return [cond_1]


def main():
    rclpy.init()
    node = Merlin2RoomPatrolFsmAction()
    node.join_spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
