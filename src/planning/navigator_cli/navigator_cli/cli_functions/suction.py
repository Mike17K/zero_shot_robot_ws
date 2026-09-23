from typing import Any, Dict, List

from rclpy.node import Node

from ..robots.wrapper import RobotInterface
from ..shared.logger import Logger
from .interface import CLIFunctionBase


class SuctionCLIFunction(CLIFunctionBase):
    """Switch the suction gripper (group_a_bringup's gripper_manager)."""
    name = "suction"
    accept_kwargs = False

    def __init__(self, node: Node, robot: RobotInterface):
        super().__init__(name=self.name)
        self._node = node
        self.robot = robot

    @staticmethod
    def __str__():
        return "Suction gripper on/off. Usage: " + SuctionCLIFunction.name + " on|off"

    def execute(self, args: List[str] = None, kwargs: Dict[str, Any] = None):
        if not args or args[0] not in ('on', 'off'):
            Logger.INFO(str(self))
            return
        if not self.robot.manager.set_suction(args[0] == 'on'):
            Logger.ERROR("Suction command failed (is gripper_manager running?).")
