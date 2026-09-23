from typing import Any, Dict, List

from rclpy.node import Node

from ..robots.wrapper import RobotInterface
from ..shared.logger import Logger
from .interface import CLIFunctionBase


class CartesianCLIFunction(CLIFunctionBase):
    """Straight-line tool move (MoveIt compute_cartesian_path), e.g. the
    demo's 10 cm approach: `cart 0 0 -0.1`."""
    name = "cart"

    def __init__(self, node: Node, robot: RobotInterface):
        super().__init__(name=self.name)
        self._node = node
        self.robot = robot

    @staticmethod
    def __str__():
        n = CartesianCLIFunction.name
        return (f"Straight-line tool move. Usage: {n} <dx> <dy> <dz> [-tool] [-speed=0.05] "
                f"(meters; world axes, or the tool's own with -tool)")

    def execute(self, args: List[str] = None, kwargs: Dict[str, Any] = None):
        kwargs = kwargs or {}
        try:
            offset = [float(v) for v in (args or [])]
            speed = float(kwargs.get('speed', 0.05))
        except ValueError:
            offset = []
        if len(offset) != 3:
            Logger.WARN(f"Need three numbers. {self}")
            return
        frame = 'tool' if 'tool' in kwargs else 'world'
        if input(f"Move the tool {offset} m ({frame} axes) at {speed} m/s? (y/N): ").strip().lower() != 'y':
            Logger.INFO("Cancelled.")
            return
        result = self.robot.manager.execute_cartesian(offset, frame=frame, max_speed=speed)
        (Logger.INFO if result.success else Logger.ERROR)(
            f"cartesian move: {result.message} ({result.fraction * 100:.0f}% of the line)")
