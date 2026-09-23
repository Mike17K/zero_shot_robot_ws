import os
from typing import Any, Dict, List

from rclpy.node import Node

from ..robots.wrapper import RobotInterface
from ..shared.constants import CANONICAL_EE_GROUP, CONFIG_FOLDER
from ..shared.logger import Logger
from .interface import CLIFunctionBase

# Named joint configurations, one per line: <key>:<j1>,...,<j6>
SAVED_POSITIONS_FILE = os.path.join(CONFIG_FOLDER, "saved_positions.txt")


class TargetPositionCLIFunction(CLIFunctionBase):
    """ta s <key>  save the current joints under <key>
    ta <key>       plan and move to a saved position (asks y/N first)
    ta i           type joint values one by one, then move
    ta l           list saved positions
    ta clear       delete all saved positions"""
    name = "ta"

    def __init__(self, node: Node, robot: RobotInterface):
        super().__init__(name=self.name)
        self._node = node
        self.robot = robot
        self.file_path = SAVED_POSITIONS_FILE

    @staticmethod
    def __str__():
        n = TargetPositionCLIFunction.name
        return (f"Saved joint positions. Usage: {n} s <key> (save current) | {n} <key> (move) | "
                f"{n} i (type joints) | {n} l (list) | {n} clear")

    def load_saved_positions(self) -> Dict[str, List[float]]:
        if not os.path.exists(self.file_path):
            return {}
        positions = {}
        with open(self.file_path) as f:
            for line in f:
                if ':' in line:
                    key, value = line.strip().split(':', 1)
                    positions[key] = [float(v) for v in value.split(',')]
        return positions

    def _write(self, positions: Dict[str, List[float]]) -> None:
        with open(self.file_path, 'w') as f:
            for k, v in positions.items():
                f.write(f"{k}:{','.join(map(str, v))}\n")

    def _current_joints(self):
        state = self.robot.get_ee_pose_for_group(CANONICAL_EE_GROUP)
        return list(state.joints) if state else None

    def _move(self, joints: List[float], what: str) -> None:
        Logger.INFO(f"Target {what}: " + ", ".join(f"{j:+.3f}" for j in joints))
        if input("Move there? (y/N): ").strip().lower() != 'y':
            Logger.INFO("Cancelled.")
            return
        ok = self.robot.execute_joint_goal(CANONICAL_EE_GROUP, joints)
        (Logger.INFO if ok else Logger.ERROR)(f"Move to {what} {'done' if ok else 'failed'}.")

    def execute(self, args: List[str] = None, kwargs: Dict[str, Any] = None):
        if not args:
            Logger.INFO(str(self))
            return
        mode = args[0].strip()
        positions = self.load_saved_positions()

        if mode == 'clear':
            if os.path.exists(self.file_path):
                os.remove(self.file_path)
            Logger.INFO("All saved positions cleared.")
        elif mode == 'l':
            for k, v in positions.items():
                Logger.INFO(f"  {k}: " + ", ".join(f"{j:+.3f}" for j in v))
            if not positions:
                Logger.INFO("No saved positions.")
        elif mode == 's' and len(args) > 1:
            joints = self._current_joints()
            if joints is None:
                return
            positions[args[1]] = joints
            self._write(positions)
            Logger.INFO(f"Position saved under key '{args[1]}'.")
        elif mode == 'i':
            joints = self._current_joints()
            if joints is None:
                return
            for i, current in enumerate(joints):
                while True:
                    raw = input(f"Joint {i + 1} (current {current:+.4f}, Enter to keep): ").strip()
                    try:
                        joints[i] = float(raw) if raw else current
                        break
                    except ValueError:
                        Logger.WARN("Not a number, try again.")
            self._move(joints, "typed joints")
        elif mode in positions:
            self._move(positions[mode], f"'{mode}'")
        else:
            Logger.WARN(f"No saved position '{mode}'. Known: {', '.join(positions) or 'none'}")
