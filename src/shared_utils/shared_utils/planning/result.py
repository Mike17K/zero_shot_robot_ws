"""Types shared by the planning clients (cuMotion and MoveIt), so callers can
swap one planner for the other."""
from dataclasses import dataclass, field
from typing import Optional

from moveit_msgs.msg import MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory

GP70L_JOINTS = ('gp_joint_1', 'gp_joint_2', 'gp_joint_3', 'gp_joint_4', 'gp_joint_5', 'gp_joint_6')

_ERROR_NAMES = {v: k for k, v in vars(MoveItErrorCodes).items()
                if k.isupper() and isinstance(v, int)}


@dataclass
class PlanResult:
    success: bool
    message: str = ''
    error_code: int = MoveItErrorCodes.FAILURE
    # One trajectory for joint/pose goals; for a cuMotion grasp: approach
    # (start -> pre-grasp), then grasp (pre-grasp -> grasp) and retract, as requested.
    trajectories: list[JointTrajectory] = field(default_factory=list)
    goal_index: int = -1  # which goal of a goal set was reached
    planning_time: float = 0.0

    @property
    def error_name(self) -> str:
        return _ERROR_NAMES.get(self.error_code, str(self.error_code))

    @property
    def trajectory(self) -> Optional[JointTrajectory]:
        return self.trajectories[0] if self.trajectories else None
