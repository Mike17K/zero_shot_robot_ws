import os

# Global tolerance for float comparisons (m for positions, rad for angles)
FLOAT_TOLERANCE = 1e-6

# The GP70L has a single MoveIt planning group (group_a_description's SRDF).
CANONICAL_EE_GROUP = 'manipulator'
AVAILABLE_GROUPS = [CANONICAL_EE_GROUP]


def _default_config_folder() -> str:
    """NAVIGATOR_CLI_CONFIG_DIR, else the package's SOURCE config folder
    when running inside the workspace (so taught graphs land in git), else
    the installed share copy."""
    if os.environ.get('NAVIGATOR_CLI_CONFIG_DIR'):
        return os.environ['NAVIGATOR_CLI_CONFIG_DIR']
    ws = os.environ.get('ISAAC_ROS_WS')
    if ws:
        src = os.path.join(ws, 'src', 'planning', 'navigator_cli', 'config')
        if os.path.isdir(src):
            return src
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('navigator_cli'), 'config')


CONFIG_FOLDER = _default_config_folder()
