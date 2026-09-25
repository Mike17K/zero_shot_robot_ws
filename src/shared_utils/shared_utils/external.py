"""Non-ROS libraries vendored under <workspace>/external (git submodules,
see scripts/setup_external.sh) - put them on sys.path at runtime:

    from shared_utils.external import add_external_to_path, external_file
    add_external_to_path('MobileSAM', 'pytorch-image-models')
    from mobile_sam import sam_model_registry
    weights = external_file('weights', 'mobile_sam.pt')

The external directory is $ZSR_EXTERNAL_DIR if set, else the container
workspace's (/workspaces/isaac_ros-dev/external).
"""
import os
import sys

DEFAULT_EXTERNAL_DIR = '/workspaces/isaac_ros-dev/external'


def external_dir() -> str:
    return os.environ.get('ZSR_EXTERNAL_DIR', DEFAULT_EXTERNAL_DIR)


def external_file(*parts: str) -> str:
    return os.path.join(external_dir(), *parts)


def add_external_to_path(*names: str) -> None:
    """Prepend external/<name> to sys.path for each name (idempotent)."""
    for name in names:
        path = external_file(name)
        if not os.path.isdir(path) or not os.listdir(path):
            raise ImportError(
                f'external library {name!r} not found at {path} - run scripts/setup_external.sh '
                f'(or set ZSR_EXTERNAL_DIR to the directory holding it)')
        if path not in sys.path:
            sys.path.insert(0, path)
