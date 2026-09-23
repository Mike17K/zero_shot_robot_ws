from typing import Any, Dict, List

from rclpy.node import Node

from ..shared.logger import Logger
from .interface import CLIFunctionBase


class ExampleCLIFunction(CLIFunctionBase):
    """Template for a new command - copy, rename `name`, implement execute(),
    then export it in cli_functions/__init__.py and register it in main.py."""
    name = "example"

    def __init__(self, node: Node):
        super().__init__(name=self.name)
        self._node = node

    @staticmethod
    def __str__():
        return "Description of the functionality. Usage: " + ExampleCLIFunction.name + " <args> [-flag] [-key=value]"

    def execute(self, args: List[str] = None, kwargs: Dict[str, Any] = None):
        Logger.INFO(f"example called with args={args} kwargs={kwargs}")
