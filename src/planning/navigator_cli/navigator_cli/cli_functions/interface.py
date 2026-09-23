from abc import ABC, abstractmethod

from typing import Dict, List, Optional, Any

###

class CLIFunctionBase(ABC):
    name: str
    accept_kwargs = True
    
    @abstractmethod
    def __init__(self, name: str):
        self.name = name

    @abstractmethod
    def execute(self, args: Optional[List[str]] = None, kwargs: Optional[Dict[str, Any]] = None):
        raise NotImplementedError("This is an abstract class.")
    
    @staticmethod
    def __str__():
        raise NotImplementedError("This is an abstract class.")

__all__ = ['CLIFunctionBase']