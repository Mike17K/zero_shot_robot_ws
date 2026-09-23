import sys
import traceback
from datetime import datetime
# --------------- Logging ----------------

class Logger:
    """Logging utility with colored output."""
    # ANSI color codes
    RED = '\033[91m'
    YELLOW = '\033[93m'
    GREEN = '\033[92m'
    RESET = '\033[0m'  # Resets the color to default

    @staticmethod
    def _timestamp():
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    @staticmethod
    def ERROR(m: str):
        # Print error messages in red
        print(f"[{Logger._timestamp()}] {Logger.RED}ERROR - {m}{Logger.RESET}")
        if sys.exc_info()[0] is not None:  # only when called while handling an exception
            traceback.print_exc()

    @staticmethod
    def INFO(m: str):
        # Print info messages in green
        print(f"[{Logger._timestamp()}] {Logger.GREEN}INFO  - {m}{Logger.RESET}")

    @staticmethod
    def WARN(m: str):
        # Print warning messages in yellow
        print(f"[{Logger._timestamp()}] {Logger.YELLOW}WARN  - {m}{Logger.RESET}")

