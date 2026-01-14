"""
Comprehensive error handling and logging system
Standardized error messages with codes for debugging
"""

from datetime import datetime
from config.constants import ERROR_CODES

class RobotError:
    """
    Error handling and logging system for robot operations
    Provides standardized error messages with codes
    """
    
    @staticmethod
    def format_error(code, details="", context=""):
        """
        Format error message with code, details, and context
        
        Args:
            code: Error code (e.g., 'E101')
            details: Detailed error description
            context: Contextual information
        
        Returns:
            Formatted error message string
        """
        error_name = ERROR_CODES.get(code, "Unknown Error")
        msg = f"[{code}] {error_name}"
        if details:
            msg += f"\n  Details: {details}"
        if context:
            msg += f"\n  Context: {context}"
        return msg
    
    @staticmethod
    def log_error(log_file, code, details="", context=""):
        """
        Write error to log file with timestamp
        
        Args:
            log_file: Open file handle for logging
            code: Error code
            details: Detailed error description
            context: Contextual information
        """
        if log_file and not log_file.closed:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            error_msg = RobotError.format_error(code, details, context)
            log_file.write(f"[{timestamp}] ERROR {error_msg}\n")
            log_file.flush()
    
    @staticmethod
    def log_warning(log_file, message):
        """
        Write warning to log file with timestamp
        
        Args:
            log_file: Open file handle for logging
            message: Warning message
        """
        if log_file and not log_file.closed:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            log_file.write(f"[{timestamp}] WARNING {message}\n")
            log_file.flush()
    
    @staticmethod
    def log_info(log_file, message):
        """
        Write info message to log file with timestamp
        
        Args:
            log_file: Open file handle for logging
            message: Info message
        """
        if log_file and not log_file.closed:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            log_file.write(f"[{timestamp}] INFO {message}\n")
            log_file.flush()