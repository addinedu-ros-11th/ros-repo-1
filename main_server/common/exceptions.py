"""
애플리케이션 전역에서 사용될 커스텀 예외 클래스
"""

class ApplicationException(Exception):
    """기본 예외 클래스"""
    @property
    def message(self):
        return "An application error occurred."

class DatabaseException(ApplicationException):
    """데이터베이스 관련 예외"""
    @property
    def message(self):
        return "A database error occurred."

class RobotNotFoundException(ApplicationException):
    """로봇을 찾을 수 없을 때 발생하는 예외"""
    @property
    def message(self):
        return "The requested robot was not found."

class TaskAssignmentException(ApplicationException):
    """작업 할당 실패 시 발생하는 예외"""
    @property
    def message(self):
        return "Failed to assign the task."
