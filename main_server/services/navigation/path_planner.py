import logging
from typing import List, Optional, Dict
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

from main_server.domains.robots.schemas import Robot

logger = logging.getLogger(__name__)

class PathPlannerService:
    """
    pathfinding 라이브러리를 사용하여 로봇의 전역 경로를 계획하는 서비스.
    """
    def __init__(self):
        # 1: 이동 가능, 0: 장애물 (pathfinding 라이브러리는 1 이상을 이동 가능으로 판단)
        # 100x100 그리드 맵 예시
        self.matrix = [[1] * 100 for _ in range(100)]
        self.grid = Grid(matrix=self.matrix)

    async def plan_global_path(
        self, 
        robot: Robot, 
        goal_x: float, 
        goal_y: float
    ) -> Optional[List[Dict[str, float]]]:
        """
        pathfinding 라이브러리의 A* 알고리즘을 사용하여 경로를 계산합니다.
        """
        # 시작점과 목적지 설정
        start_node = self.grid.node(int(robot.pose_x), int(robot.pose_y))
        end_node = self.grid.node(int(goal_x), int(goal_y))

        # Finder 초기화 (대각선 이동 허용)
        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        
        logger.info(f"[PathPlanner] A* 계산 시작: ({start_node.x}, {start_node.y}) -> ({end_node.x}, {end_node.y})")

        # 경로 계산
        path, runs = finder.find_path(start_node, end_node, self.grid)
        
        # 그리드 상태 초기화 (다음 계산을 위해 필요)
        self.grid.cleanup()

        if not path or len(path) == 0:
            logger.warning("경로를 찾을 수 없습니다.")
            return None

        logger.info(f"경로 계산 완료 (단계: {runs}, 길이: {len(path)})")

        # 결과 반환 (딕셔너리 리스트 형태)
        return [{"x": float(node.x), "y": float(node.y)} for node in path]

    def update_map(self, new_matrix: List[List[int]]):
        """
        로봇으로부터 받은 점유 격자 지도(Occupancy Grid) 정보를 업데이트합니다.
        ROS 2의 0~100 값을 라이브러리용 0(벽), 1(길)로 변환해야 합니다.
        """
        # 예: ROS 2에서 100(장애물)은 0으로, 0(자유공간)은 1로 변환
        self.matrix = [[(1 if val < 50 else 0) for val in row] for row in new_matrix]
        self.grid = Grid(matrix=self.matrix)
