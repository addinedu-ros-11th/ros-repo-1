import logging
import yaml
import numpy as np
from PIL import Image
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
    def __init__(self, yaml_path: str):
        self.load_map_config(yaml_path)
        self.finder = AStarFinder(diagonal_movement=DiagonalMovement.always)

    def load_map_config(self, yaml_path: str):
            """YAML 파일에서 해상도와 원점 정보를 읽고 PGM 이미지를 로드합니다."""
            with open(yaml_path, 'r') as f:
                config = yaml.safe_load(f)
            
            self.resolution = config['resolution']  # 예: 0.05 (5cm)
            self.origin = config['origin']          # [x, y, yaw]
            pgm_filename = config['image']
            
            # PGM 파일 읽기 (절대경로 처리가 필요할 수 있음)
            map_image = Image.open(pgm_filename)
            # 0(검정, 장애물) ~ 255(흰색, 자유공간)
            raw_data = np.array(map_image)

            # 1. 이진화 (ROS 표준: 255는 통로, 0은 벽, 205는 미탐색)
            # pathfinding용: 1(이동가능), 0(장애물)
            binary_map = np.where(raw_data >= 250, 1, 0).astype(np.uint8)

            # 2. 로봇 크기만큼 Inflation 적용
            # 로봇 12cm / 해상도 5cm = 약 2.4칸 -> 안전하게 2~3칸 팽창
            self.static_matrix = self.inflate_map(binary_map, inflation_cells=3)
            # 초기 그리드는 정적 장애물만 포함
            self.matrix = np.copy(self.static_matrix)
            self.grid = Grid(matrix=self.matrix.tolist())
            self.forbidden_zones: List[Dict] = []
            
            logger.info(f"Map Loaded: {self.matrix.shape} grid size with resolution {self.resolution}")

    def update_forbidden_zones(self, zones: List[Dict]):
        """관리자가 설정한 금지 구역을 지도 데이터에 반영합니다."""
        self.forbidden_zones = zones
        # 정적 맵에서 다시 시작
        new_matrix = np.copy(self.static_matrix)
        rows, cols = new_matrix.shape

        for zone in zones:
            # UI에서 전달된 left, top, width, height (픽셀 단위)
            # 맵 이미지 좌표계와 일치한다고 가정
            z_left = int(zone.get('left', 0))
            z_top = int(zone.get('top', 0))
            z_width = int(zone.get('width', 0))
            z_height = int(zone.get('height', 0))

            # 그리드 인덱스 범위 계산 (행: top~top+height, 열: left~left+width)
            r_start, r_end = max(0, z_top), min(rows, z_top + z_height)
            c_start, c_end = max(0, z_left), min(cols, z_left + z_width)

            if r_start < r_end and c_start < c_end:
                new_matrix[r_start:r_end, c_start:c_end] = 0
                logger.info(f"금지 구역 적용: ({c_start}, {r_start}) ~ ({c_end}, {r_end})")

        self.matrix = new_matrix
        # pathfinding 라이브러리의 Grid는 매번 새로 생성하거나 
        # 기존 노드의 walkable 상태를 업데이트해야 함. 
        # 여기서는 단순함을 위해 매번 새로 생성하거나 필드만 업데이트.
        self.grid = Grid(matrix=self.matrix.tolist())
        logger.info(f"금지 구역 {len(zones)}개 반영 완료.")

    def inflate_map(self, matrix: np.ndarray, inflation_cells: int) -> np.ndarray:
        """벽(0) 주변을 지정된 셀 만큼 0으로 채워 로봇 충돌을 방지합니다."""
        inflated = np.copy(matrix)
        rows, cols = matrix.shape
        
        # 실제로는 OpenCV의 erode/dilate를 쓰면 빠르지만, 
        # 간단한 구현을 위해 벽인 곳 주변을 0으로 덮어씁니다.
        wall_indices = np.argwhere(matrix == 0)
        for r, c in wall_indices:
            for dr in range(-inflation_cells, inflation_cells + 1):
                for dc in range(-inflation_cells, inflation_cells + 1):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < rows and 0 <= nc < cols:
                        inflated[nr, nc] = 0
        return inflated
    
    def world_to_grid(self, x_m: float, y_m: float):
        """실제 거리(m)를 그리드 인덱스로 변환"""
        grid_x = int((x_m - self.origin[0]) / self.resolution)
        grid_y = int((y_m - self.origin[1]) / self.resolution)
        # PGM 이미지는 좌상단이 (0,0)이고 ROS 좌표는 좌하단 기준일 수 있으므로 
        # 맵 파일 특성에 따라 y축 반전이 필요할 수 있습니다.
        return grid_x, grid_y

    def grid_to_world(self, gx: int, gy: int):
        """그리드 인덱스를 실제 거리(m)로 변환"""
        x_m = (gx * self.resolution) + self.origin[0]
        y_m = (gy * self.resolution) + self.origin[1]
        return x_m, y_m


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
        start_goal_x, start_goal_y = self.world_to_grid(robot.pose_x, robot.pose_y)
        end_goal_x, end_goal_y = self.world_to_grid(goal_x, goal_y)

        # 그리드 범위 체크
        if not (0 <= start_goal_x < self.grid.width and 0 <= start_goal_y < self.grid.height):
            logger.error("시작점이 맵 밖에 있습니다.")
            return None
        
        start_node = self.grid.node(start_goal_x, start_goal_y)
        end_node = self.grid.node(end_goal_x, end_goal_y)
        
        logger.info(f"[PathPlanner] A* 계산 시작: ({start_node.x}, {start_node.y}) -> ({end_node.x}, {end_node.y})")

        # 경로 계산
        path, runs = self.finder.find_path(start_node, end_node, self.grid)
        
        # 그리드 상태 초기화 (다음 계산을 위해 필요)
        self.grid.cleanup()

        if not path or len(path) == 0:
            logger.warning("경로를 찾을 수 없습니다.")
            return None

        logger.info(f"경로 계산 완료 (단계: {runs}, 길이: {len(path)})")

        result_path = []
        for node in path:
            world_x, world_y = self.grid_to_world(node.x, node.y)
            result_path.append({"x": world_x, "y": world_y})

        # 결과 반환 (딕셔너리 리스트 형태)
        return result_path
