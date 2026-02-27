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
            import os
            with open(yaml_path, 'r') as f:
                config_data = yaml.safe_load(f)
            
            self.resolution = config_data['resolution']  # 예: 0.05 (5cm)
            self.origin = config_data['origin']          # [x, y, yaw]
            pgm_filename = config_data['image']
            
            # PGM 파일 경로를 YAML 파일 위치 기준으로 절대 경로화
            if not os.path.isabs(pgm_filename):
                yaml_dir = os.path.dirname(os.path.abspath(yaml_path))
                pgm_path = os.path.join(yaml_dir, pgm_filename)
            else:
                pgm_path = pgm_filename
                
            logger.info(f"Loading map image from: {pgm_path}")
            map_image = Image.open(pgm_path)
            # 0(검정, 장애물) ~ 255(흰색, 자유공간)
            raw_data = np.array(map_image)
            self.height, self.width = raw_data.shape

            # ROS 맵 보정용 설정 (나중에 바꾸기 쉽게 필드로 유지)
            self.map_invert_y = True 

            # 1. 이진화 (ROS 표준: 255는 통로, 0은 벽, 205는 미탐색)
            # 문턱값을 200으로 완화하여 밝은 회색(미탐색/노이즈)도 최대한 통로로 포함
            binary_map = np.where(raw_data >= 200, 1, 0).astype(np.uint8)

            # 2. 로봇 크기만큼 Inflation 적용
            # 로봇 12cm / 해상도 5cm = 약 2.4칸 -> 안전하게 2~3칸 팽창
            # 테스트를 위해 임시로 1셀(5cm)로 축소
            self.static_matrix = self.inflate_map(binary_map, inflation_cells=0)
            self.binary_map = binary_map # 진단용 저장
            # 초기 그리드는 정적 장애물만 포함
            self.matrix = np.copy(self.static_matrix)
            self.grid = Grid(matrix=self.matrix.tolist())
            self.forbidden_zones: List[Dict] = []
            
            logger.info(f"Map Loaded: {self.width}x{self.height} (Res: {self.resolution}, Origin: {self.origin})")

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
        """실제 거리(m)를 그리드 인덱스로 변환 (Y축 반전만 적용, 범위 체크는 호출부 위임)"""
        gx = int((x_m - self.origin[0]) / self.resolution)
        gy = int((y_m - self.origin[1]) / self.resolution)
        
        # Matrix 좌표계(상단0)와 World 좌표계(하단0) 보정
        if hasattr(self, 'map_invert_y') and self.map_invert_y:
            gy = (self.height - 1) - gy

        # Clamping 제거: 범위를 벗어난 값 그대로 반환하여 호출부에서 에러 처리하도록 함
        return gx, gy

    def grid_to_world(self, gx: int, gy: int):
        """그리드 인덱스를 실제 거리(m)로 변환 (반전 포함)"""
        raw_gy = gy
        if hasattr(self, 'map_invert_y') and self.map_invert_y:
            raw_gy = (self.height - 1) - gy
            
        x_m = (gx * self.resolution) + self.origin[0]
        y_m = (raw_gy * self.resolution) + self.origin[1]
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
        start_gx, start_gy = self.world_to_grid(robot.pose_x, robot.pose_y)
        end_gx, end_gy = self.world_to_grid(goal_x, goal_y)

        logger.info(f"[PathPlanner] A* 계산 요청: World({robot.pose_x:.2f}, {robot.pose_y:.2f}) -> World({goal_x:.2f}, {goal_y:.2f})")

        # 그리드 범위 체크 (Strict Validation)
        if not (0 <= start_gx < self.width and 0 <= start_gy < self.height):
            logger.error(f"시작점이 맵 범위를 벗어났습니다: World({robot.pose_x:.2f}, {robot.pose_y:.2f})")
            return None
        
        if not (0 <= end_gx < self.width and 0 <= end_gy < self.height):
            logger.error(f"목적지가 맵 범위를 벗어났습니다: World({goal_x:.2f}, {goal_y:.2f})")
            return None

        # 워커블 체크 및 주변 보정 (Soft Start)
        if not self.grid.walkable(start_gx, start_gy):
            logger.warning(f"시작 위치 World({robot.pose_x:.2f}, {robot.pose_y:.2f}) -> Grid({start_gx}, {start_gy})가 장애물 영역입니다. 주변 탐색 시작...")
            
            found_alt = False
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    nx, ny = start_gx + dx, start_gy + dy
                    if 0 <= nx < self.width and 0 <= ny < self.height and self.grid.walkable(nx, ny):
                        logger.info(f"시작점 보정 완료: Grid({start_gx}, {start_gy}) -> Grid({nx}, {ny})")
                        start_gx, start_gy = nx, ny
                        found_alt = True
                        break
                if found_alt: break
            
            if not found_alt:
                # 진단 로그 추가
                is_original_wall = self.binary_map[start_gy][start_gx] == 0
                is_inflation = not is_original_wall and self.static_matrix[start_gy][start_gx] == 0
                
                logger.error(f"보정 실패: 주변에도 이동 가능한 영역이 없습니다. (원인: {'벽' if is_original_wall else '인플레이션'})")
                return None
            # return None # 필요시 주석 해제하여 엄격하게 차단

        try:
            start_node = self.grid.node(start_gx, start_gy)
            end_node = self.grid.node(end_gx, end_gy)
            
            # 경로 계산
            path, runs = self.finder.find_path(start_node, end_node, self.grid)
            
            # 그리드 상태 초기화 (다음 계산을 위해 필요)
            self.grid.cleanup()

            if not path or len(path) == 0:
                logger.warning(f"경로를 찾을 수 없습니다: World({robot.pose_x:.2f}, {robot.pose_y:.2f}) -> World({goal_x:.2f}, {goal_y:.2f})")
                return None

            logger.info(f"경로 계산 완료 (단계: {runs}, 길이: {len(path)})")

            return [
                {"x": x, "y": y} 
                for x, y in [self.grid_to_world(n.x, n.y) for n in path]
            ]
        except Exception as e:
            logger.error(f"경로 계획 중 예외 발생: {e}")
            return None
