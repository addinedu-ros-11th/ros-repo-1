"""
인코딩 캐시 테스트
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from ai_server.services.vision_service import VisionService

print("=" * 60)
print("인코딩 캐시 성능 테스트")
print("=" * 60)

# 첫 번째 로드 (캐시 사용)
print("\n1. Vision Service 초기화 (캐시에서 로드)...")
start = time.time()
service = VisionService()
service.initialize()
load_time = time.time() - start

print(f"   ✓ 로딩 완료: {load_time:.4f}초")
print(f"   ✓ 직원 수: {len(service._employee_face_db)}명")

# 캐시 파일 정보
cache_path = service.encodings_cache_path
if cache_path.exists():
    size_kb = cache_path.stat().st_size / 1024
    print(f"   ✓ 캐시 파일: {cache_path.name} ({size_kb:.2f} KB)")

print("\n" + "=" * 60)
print("✅ 테스트 완료")
