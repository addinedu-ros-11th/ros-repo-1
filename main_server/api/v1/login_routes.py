from fastapi import APIRouter, HTTPException, status, Form
from ...infrastructure.database.repositories.mysql_user_repository import UserRepository

router = APIRouter(
    prefix="/api/v1",
    tags=["Auth"]
)

user_repo = UserRepository()

@router.post("/login")
async def login_api(username: str = Form(...), password: str = Form(...)):
    # 1. DB에서 사용자 조회
    user = await user_repo.get_user_by_username(username)
    
    # 2. 아이디/비밀번호 검증
    if not user or user.password_hash != password:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="로그인 정보가 올바르지 않습니다."
        )
    
    # 3. 성공 시 권한 정보 등을 담아 JSON으로 반환
    return {
        "status": "success",
        "role": user.role,  # "ADMIN" 또는 "STAFF"
        "name": user.name
    }