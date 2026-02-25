from fastapi import APIRouter, HTTPException, status, Form, Response # Response 추가
from urllib.parse import quote
from ...infrastructure.database.repositories.mysql_user_repository import MySQLUserRepository

router = APIRouter(
    prefix="/api/v1",
    tags=["Auth"]
)

user_repo = MySQLUserRepository()

@router.post("/login")
async def login_api(response: Response, username: str = Form(...), password: str = Form(...)):
    user = await user_repo.get_user_by_username(username)
    
    if not user or user.password_hash != password:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="로그인 정보가 올바르지 않습니다."
        )
    
    # [핵심 수정] 로그인 성공 시 브라우저에 쿠키를 심음
    # 1. 계정 정보 (user_id라는 키로 저장)
    response.set_cookie(
        key="user_id",
        value=user.account,
        httponly=False,
        max_age=3600,
        path="/"
    )
    # 2. 권한 정보
    response.set_cookie(
        key="user_role", 
        # user.role이 Enum이므로 .value를 붙여서 "ADMIN" 문자열만 뽑아냅니다.
        value=user.role.value if hasattr(user.role, 'value') else str(user.role), 
        httponly=False,  # 자바스크립트가 쿠키를 읽을 수 있도록 허용
        max_age=3600,
        path="/"         # 모든 경로에서 쿠키가 유효하도록 설정
    )
    # 3. 이름 정보 (사용자님이 말씀하신 'name' 키로 통일)
    # quote를 사용해야 한글 이름이 깨지지 않고 프론트로 전달
    response.set_cookie(
        key="name",
        value=quote(user.name) if user.name else "",
        httponly=False, 
        max_age=3600,
        path="/"
    )

    return {
        "status": "success",
        "user_id": user.account,
        "role": user.role,
        "name": user.name
    }

@router.post("/logout")
async def logout(response: Response):
    # 쿠키를 삭제합니다. (만료 시간을 0으로 설정하여 즉시 삭제)
    response.delete_cookie(key="user_id", path="/")
    response.delete_cookie(key="user_role", path="/")
    return {"status": "success", "message": "로그아웃 되었습니다."}