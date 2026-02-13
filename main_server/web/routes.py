from fastapi import APIRouter, Request, Form
from fastapi.responses import HTMLResponse, RedirectResponse
from fastapi.templating import Jinja2Templates
# 담당자님의 요청대로 api/v1에 만든 로직을 호출하거나, 직접 리포지토리를 사용합니다.
from ..infrastructure.database.repositories.mysql_user_repository import UserRepository

router = APIRouter(
    tags=["Web UI"],
    include_in_schema=False
)

templates = Jinja2Templates(directory="main_server/web/templates")
user_repo = UserRepository()

# 1. 로그인 페이지 렌더링
@router.get("/web", response_class=HTMLResponse)
async def login_page(request: Request):
    return templates.TemplateResponse("login.html", {"request": request})

# 2. 로그인 처리 (이 부분이 추가되어야 버튼이 작동합니다!)
@router.post("/login")
async def login(request: Request, username: str = Form(...), password: str = Form(...)):
    # DB에서 사용자 정보 조회
    user = await user_repo.get_user_by_username(username)

    # 검증 (단순 비교 버전)
    if user and user.password_hash == password:
        if user.role == "ADMIN":
            return RedirectResponse(url="/web/admin", status_code=303)
        elif user.role == "STAFF":
            return RedirectResponse(url="/web/employee", status_code=303)

    # 실패 시 에러 메시지와 함께 다시 로그인창으로
    return templates.TemplateResponse("login.html", {
        "request": request, 
        "error": "아이디 또는 비밀번호가 틀렸습니다."
    })

# 3. 관리자 대시보드
@router.get("/web/admin", response_class=HTMLResponse)
async def serve_admin_dashboard(request: Request):
    return templates.TemplateResponse("admin_dashboard.html", {"request": request})

# 4. 직원 전용 앱
@router.get("/web/employee", response_class=HTMLResponse)
async def serve_employee_app(request: Request):
    return templates.TemplateResponse("employee_app.html", {"request": request})