# from fastapi import APIRouter, Request
# from fastapi.responses import HTMLResponse
# from fastapi.templating import Jinja2Templates

# # FastAPI 라우터 생성
# router = APIRouter(
#     tags=["Web UI"],
#     include_in_schema=False # API 문서에 이 라우트들을 포함하지 않음
# )

# # Jinja2 템플릿 설정
# templates = Jinja2Templates(directory="main_server/web/templates")


# @router.get("/web/admin", response_class=HTMLResponse)
# async def serve_admin_dashboard(request: Request):
#     """
#     관리자용 웹 대시보드 페이지를 렌더링합니다.
#     """
#     return templates.TemplateResponse("admin_dashboard.html", {"request": request})


# @router.get("/web/employee", response_class=HTMLResponse)
# async def serve_employee_app(request: Request):
#     """
#     직원용 작업 요청 웹 앱 페이지를 렌더링합니다.
#     """
#     return templates.TemplateResponse("employee_app.html", {"request": request})

from fastapi import APIRouter, Request, Form
from fastapi.responses import HTMLResponse, RedirectResponse
from fastapi.templating import Jinja2Templates

router = APIRouter(
    tags=["Web UI"],
    include_in_schema=False
)

templates = Jinja2Templates(directory="main_server/web/templates")

# 1. http://127.0.0.1:8000/web 접속 시 로그인 페이지로
@router.get("/web", response_class=HTMLResponse)
async def login_page(request: Request):
    return templates.TemplateResponse("login.html", {"request": request})

# 2. 로그인 처리 (아이디/비번 체크)
@router.post("/login")
async def login(request: Request, username: str = Form(...), password: str = Form(...)):
    if username == "admin" and password == "1234":
        # 관리자용 대시보드로 이동
        return RedirectResponse(url="/web/admin", status_code=303)
    elif username == "staff" and password == "1234":
        # 직원용 앱으로 이동
        return RedirectResponse(url="/web/employee", status_code=303)
    else:
        # 실패 시 에러 메시지와 함께 다시 로그인창으로
        return templates.TemplateResponse("login.html", {
            "request": request, 
            "error": "아이디 또는 비밀번호가 틀렸습니다."
        })

# 3. 기존 관리자 페이지 (admin 전용)
@router.get("/web/admin", response_class=HTMLResponse)
async def serve_admin_dashboard(request: Request):
    return templates.TemplateResponse("admin_dashboard.html", {"request": request})

# 4. 직원용 페이지 (staff 전용)
@router.get("/web/employee", response_class=HTMLResponse)
async def serve_employee_app(request: Request):
    return templates.TemplateResponse("employee_app.html", {"request": request})