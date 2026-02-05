from fastapi import APIRouter, Request
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates

# FastAPI 라우터 생성
router = APIRouter(
    tags=["Web UI"],
    include_in_schema=False # API 문서에 이 라우트들을 포함하지 않음
)

# Jinja2 템플릿 설정
templates = Jinja2Templates(directory="main_server/web/templates")


@router.get("/web/admin", response_class=HTMLResponse)
async def serve_admin_dashboard(request: Request):
    """
    관리자용 웹 대시보드 페이지를 렌더링합니다.
    """
    return templates.TemplateResponse("admin_dashboard.html", {"request": request})


@router.get("/web/employee", response_class=HTMLResponse)
async def serve_employee_app(request: Request):
    """
    직원용 작업 요청 웹 앱 페이지를 렌더링합니다.
    """
    return templates.TemplateResponse("employee_app.html", {"request": request})
