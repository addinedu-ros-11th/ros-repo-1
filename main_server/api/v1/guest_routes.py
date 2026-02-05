from fastapi import APIRouter, status

router = APIRouter(
    prefix="/api/v1/guest",
    tags=["Guest"],
    responses={404: {"description": "Not found"}},
)

@router.post("/qr/verify", status_code=status.HTTP_200_OK)
async def verify_guest_qr_code(qr_code: str):
    """
    Verifies a guest's QR code and, if valid, requests the fleet manager
    to dispatch a robot for guidance.
    """
    # TODO:
    # 1. Decode and validate the QR code content.
    # 2. Check against a temporary guest list or database.
    # 3. If valid, create a guidance task via the TaskManager.
    return {"status": "verified", "message": "A robot will guide you shortly."}
