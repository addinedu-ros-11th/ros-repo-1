"""
AI Server GUI Log Viewer
서버 시작/중지 + 실시간 로그 표시 GUI
tkinter 기반 — 추가 패키지 불필요
"""

import os
import sys
import json
import signal
import socket
import subprocess
import threading
import tkinter as tk
from tkinter import scrolledtext, font as tkfont
from datetime import datetime
from pathlib import Path
from urllib.request import urlopen
from urllib.error import URLError

try:
    from PIL import Image, ImageTk

    HAS_PIL = True
except ImportError:
    HAS_PIL = False

# ── 경로 설정 ──────────────────────────────────────────────
PROJECT_ROOT = Path(__file__).resolve().parent.parent
AI_SERVER_MODULE = "ai_server.server"
PYTHON_EXE = sys.executable
PREVIEW_FRAME_PATTERN = "/tmp/ai_server_robot_{}_frame.jpg"  # {} → robot index
ROBOTS_INFO_PATH = Path("/tmp/ai_server_robots.json")
MAX_ROBOTS = 2
VIDEO_UPDATE_INTERVAL_MS = 100  # 10fps
TEST_MODE_FLAG_PATH = Path("/tmp/ai_server_test_mode.flag")
TEST_MODE_RESULT_PATH = Path("/tmp/ai_server_test_results.json")
TEST_RESULT_CHECK_INTERVAL_MS = 500  # 0.5초마다 결과 확인


# ── 환경변수 헬퍼 (.env 값에 따옴표가 섞여있을 수 있으므로 strip) ──
def _env(key: str, default: str) -> str:
    return os.getenv(key, default).strip().strip("'\"")


# ── 연결 대상 설정 ─────────────────────────────────────────
# Main Server 는 DB 와 같은 머신(192.168.1.1)에서 실행
_MAIN_SERVER_HOST = _env("MAIN_SERVER_HOST", _env("DB_HOST", "192.168.1.1"))

CONNECTION_TARGETS = {
    "Main Server": {
        "host": _MAIN_SERVER_HOST,
        "port": int(_env("SERVER_PORT", "8000")),
        "proto": "HTTP",
        "health": "/",
    },
    "ROS Bridge": {
        "host": _env("ROS_BRIDGE_HOST", "localhost"),
        "port": int(_env("ROS_BRIDGE_PORT", "9090")),
        "proto": "WS",
    },
    "MySQL DB": {
        "host": _env("DB_HOST", "192.168.1.1"),
        "port": int(_env("DB_PORT", "3306")),
        "proto": "TCP",
    },
    "LLM gRPC": {
        "host": "localhost",
        "port": int(_env("LLM_GRPC_PORT", "50051")),
        "proto": "TCP",
    },
    "Vision gRPC": {
        "host": "localhost",
        "port": int(_env("VISION_GRPC_PORT", "50052")),
        "proto": "TCP",
    },
}

CONNECTION_CHECK_INTERVAL_MS = 5000  # 5초마다 체크

# ── 색상/스타일 ────────────────────────────────────────────
COLORS = {
    "bg": "#1e1e2e",
    "fg": "#cdd6f4",
    "accent": "#89b4fa",
    "green": "#a6e3a1",
    "red": "#f38ba8",
    "yellow": "#f9e2af",
    "orange": "#fab387",
    "surface": "#313244",
    "overlay": "#45475a",
    "subtext": "#a6adc8",
}

LOG_COLORS = {
    "DEBUG": "#89b4fa",
    "INFO": "#a6e3a1",
    "WARNING": "#f9e2af",
    "ERROR": "#f38ba8",
    "CRITICAL": "#f38ba8",
    "DETECTION": "#f5c2e7",  # 핑크 — 테스트 모드 감지 결과
}

# ── gRPC 수신 요청 감지 패턴 ───────────────────────────────
# 서버 로그에서 메인서버 → AI서버 요청을 감지하는 키워드 매핑
GRPC_REQUEST_PATTERNS = {
    "자연어 프롬프트 해석 요청": {
        "rpc": "ParseNaturalLanguage",
        "icon": "💬",
        "service": "LLM",
    },
    "객체 인식 요청": {"rpc": "DetectObjects", "icon": "📦", "service": "Vision"},
    "얼굴 인식 요청": {"rpc": "RecognizeFaces", "icon": "👤", "service": "Vision"},
    "복수 객체 인식 요청": {
        "rpc": "DetectMultipleObjects",
        "icon": "📦",
        "service": "Vision",
    },
    "추론 상태 업데이트": {
        "rpc": "UpdateInferenceState",
        "icon": "⚡",
        "service": "Vision",
    },
    "비전 결과 스트리밍 시작": {
        "rpc": "StreamVisionResults",
        "icon": "📡",
        "service": "Vision",
    },
    "얼굴 인식: ": {
        "rpc": "FaceRecognition",
        "icon": "👤",
        "service": "Vision",
        "is_response": True,
    },
    "장애물 감지: ": {
        "rpc": "ObjectDetection",
        "icon": "🔶",
        "service": "Vision",
        "is_response": True,
    },
    "얼굴 인식 전송: ": {
        "rpc": "FaceRecognition → Main",
        "icon": "📤",
        "service": "Vision",
        "is_send": True,
    },
    "장애물 감지 전송: ": {
        "rpc": "ObjectDetection → Main",
        "icon": "📤",
        "service": "Vision",
        "is_send": True,
    },
    "자연어 해석 완료": {
        "rpc": "ParseNaturalLanguage",
        "icon": "✅",
        "service": "LLM",
        "is_response": True,
    },
    "LLM 응답 전송": {
        "rpc": "LLM → Main Server",
        "icon": "📤",
        "service": "LLM",
        "is_response": True,
        "is_send": True,
    },
    "자연어 해석 중 오류": {
        "rpc": "ParseNaturalLanguage",
        "icon": "❌",
        "service": "LLM",
        "is_response": True,
        "is_error": True,
    },
}


class AIServerGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.process: subprocess.Popen | None = None
        self.log_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        # 비디오 미리보기 상태
        self._video_visible = False
        self._video_update_id = None
        self._video_photo = None  # PhotoImage 참조 유지

        # 테스트 모드 상태
        self._test_mode_active = False
        self._test_result_update_id = None
        self._last_test_result_ts = 0.0

        self._setup_window()
        self._build_header()
        self._build_connection_panel()
        self._build_video_panel()  # 비디오 패널 (초기 숨김)
        self._build_log_area()
        self._build_status_bar()

        # 연결 상태 주기적 체크 시작
        self._check_connections()

        # 종료 시 정리
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._append_log("AI Server GUI 시작됨", "INFO")
        self._append_log(f"Python: {PYTHON_EXE}", "DEBUG")
        self._append_log(f"프로젝트 루트: {PROJECT_ROOT}", "DEBUG")

    # ── 윈도우 설정 ────────────────────────────────────────
    def _setup_window(self):
        self.root.title("AI Server — Log Viewer")
        self.root.geometry("1280x720")
        self.root.minsize(900, 540)
        self.root.configure(bg=COLORS["bg"])

        # 아이콘 설정 시도 (실패해도 무시)
        try:
            self.root.iconname("AI Server")
        except Exception:
            pass

    # ── 헤더 (상태 표시 + 버튼) ────────────────────────────
    def _build_header(self):
        header = tk.Frame(self.root, bg=COLORS["surface"], pady=8, padx=12)
        header.pack(fill=tk.X)

        # 타이틀
        title_font = tkfont.Font(family="Helvetica", size=14, weight="bold")
        tk.Label(
            header,
            text="🤖 AI Server",
            font=title_font,
            bg=COLORS["surface"],
            fg=COLORS["accent"],
        ).pack(side=tk.LEFT)

        # 상태 표시
        self.status_var = tk.StringVar(value="● STOPPED")
        self.status_label = tk.Label(
            header,
            textvariable=self.status_var,
            font=("Helvetica", 11, "bold"),
            bg=COLORS["surface"],
            fg=COLORS["red"],
            padx=16,
        )
        self.status_label.pack(side=tk.LEFT)

        # 버튼 프레임
        btn_frame = tk.Frame(header, bg=COLORS["surface"])
        btn_frame.pack(side=tk.RIGHT)

        btn_style = dict(
            font=("Helvetica", 10, "bold"),
            relief=tk.FLAT,
            cursor="hand2",
            padx=14,
            pady=4,
            bd=0,
        )

        self.btn_start = tk.Button(
            btn_frame,
            text="▶  Start",
            bg=COLORS["green"],
            fg=COLORS["bg"],
            command=self._start_server,
            **btn_style,
        )
        self.btn_start.pack(side=tk.LEFT, padx=4)

        self.btn_stop = tk.Button(
            btn_frame,
            text="■  Stop",
            bg=COLORS["red"],
            fg=COLORS["bg"],
            command=self._stop_server,
            state=tk.DISABLED,
            **btn_style,
        )
        self.btn_stop.pack(side=tk.LEFT, padx=4)

        self.btn_restart = tk.Button(
            btn_frame,
            text="↻  Restart",
            bg=COLORS["orange"],
            fg=COLORS["bg"],
            command=self._restart_server,
            state=tk.DISABLED,
            **btn_style,
        )
        self.btn_restart.pack(side=tk.LEFT, padx=4)

        self.btn_clear = tk.Button(
            btn_frame,
            text="🗑  Clear",
            bg=COLORS["overlay"],
            fg=COLORS["fg"],
            command=self._clear_logs,
            **btn_style,
        )
        self.btn_clear.pack(side=tk.LEFT, padx=4)

        self.btn_video = tk.Button(
            btn_frame,
            text="📹  Video",
            bg=COLORS["accent"],
            fg=COLORS["bg"],
            command=self._toggle_video,
            **btn_style,
        )
        self.btn_video.pack(side=tk.LEFT, padx=4)

        self.btn_test_mode = tk.Button(
            btn_frame,
            text="🧪  Test",
            bg=COLORS["overlay"],
            fg=COLORS["fg"],
            command=self._toggle_test_mode,
            **btn_style,
        )
        self.btn_test_mode.pack(side=tk.LEFT, padx=4)

    # ── 연결 상태 패널 ────────────────────────────────────
    def _build_connection_panel(self):
        self._conn_panel = tk.Frame(self.root, bg=COLORS["surface"], pady=6, padx=12)
        self._conn_panel.pack(fill=tk.X)

        # 제목
        tk.Label(
            self._conn_panel,
            text="연결 상태",
            font=("Helvetica", 10, "bold"),
            bg=COLORS["surface"],
            fg=COLORS["accent"],
        ).pack(side=tk.LEFT, padx=(0, 16))

        self._conn_indicators: dict[str, dict] = {}

        for name, cfg in CONNECTION_TARGETS.items():
            frame = tk.Frame(self._conn_panel, bg=COLORS["surface"])
            frame.pack(side=tk.LEFT, padx=8)

            dot_var = tk.StringVar(value="●")
            dot_label = tk.Label(
                frame,
                textvariable=dot_var,
                font=("Helvetica", 12, "bold"),
                bg=COLORS["surface"],
                fg=COLORS["overlay"],  # 초기: 회색 (unknown)
            )
            dot_label.pack(side=tk.LEFT)

            port_str = f":{cfg['port']}"
            text_label = tk.Label(
                frame,
                text=f"{name} {port_str}",
                font=("Helvetica", 9),
                bg=COLORS["surface"],
                fg=COLORS["subtext"],
            )
            text_label.pack(side=tk.LEFT, padx=(2, 0))

            status_var = tk.StringVar(value="확인 중...")
            status_label = tk.Label(
                frame,
                textvariable=status_var,
                font=("Helvetica", 8),
                bg=COLORS["surface"],
                fg=COLORS["subtext"],
            )
            status_label.pack(side=tk.LEFT, padx=(4, 0))

            self._conn_indicators[name] = {
                "dot_label": dot_label,
                "status_var": status_var,
                "status_label": status_label,
                "cfg": cfg,
            }

    # ── 비디오 미리보기 패널 (듀얼 로봇 지원) ──────────────
    def _build_video_panel(self):
        """UDP 영상 실시간 미리보기 패널 — 2대 로봇 동시 표시 (초기 숨김)"""
        self.video_frame = tk.Frame(self.root, bg=COLORS["bg"])
        # 초기에는 pack하지 않음 (숨김 상태)

        # 비디오 헤더
        video_header = tk.Frame(self.video_frame, bg=COLORS["surface"], pady=4, padx=8)
        video_header.pack(fill=tk.X)

        tk.Label(
            video_header,
            text="📹 로봇 카메라 (UDP 실시간)",
            font=("Helvetica", 10, "bold"),
            bg=COLORS["surface"],
            fg=COLORS["accent"],
        ).pack(side=tk.LEFT)

        self._video_fps_var = tk.StringVar(value="대기 중...")
        tk.Label(
            video_header,
            textvariable=self._video_fps_var,
            font=("Helvetica", 9),
            bg=COLORS["surface"],
            fg=COLORS["yellow"],
        ).pack(side=tk.RIGHT)

        # 로봇 2대 비디오 표시 영역 (좌우 배치)
        video_container = tk.Frame(self.video_frame, bg=COLORS["bg"], padx=4, pady=4)
        video_container.pack(fill=tk.BOTH, expand=True)
        video_container.columnconfigure(0, weight=1)
        video_container.columnconfigure(1, weight=1)
        video_container.rowconfigure(1, weight=1)

        self._robot_video_labels: dict = {}
        self._robot_video_photos: dict = {}
        self._robot_video_fps: dict = {}
        self._robot_video_status_vars: dict = {}

        for i in range(MAX_ROBOTS):
            # 로봇별 헤더
            robot_header = tk.Frame(
                video_container, bg=COLORS["overlay"], pady=2, padx=6
            )
            robot_header.grid(row=0, column=i, sticky="ew", padx=2)

            status_var = tk.StringVar(value=f"Robot #{i + 1} — 연결 대기")
            tk.Label(
                robot_header,
                textvariable=status_var,
                font=("Helvetica", 9, "bold"),
                bg=COLORS["overlay"],
                fg=COLORS["fg"],
            ).pack(side=tk.LEFT)

            fps_var = tk.StringVar(value="")
            tk.Label(
                robot_header,
                textvariable=fps_var,
                font=("Helvetica", 8),
                bg=COLORS["overlay"],
                fg=COLORS["yellow"],
            ).pack(side=tk.RIGHT)

            # 비디오 라벨
            label = tk.Label(
                video_container,
                bg="#000000",
                text=(
                    "영상 수신 대기 중..."
                    if HAS_PIL
                    else "⚠ Pillow 패키지 필요 (pip install Pillow)"
                ),
                fg=COLORS["subtext"],
                font=("Helvetica", 10),
                anchor=tk.CENTER,
            )
            label.grid(row=1, column=i, sticky="nsew", padx=2, pady=2)

            self._robot_video_labels[i] = label
            self._robot_video_photos[i] = None
            self._robot_video_fps[i] = {
                "count": 0,
                "last_time": 0.0,
                "var": fps_var,
            }
            self._robot_video_status_vars[i] = status_var

    def _toggle_video(self):
        """비디오 미리보기 패널 표시/숨김 토글"""
        if self._video_visible:
            # 숨기기
            self.video_frame.pack_forget()
            self._video_visible = False
            self._cancel_video_update()
            self.btn_video.configure(bg=COLORS["accent"])
            self._append_log("비디오 미리보기 OFF", "INFO")
        else:
            # 표시 — 연결 패널 아래, 로그 영역 위에 삽입
            self.video_frame.pack(
                fill=tk.BOTH, after=self._conn_panel, before=self.paned
            )
            self._video_visible = True
            # 로봇별 FPS 카운터 초기화
            for fps_info in self._robot_video_fps.values():
                fps_info["count"] = 0
                fps_info["last_time"] = 0.0
            self._schedule_video_update()
            self.btn_video.configure(bg=COLORS["green"])
            self._append_log("비디오 미리보기 ON (2대 동시 표시)", "INFO")

    def _schedule_video_update(self):
        """비디오 프레임 업데이트 스케줄"""
        self._video_update_id = self.root.after(
            VIDEO_UPDATE_INTERVAL_MS, self._update_video_frame
        )

    def _cancel_video_update(self):
        """비디오 프레임 업데이트 취소"""
        if self._video_update_id is not None:
            self.root.after_cancel(self._video_update_id)
            self._video_update_id = None

    def _update_video_frame(self):
        """주기적으로 각 로봇의 최신 프레임을 읽어 비디오 라벨에 표시"""
        if not self._video_visible:
            return

        try:
            # 로봇 연결 정보 읽기
            connected_count = 0
            if ROBOTS_INFO_PATH.exists():
                try:
                    with open(str(ROBOTS_INFO_PATH), "r") as f:
                        info = json.load(f)
                    robots = info.get("robots", [])
                    connected_count = len(robots)
                    for r in robots:
                        idx = r["index"]
                        ip = r["ip"]
                        robot_id = r.get("robot_id", ip)
                        if idx in self._robot_video_status_vars:
                            self._robot_video_status_vars[idx].set(f"{robot_id} — {ip}")
                except Exception:
                    pass

            if connected_count > 0:
                self._video_fps_var.set(f"{connected_count}대 연결")

            # 각 로봇의 프레임 업데이트
            for i in range(MAX_ROBOTS):
                preview_path = Path(PREVIEW_FRAME_PATTERN.format(i))
                label = self._robot_video_labels[i]

                if HAS_PIL and preview_path.exists():
                    try:
                        img = Image.open(str(preview_path))

                        # 패널 너비에 맞춰 리사이즈 (비율 유지)
                        panel_w = label.winfo_width()
                        if panel_w < 50:
                            panel_w = 400
                        target_h = int(panel_w * img.height / img.width)
                        if target_h > 480:
                            target_h = 480
                            panel_w = int(target_h * img.width / img.height)
                        img = img.resize((panel_w, target_h), Image.LANCZOS)

                        photo = ImageTk.PhotoImage(img)
                        label.configure(image=photo, text="")
                        self._robot_video_photos[i] = photo  # GC 방지

                        # FPS 계산
                        fps_info = self._robot_video_fps[i]
                        fps_info["count"] += 1
                        now = datetime.now().timestamp()
                        if fps_info["last_time"] == 0:
                            fps_info["last_time"] = now
                        elif (now - fps_info["last_time"]) >= 1.0:
                            fps = fps_info["count"] / (now - fps_info["last_time"])
                            fps_info["var"].set(f"{fps:.1f} FPS")
                            fps_info["count"] = 0
                            fps_info["last_time"] = now
                    except Exception:
                        pass

        except Exception:
            pass  # 파일 읽기/디코딩 일시 실패 무시

        # 다음 업데이트 예약
        self._schedule_video_update()

    # ── 테스트 모드 ──────────────────────────────────────
    def _toggle_test_mode(self):
        """테스트 모드 ON/OFF 토글 — 플래그 파일로 서버 프로세스와 통신"""
        if self._test_mode_active:
            # OFF
            self._test_mode_active = False
            try:
                TEST_MODE_FLAG_PATH.unlink(missing_ok=True)
                TEST_MODE_RESULT_PATH.unlink(missing_ok=True)
            except Exception:
                pass
            self._cancel_test_result_update()
            self.btn_test_mode.configure(bg=COLORS["overlay"], fg=COLORS["fg"])
            self._append_log("🧪 테스트 모드 OFF", "INFO")
        else:
            # ON — 비디오 미리보기도 자동으로 켜기
            self._test_mode_active = True
            try:
                TEST_MODE_FLAG_PATH.touch()
            except Exception as e:
                self._append_log(f"테스트 모드 활성화 실패: {e}", "ERROR")
                self._test_mode_active = False
                return

            if not self._video_visible:
                self._toggle_video()

            self._last_test_result_ts = 0.0
            self._schedule_test_result_update()
            self.btn_test_mode.configure(bg=COLORS["yellow"], fg=COLORS["bg"])
            self._append_log(
                "🧪 테스트 모드 ON — OBSTACLE + EMPLOYEE 추론 활성화", "INFO"
            )

    def _schedule_test_result_update(self):
        """테스트 결과 주기적 읽기 예약"""
        self._test_result_update_id = self.root.after(
            TEST_RESULT_CHECK_INTERVAL_MS, self._update_test_results
        )

    def _cancel_test_result_update(self):
        """테스트 결과 업데이트 취소"""
        if self._test_result_update_id is not None:
            self.root.after_cancel(self._test_result_update_id)
            self._test_result_update_id = None

    def _update_test_results(self):
        """테스트 모드 결과 JSON 파일을 읽어 로그에 표시"""
        if not self._test_mode_active:
            return

        try:
            if TEST_MODE_RESULT_PATH.exists():
                with open(str(TEST_MODE_RESULT_PATH), "r") as f:
                    data = json.load(f)

                ts = data.get("timestamp", 0)
                if ts > self._last_test_result_ts:
                    self._last_test_result_ts = ts
                    results = data.get("results", [])
                    if results:
                        parts = []
                        for r in results:
                            if r["type"] == "obstacle":
                                parts.append(f"🔶 {r['name']} ({r['confidence']:.0%})")
                            elif r["type"] == "face":
                                if r["person_type"] == "Employee":
                                    parts.append(
                                        f"🟢 직원: {r['employee_id']} ({r['confidence']:.0%})"
                                    )
                                else:
                                    parts.append(
                                        f"🟠 {r['person_type']} ({r['confidence']:.0%})"
                                    )
                        if parts:
                            self._append_log(
                                f"🧪 감지: {' | '.join(parts)}", "DETECTION"
                            )
        except Exception:
            pass

        self._schedule_test_result_update()

    # ── 연결 상태 체크 ────────────────────────────────────
    def _check_connections(self):
        """별도 스레드에서 모든 연결 대상 체크 후 UI 업데이트"""

        def _worker():
            results = {}
            for name, info in self._conn_indicators.items():
                cfg = info["cfg"]
                ok = self._probe_connection(cfg)
                results[name] = ok
            # UI 스레드에서 업데이트
            self.root.after(0, self._update_connection_ui, results)

        threading.Thread(target=_worker, daemon=True).start()
        # 다음 체크 예약
        self.root.after(CONNECTION_CHECK_INTERVAL_MS, self._check_connections)

    def _probe_connection(self, cfg: dict) -> bool:
        """단일 연결 대상의 도달 가능 여부 확인"""
        host = cfg["host"]
        port = cfg["port"]
        proto = cfg.get("proto", "TCP")

        if proto == "HTTP":
            # HTTP health check
            health = cfg.get("health", "/")
            try:
                url = f"http://{host}:{port}{health}"
                resp = urlopen(url, timeout=2)
                return resp.status == 200
            except Exception:
                return False
        else:
            # TCP/WS — 포트 연결 가능 여부
            try:
                with socket.create_connection((host, port), timeout=2):
                    return True
            except Exception:
                return False

    def _update_connection_ui(self, results: dict[str, bool]):
        """체크 결과를 UI에 반영"""
        for name, ok in results.items():
            info = self._conn_indicators[name]
            if ok:
                info["dot_label"].configure(fg=COLORS["green"])
                info["status_var"].set("연결됨")
                info["status_label"].configure(fg=COLORS["green"])
            else:
                info["dot_label"].configure(fg=COLORS["red"])
                info["status_var"].set("끊김")
                info["status_label"].configure(fg=COLORS["red"])

    # ── 로그 + 수신 요청 영역 (좌우 분할) ─────────────────
    def _build_log_area(self):
        # PanedWindow 로 좌우 분할
        self.paned = tk.PanedWindow(
            self.root,
            orient=tk.HORIZONTAL,
            bg=COLORS["overlay"],
            sashwidth=4,
            sashrelief=tk.FLAT,
        )
        self.paned.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        # ── 왼쪽: 서버 로그 ──
        left_frame = tk.Frame(self.paned, bg=COLORS["bg"])
        self.paned.add(left_frame, stretch="always")

        log_header = tk.Frame(left_frame, bg=COLORS["surface"], pady=4, padx=8)
        log_header.pack(fill=tk.X)
        tk.Label(
            log_header,
            text="📋 서버 로그",
            font=("Helvetica", 10, "bold"),
            bg=COLORS["surface"],
            fg=COLORS["accent"],
        ).pack(side=tk.LEFT)

        log_font = tkfont.Font(family="Consolas", size=10)
        self.log_text = scrolledtext.ScrolledText(
            left_frame,
            wrap=tk.WORD,
            font=log_font,
            bg=COLORS["bg"],
            fg=COLORS["fg"],
            insertbackground=COLORS["fg"],
            selectbackground=COLORS["accent"],
            borderwidth=0,
            highlightthickness=0,
            state=tk.DISABLED,
            padx=8,
            pady=8,
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)

        for level, color in LOG_COLORS.items():
            self.log_text.tag_configure(level, foreground=color)
        self.log_text.tag_configure("TIMESTAMP", foreground=COLORS["subtext"])
        self.log_text.tag_configure("SEPARATOR", foreground=COLORS["overlay"])

        # ── 오른쪽: 수신 요청 패널 ──
        right_frame = tk.Frame(self.paned, bg=COLORS["bg"])
        self.paned.add(right_frame, stretch="always")

        req_header = tk.Frame(right_frame, bg=COLORS["surface"], pady=4, padx=8)
        req_header.pack(fill=tk.X)
        tk.Label(
            req_header,
            text="📨 수신 요청 (Main → AI)",
            font=("Helvetica", 10, "bold"),
            bg=COLORS["surface"],
            fg=COLORS["orange"],
        ).pack(side=tk.LEFT)

        self._req_count_var = tk.StringVar(value="0건")
        tk.Label(
            req_header,
            textvariable=self._req_count_var,
            font=("Helvetica", 9, "bold"),
            bg=COLORS["surface"],
            fg=COLORS["yellow"],
        ).pack(side=tk.RIGHT)

        btn_clear_req = tk.Button(
            req_header,
            text="Clear",
            font=("Helvetica", 8, "bold"),
            bg=COLORS["overlay"],
            fg=COLORS["fg"],
            relief=tk.FLAT,
            bd=0,
            padx=8,
            command=self._clear_requests,
        )
        btn_clear_req.pack(side=tk.RIGHT, padx=4)

        req_font = tkfont.Font(family="Consolas", size=10)
        self.req_text = scrolledtext.ScrolledText(
            right_frame,
            wrap=tk.WORD,
            font=req_font,
            bg=COLORS["bg"],
            fg=COLORS["fg"],
            insertbackground=COLORS["fg"],
            selectbackground=COLORS["accent"],
            borderwidth=0,
            highlightthickness=0,
            state=tk.DISABLED,
            padx=8,
            pady=8,
        )
        self.req_text.pack(fill=tk.BOTH, expand=True)

        # 수신 요청 패널 태그
        self.req_text.tag_configure(
            "RPC_NAME", foreground=COLORS["accent"], font=("Consolas", 10, "bold")
        )
        self.req_text.tag_configure("SERVICE", foreground=COLORS["orange"])
        self.req_text.tag_configure("DETAIL", foreground=COLORS["fg"])
        self.req_text.tag_configure("RESPONSE", foreground=COLORS["green"])
        self.req_text.tag_configure("SEND", foreground="#94e2d5")  # teal: AI→Main
        self.req_text.tag_configure("FIELD_KEY", foreground=COLORS["yellow"])
        self.req_text.tag_configure("FIELD_VAL", foreground=COLORS["fg"])
        self.req_text.tag_configure("ERROR_TAG", foreground=COLORS["red"])
        self.req_text.tag_configure("TIME", foreground=COLORS["subtext"])
        self.req_text.tag_configure("DIVIDER", foreground=COLORS["overlay"])

        self._req_count = 0

        # 초기 사시비율: 왼쪽 65%, 오른쪽 35%
        self.root.update_idletasks()
        self.paned.sash_place(0, int(self.root.winfo_width() * 0.62), 0)

    # ── 하단 상태바 ───────────────────────────────────────
    def _build_status_bar(self):
        bar = tk.Frame(self.root, bg=COLORS["surface"], pady=4, padx=12)
        bar.pack(fill=tk.X, side=tk.BOTTOM)

        self.info_var = tk.StringVar(
            value="LLM gRPC :50051  |  Vision gRPC :50052  |  UDP 영상 :54321"
        )
        tk.Label(
            bar,
            textvariable=self.info_var,
            font=("Helvetica", 9),
            bg=COLORS["surface"],
            fg=COLORS["subtext"],
        ).pack(side=tk.LEFT)

        self.pid_var = tk.StringVar(value="PID: —")
        tk.Label(
            bar,
            textvariable=self.pid_var,
            font=("Helvetica", 9),
            bg=COLORS["surface"],
            fg=COLORS["subtext"],
        ).pack(side=tk.RIGHT)

    # ── 로그 추가 ─────────────────────────────────────────
    def _append_log(self, message: str, level: str = "INFO"):
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        tag = level if level in LOG_COLORS else "INFO"

        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{timestamp}] ", "TIMESTAMP")
        self.log_text.insert(tk.END, f"[{level:^8s}] ", tag)
        self.log_text.insert(tk.END, f"{message}\n")
        self.log_text.configure(state=tk.DISABLED)
        self.log_text.see(tk.END)

    def _append_raw(self, line: str):
        """서버 출력을 파싱해서 적절한 로그 레벨로 표시 + 수신 요청 감지"""
        line = line.rstrip()
        if not line:
            return

        level = "INFO"
        for lvl in ("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"):
            if f" - {lvl} - " in line or f"- {lvl} -" in line:
                level = lvl
                break

        # 로그 패널에 표시
        self.log_text.configure(state=tk.NORMAL)
        tag = level if level in LOG_COLORS else "INFO"
        self.log_text.insert(tk.END, line + "\n", tag)
        self.log_text.configure(state=tk.DISABLED)
        self.log_text.see(tk.END)

        # gRPC 수신 요청 감지 → 수신 요청 패널에 표시
        for keyword, info in GRPC_REQUEST_PATTERNS.items():
            if keyword in line:
                self._append_request(line, info)
                break

    # ── 수신 요청 패널에 추가 ─────────────────────────────
    def _append_request(self, raw_line: str, info: dict):
        """감지된 gRPC 요청을 수신 요청 패널에 시각적으로 표시"""
        ts = datetime.now().strftime("%H:%M:%S")
        icon = info["icon"]
        rpc = info["rpc"]
        service = info["service"]
        is_resp = info.get("is_response", False)
        is_send = info.get("is_send", False)
        is_error = info.get("is_error", False)

        # 로그 내용에서 세부사항 추출 (마지막 ' - ' 이후)
        detail = ""
        if " - " in raw_line:
            parts = raw_line.split(" - ", maxsplit=3)
            detail = parts[-1].strip() if len(parts) > 1 else raw_line

        self.req_text.configure(state=tk.NORMAL)

        if is_error:
            # 에러 표시
            self.req_text.insert(tk.END, f"  {ts} ", "TIME")
            self.req_text.insert(tk.END, f"{icon} ", "ERROR_TAG")
            self.req_text.insert(tk.END, f"{rpc} 오류\n", "ERROR_TAG")
            if detail:
                self.req_text.insert(tk.END, f"         {detail}\n", "ERROR_TAG")

        elif is_send:
            # LLM 응답 전송 (AI → Main) — 상세 필드 파싱
            self.req_text.insert(tk.END, f"  {ts} ", "TIME")
            self.req_text.insert(tk.END, f"{icon} ", "SEND")
            self.req_text.insert(tk.END, f"{rpc}\n", "SEND")

            # 상세 필드 파싱: task_type=..., confidence=..., fields={...}
            parsed = self._parse_llm_response(detail)
            if parsed:
                if "task_type" in parsed:
                    self.req_text.insert(tk.END, "         ", "TIME")
                    self.req_text.insert(tk.END, "task_type: ", "FIELD_KEY")
                    self.req_text.insert(tk.END, f"{parsed['task_type']}\n", "RPC_NAME")
                if "confidence" in parsed:
                    self.req_text.insert(tk.END, "         ", "TIME")
                    self.req_text.insert(tk.END, "confidence: ", "FIELD_KEY")
                    conf_val = parsed["confidence"]
                    conf_tag = (
                        "RESPONSE"
                        if float(conf_val) >= 0.7
                        else "ERROR_TAG" if float(conf_val) < 0.3 else "FIELD_VAL"
                    )
                    self.req_text.insert(tk.END, f"{conf_val}\n", conf_tag)
                if "fields" in parsed and parsed["fields"]:
                    self.req_text.insert(tk.END, "         ", "TIME")
                    self.req_text.insert(tk.END, "fields:\n", "FIELD_KEY")
                    for fk, fv in parsed["fields"].items():
                        self.req_text.insert(tk.END, f"           ", "TIME")
                        self.req_text.insert(tk.END, f"{fk}: ", "FIELD_KEY")
                        self.req_text.insert(tk.END, f"{fv}\n", "FIELD_VAL")
            elif detail:
                self.req_text.insert(tk.END, f"         {detail}\n", "DETAIL")

        elif is_resp:
            # 응답 완료 표시
            self.req_text.insert(tk.END, f"  {ts} ", "TIME")
            self.req_text.insert(tk.END, f"{icon} ", "RESPONSE")
            self.req_text.insert(tk.END, f"← {rpc} ", "RESPONSE")
            self.req_text.insert(tk.END, f"완료\n", "RESPONSE")
            if detail:
                self.req_text.insert(tk.END, f"         {detail}\n", "DETAIL")
        else:
            # 새 요청 표시
            self._req_count += 1
            self._req_count_var.set(f"{self._req_count}건")

            self.req_text.insert(tk.END, f"─" * 42 + "\n", "DIVIDER")
            self.req_text.insert(tk.END, f"  {ts} ", "TIME")
            self.req_text.insert(tk.END, f"{icon} ", "RPC_NAME")
            self.req_text.insert(tk.END, f"{rpc}", "RPC_NAME")
            self.req_text.insert(tk.END, f"  [{service}]\n", "SERVICE")
            if detail:
                self.req_text.insert(tk.END, f"         {detail}\n", "DETAIL")

        self.req_text.configure(state=tk.DISABLED)
        self.req_text.see(tk.END)

    def _parse_llm_response(self, detail: str) -> dict | None:
        """
        LLM 응답 전송 로그를 파싱하여 task_type, confidence, fields를 추출.
        예: 'LLM 응답 전송 [req_id=abc]: task_type=SNACK_DELIVERY, confidence=0.95, fields={...}'
        """
        import re

        result = {}
        try:
            # task_type
            m = re.search(r"task_type=(\w+)", detail)
            if m:
                result["task_type"] = m.group(1)
            # confidence
            m = re.search(r"confidence=([\d.]+)", detail)
            if m:
                result["confidence"] = m.group(1)
            # fields={...}
            m = re.search(r"fields=\{(.+)\}", detail)
            if m:
                fields_str = m.group(1)
                fields = {}
                # 'key': 'value' 패턴 파싱
                for pair in re.finditer(
                    r"'(\w+)'\s*:\s*'([^']*)'|'(\w+)'\s*:\s*([\d.]+)", fields_str
                ):
                    key = pair.group(1) or pair.group(3)
                    val = pair.group(2) or pair.group(4)
                    if key and val:
                        fields[key] = val
                result["fields"] = fields
        except Exception:
            return None
        return result if result else None

    def _clear_requests(self):
        """수신 요청 패널 초기화"""
        self.req_text.configure(state=tk.NORMAL)
        self.req_text.delete("1.0", tk.END)
        self.req_text.configure(state=tk.DISABLED)
        self._req_count = 0
        self._req_count_var.set("0건")

    # ── 서버 시작 ─────────────────────────────────────────
    def _start_server(self):
        if self.process and self.process.poll() is None:
            self._append_log("서버가 이미 실행 중입니다", "WARNING")
            return

        self._append_log("=" * 56, "INFO")
        self._append_log("AI Server 시작 중...", "INFO")
        self._append_log("=" * 56, "INFO")

        env = os.environ.copy()
        pythonpath = env.get("PYTHONPATH", "")
        env["PYTHONPATH"] = f"{PROJECT_ROOT}:{pythonpath}"
        # 로그가 즉시 출력되도록 버퍼링 비활성화
        env["PYTHONUNBUFFERED"] = "1"

        try:
            self.process = subprocess.Popen(
                [PYTHON_EXE, "-m", AI_SERVER_MODULE],
                cwd=str(PROJECT_ROOT),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=env,
                preexec_fn=os.setsid,
            )
        except Exception as e:
            self._append_log(f"서버 시작 실패: {e}", "ERROR")
            return

        self._stop_event.clear()
        self.log_thread = threading.Thread(target=self._read_output, daemon=True)
        self.log_thread.start()

        self._set_running_state(True)
        self.pid_var.set(f"PID: {self.process.pid}")
        self._append_log(f"서버 프로세스 시작됨 (PID {self.process.pid})", "INFO")

        # 프로세스 종료 감시
        threading.Thread(target=self._watch_process, daemon=True).start()

    # ── 서버 중지 ─────────────────────────────────────────
    def _stop_server(self):
        if not self.process or self.process.poll() is not None:
            self._append_log("실행 중인 서버가 없습니다", "WARNING")
            self._set_running_state(False)
            return

        self._append_log("서버 종료 요청 중...", "WARNING")
        self._stop_event.set()

        try:
            # 프로세스 그룹 전체에 SIGTERM
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            try:
                self.process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                self._append_log("SIGTERM 타임아웃 → SIGKILL 전송", "ERROR")
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                self.process.wait(timeout=5)
        except ProcessLookupError:
            pass
        except Exception as e:
            self._append_log(f"종료 중 오류: {e}", "ERROR")

        self._set_running_state(False)
        self._append_log("서버가 종료되었습니다", "INFO")
        self.pid_var.set("PID: —")

        # 테스트 모드 OFF
        if self._test_mode_active:
            self._test_mode_active = False
            self._cancel_test_result_update()
            try:
                TEST_MODE_FLAG_PATH.unlink(missing_ok=True)
                TEST_MODE_RESULT_PATH.unlink(missing_ok=True)
            except Exception:
                pass
            self.btn_test_mode.configure(bg=COLORS["overlay"], fg=COLORS["fg"])

    # ── 재시작 ────────────────────────────────────────────
    def _restart_server(self):
        self._append_log("서버 재시작 중...", "WARNING")
        self._stop_server()
        self.root.after(1000, self._start_server)

    # ── 출력 읽기 (별도 스레드) ───────────────────────────
    def _read_output(self):
        try:
            for line in iter(self.process.stdout.readline, b""):
                if self._stop_event.is_set():
                    break
                decoded = line.decode("utf-8", errors="replace")
                # UI 스레드에서 로그 추가
                self.root.after(0, self._append_raw, decoded)
        except Exception:
            pass

    # ── 프로세스 종료 감시 ────────────────────────────────
    def _watch_process(self):
        if self.process:
            retcode = self.process.wait()
            if not self._stop_event.is_set():
                self.root.after(
                    0,
                    self._append_log,
                    f"서버가 예기치 않게 종료됨 (exit code: {retcode})",
                    "ERROR",
                )
                self.root.after(0, self._set_running_state, False)
                self.root.after(0, self.pid_var.set, "PID: —")

    # ── UI 상태 전환 ──────────────────────────────────────
    def _set_running_state(self, running: bool):
        if running:
            self.status_var.set("● RUNNING")
            self.status_label.configure(fg=COLORS["green"])
            self.btn_start.configure(state=tk.DISABLED)
            self.btn_stop.configure(state=tk.NORMAL)
            self.btn_restart.configure(state=tk.NORMAL)
        else:
            self.status_var.set("● STOPPED")
            self.status_label.configure(fg=COLORS["red"])
            self.btn_start.configure(state=tk.NORMAL)
            self.btn_stop.configure(state=tk.DISABLED)
            self.btn_restart.configure(state=tk.DISABLED)

    # ── 로그 초기화 ──────────────────────────────────────
    def _clear_logs(self):
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.configure(state=tk.DISABLED)
        self._append_log("로그 초기화됨", "INFO")

    # ── 종료 처리 ─────────────────────────────────────────
    def _on_close(self):
        self._cancel_video_update()
        self._cancel_test_result_update()
        # 테스트 모드 플래그 정리
        try:
            TEST_MODE_FLAG_PATH.unlink(missing_ok=True)
            TEST_MODE_RESULT_PATH.unlink(missing_ok=True)
        except Exception:
            pass
        # 로봇 미리보기 파일 정리
        for i in range(MAX_ROBOTS):
            try:
                Path(PREVIEW_FRAME_PATTERN.format(i)).unlink(missing_ok=True)
            except Exception:
                pass
        try:
            ROBOTS_INFO_PATH.unlink(missing_ok=True)
        except Exception:
            pass
        if self.process and self.process.poll() is None:
            self._stop_server()
        self.root.destroy()


def main():
    root = tk.Tk()
    AIServerGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
