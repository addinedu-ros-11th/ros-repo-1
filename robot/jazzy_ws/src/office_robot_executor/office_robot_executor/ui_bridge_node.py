import json
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

os.environ.setdefault("HOME", "/home/pinky")
os.environ.setdefault("TMPDIR", "/tmp")

for candidate in (
    "/home/pinky/.local/lib/python3.12/site-packages",
    "/home/pinky/pinky_pro/install/pinky_emotion/lib/python3.12/site-packages",
    "/home/pinky/pinky_lcd/example",
):
    if os.path.isdir(candidate) and candidate not in sys.path:
        sys.path.append(candidate)

PIL_IMPORT_ERROR: Optional[str] = None
LCD_IMPORT_ERROR: Optional[str] = None
LED_IMPORT_ERROR: Optional[str] = None
SETLED_IMPORT_ERROR: Optional[str] = None

try:  # pragma: no cover - runtime environment dependent
    from PIL import Image, ImageDraw, ImageFont
except Exception as exc:  # pragma: no cover - runtime environment dependent
    Image = None
    ImageDraw = None
    ImageFont = None
    PIL_IMPORT_ERROR = repr(exc)

try:  # pragma: no cover - runtime environment dependent
    from pinky_lcd import LCD as PinkyLCD
except Exception as exc:  # pragma: no cover - runtime environment dependent
    LCD_IMPORT_ERROR = repr(exc)
    try:
        from pinky_emotion.pinky_lcd import LCD as PinkyLCD
        LCD_IMPORT_ERROR = None
    except Exception as fallback_exc:  # pragma: no cover - runtime environment dependent
        PinkyLCD = None
        LCD_IMPORT_ERROR = f"primary={LCD_IMPORT_ERROR}; fallback={repr(fallback_exc)}"

try:  # pragma: no cover - runtime environment dependent
    from pinkylib.led import LED as PinkyLED
except Exception as exc:  # pragma: no cover - runtime environment dependent
    PinkyLED = None
    LED_IMPORT_ERROR = repr(exc)

try:  # pragma: no cover - runtime environment dependent
    from pinky_interfaces.srv import SetLed as PinkySetLed
except Exception as exc:  # pragma: no cover - runtime environment dependent
    PinkySetLed = None
    SETLED_IMPORT_ERROR = repr(exc)


COLOR_NAME_MAP = {
    "RED": (255, 0, 0),
    "GREEN": (0, 255, 0),
    "BLUE": (0, 0, 255),
    "YELLOW": (255, 255, 0),
    "ORANGE": (255, 128, 0),
    "WHITE": (255, 255, 255),
    "BLACK": (0, 0, 0),
    "PURPLE": (128, 0, 255),
    "PINK": (255, 105, 180),
    "CYAN": (0, 255, 255),
}


class OfficeRobotUiBridge(Node):
    def __init__(self) -> None:
        super().__init__("office_robot_ui_bridge")

        self.declare_parameter("robot_name", "robot01")
        self.declare_parameter("display_topic", "display")
        self.declare_parameter("led_topic", "led_command")
        self.declare_parameter("lcd_enabled", True)
        self.declare_parameter("led_enabled", True)
        self.declare_parameter("led_service_enabled", True)
        self.declare_parameter("led_service_name", "/set_led")
        self.declare_parameter("led_service_wait_sec", 0.15)
        self.declare_parameter("lcd_font_path", "")
        self.declare_parameter(
            "lcd_font_candidates",
            ",".join(
                [
                    "/home/pinky/pinky_lcd/example/MaruBuri-Bold.ttf",
                    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
                    "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
                    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                ]
            ),
        )
        self.declare_parameter("lcd_font_size", 28)
        self.declare_parameter("lcd_backlight", 100)
        self.declare_parameter("lcd_width", 320)
        self.declare_parameter("lcd_height", 240)
        self.declare_parameter("lcd_wrap_width_px", 210)
        self.declare_parameter("lcd_line_spacing_px", 10)
        self.declare_parameter("lcd_text_color", "255,255,255")
        self.declare_parameter("lcd_accent_color", "100,180,255")
        self.declare_parameter("lcd_bg_color", "0,0,0")
        self.declare_parameter("lcd_clear_on_shutdown", False)
        self.declare_parameter("led_clear_on_shutdown", True)

        self.robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        self.display_topic = self.get_parameter("display_topic").get_parameter_value().string_value
        self.led_topic = self.get_parameter("led_topic").get_parameter_value().string_value
        self.lcd_enabled = self.get_parameter("lcd_enabled").get_parameter_value().bool_value
        self.led_enabled = self.get_parameter("led_enabled").get_parameter_value().bool_value
        self.led_service_enabled = (
            self.get_parameter("led_service_enabled").get_parameter_value().bool_value
        )
        self.led_service_name = (
            self.get_parameter("led_service_name").get_parameter_value().string_value
        )
        self.led_service_wait_sec = max(
            0.0,
            float(self.get_parameter("led_service_wait_sec").get_parameter_value().double_value),
        )
        self.lcd_font_path = self.get_parameter("lcd_font_path").get_parameter_value().string_value
        self.lcd_font_candidates = [
            token.strip()
            for token in self.get_parameter("lcd_font_candidates")
            .get_parameter_value()
            .string_value.split(",")
            if token.strip()
        ]
        self.lcd_font_size = max(
            12, self.get_parameter("lcd_font_size").get_parameter_value().integer_value
        )
        self.lcd_backlight = max(
            0, min(100, self.get_parameter("lcd_backlight").get_parameter_value().integer_value)
        )
        self.lcd_width = max(
            100, self.get_parameter("lcd_width").get_parameter_value().integer_value
        )
        self.lcd_height = max(
            100, self.get_parameter("lcd_height").get_parameter_value().integer_value
        )
        self.lcd_wrap_width_px = max(
            100, self.get_parameter("lcd_wrap_width_px").get_parameter_value().integer_value
        )
        self.lcd_line_spacing_px = max(
            0, self.get_parameter("lcd_line_spacing_px").get_parameter_value().integer_value
        )
        self.lcd_text_color = self._parse_rgb_string(
            self.get_parameter("lcd_text_color").get_parameter_value().string_value,
            (255, 255, 255),
        )
        self.lcd_accent_color = self._parse_rgb_string(
            self.get_parameter("lcd_accent_color").get_parameter_value().string_value,
            (100, 180, 255),
        )
        self.lcd_bg_color = self._parse_rgb_string(
            self.get_parameter("lcd_bg_color").get_parameter_value().string_value,
            (0, 0, 0),
        )
        self.lcd_clear_on_shutdown = (
            self.get_parameter("lcd_clear_on_shutdown").get_parameter_value().bool_value
        )
        self.led_clear_on_shutdown = (
            self.get_parameter("led_clear_on_shutdown").get_parameter_value().bool_value
        )

        self._lcd = None
        self._led = None
        self._led_service_client = None
        self._led_blink_timer = None
        self._led_blink_color = (255, 255, 255)
        self._led_blink_on = False
        self._warned_led_service_unavailable = False

        self._init_lcd()
        self._init_led()

        self.display_sub = self.create_subscription(
            String, self.display_topic, self._on_display, 10
        )
        self.led_sub = self.create_subscription(String, self.led_topic, self._on_led, 10)

        self.get_logger().info(
            f"UI bridge ready (robot={self.robot_name}, display_topic={self.display_topic}, "
            f"lcd_enabled={self._lcd is not None}, led_topic={self.led_topic}, "
            f"led_enabled={self._led is not None}, "
            f"led_service={self.led_service_name if self._led_service_client else 'disabled'})"
        )

    def _init_lcd(self) -> None:
        if not self.lcd_enabled:
            return
        if PinkyLCD is None:
            self.get_logger().warn(
                f"LCD bridge disabled: LCD module unavailable ({LCD_IMPORT_ERROR})."
            )
            return
        if Image is None or ImageDraw is None or ImageFont is None:
            self.get_logger().warn(
                f"LCD bridge disabled: PIL modules unavailable ({PIL_IMPORT_ERROR})."
            )
            return
        try:
            self._lcd = PinkyLCD()
            if hasattr(self._lcd, "set_backlight"):
                self._lcd.set_backlight(self.lcd_backlight)
        except Exception as exc:
            self._lcd = None
            self.get_logger().error(f"Failed to initialize LCD bridge: {exc}")

    def _init_led(self) -> None:
        if not self.led_enabled:
            return
        if self.led_service_enabled:
            if PinkySetLed is None:
                self.get_logger().warn(
                    f"LED service bridge disabled: pinky_interfaces.srv.SetLed unavailable "
                    f"({SETLED_IMPORT_ERROR})."
                )
            else:
                self._led_service_client = self.create_client(PinkySetLed, self.led_service_name)
        if PinkyLED is None:
            if self._led_service_client is None:
                self.get_logger().warn(
                    f"LED bridge disabled: pinkylib.led.LED unavailable ({LED_IMPORT_ERROR})."
                )
                return
            return
        try:
            self._led = PinkyLED()
        except Exception as exc:
            self._led = None
            if self._led_service_client is None:
                self.get_logger().error(f"Failed to initialize LED bridge: {exc}")
            else:
                self.get_logger().warn(f"Failed to initialize local LED fallback: {exc}")

    def _on_display(self, msg: String) -> None:
        payload = self._decode_payload(msg.data)
        text = str(payload.get("text", "")).strip()
        icon = str(payload.get("icon", "")).strip()
        if not text:
            return
        if self._lcd is None:
            self.get_logger().info(f"Display message received without LCD backend: {text}")
            return
        try:
            image = self._render_text_image(text, icon)
            self._lcd.img_show(image)
            self.get_logger().info(f"LCD display updated: text={text!r}, icon={icon!r}")
        except Exception as exc:
            self.get_logger().error(f"LCD render failed: {exc}")

    def _on_led(self, msg: String) -> None:
        payload = self._decode_payload(msg.data)
        params = payload.get("params", payload)
        if not isinstance(params, dict):
            return
        if self._led is None and self._led_service_client is None:
            self.get_logger().info(f"LED command received without LED backend: {params}")
            return
        try:
            self._apply_led_params(params)
            self.get_logger().info(f"LED command applied: {json.dumps(params, ensure_ascii=False)}")
        except Exception as exc:
            self.get_logger().error(f"LED command failed: {exc}")

    def _render_text_image(self, text: str, icon: str):
        image = Image.new("RGB", (self.lcd_width, self.lcd_height), self.lcd_bg_color)
        draw = ImageDraw.Draw(image)
        main_font = self._load_font(self.lcd_font_size)
        accent_font = self._load_font(max(16, self.lcd_font_size // 2))
        lines = self._wrap_text(draw, text, main_font, self.lcd_wrap_width_px)

        line_heights = []
        total_height = 0
        for line in lines:
            bbox = draw.textbbox((0, 0), line, font=main_font)
            height = max(1, bbox[3] - bbox[1])
            line_heights.append(height)
            total_height += height
        if lines:
            total_height += self.lcd_line_spacing_px * (len(lines) - 1)

        current_y = max(20, (self.lcd_height - total_height) // 2)
        for idx, line in enumerate(lines):
            bbox = draw.textbbox((0, 0), line, font=main_font)
            width = bbox[2] - bbox[0]
            draw.text(
                ((self.lcd_width - width) / 2, current_y),
                line,
                font=main_font,
                fill=self.lcd_text_color,
            )
            current_y += line_heights[idx] + self.lcd_line_spacing_px

        if icon:
            accent = f"[{icon}]"
            bbox = draw.textbbox((0, 0), accent, font=accent_font)
            width = bbox[2] - bbox[0]
            height = bbox[3] - bbox[1]
            draw.text(
                ((self.lcd_width - width) / 2, self.lcd_height - height - 14),
                accent,
                font=accent_font,
                fill=self.lcd_accent_color,
            )

        return image

    def _load_font(self, size: int):
        candidates: List[str] = []
        if self.lcd_font_path.strip():
            candidates.append(self.lcd_font_path.strip())
        candidates.extend(self.lcd_font_candidates)
        for path in candidates:
            if not path or not os.path.exists(path):
                continue
            try:
                return ImageFont.truetype(path, size)
            except Exception:
                continue
        return ImageFont.load_default()

    @staticmethod
    def _wrap_text(draw, text: str, font, max_width: int) -> List[str]:
        lines: List[str] = []
        for paragraph in str(text).splitlines() or [""]:
            paragraph = paragraph.strip()
            if not paragraph:
                lines.append("")
                continue
            current = ""
            tokens = paragraph.split(" ") if " " in paragraph else list(paragraph)
            joiner = " " if " " in paragraph else ""
            for token in tokens:
                candidate = token if not current else f"{current}{joiner}{token}"
                bbox = draw.textbbox((0, 0), candidate, font=font)
                width = bbox[2] - bbox[0]
                if current and width > max_width:
                    lines.append(current)
                    current = token
                else:
                    current = candidate
            if current:
                lines.append(current)
        return lines or [""]

    def _apply_led_params(self, params: Dict[str, Any]) -> None:
        self._stop_led_blink()
        command = str(params.get("command", "")).strip().lower()
        mode = str(params.get("mode", command or "solid")).strip().upper()
        pixels = params.get("pixels") or []

        if command == "clear" or mode in {"CLEAR", "OFF"}:
            self._led_clear()
            return

        color = self._resolve_led_color(params)
        if mode == "BLINK":
            rate = max(0.2, float(params.get("rate", 1.0) or 1.0))
            self._led_blink_color = color
            self._led_blink_on = False
            period = max(0.1, 1.0 / (rate * 2.0))
            self._led_blink_timer = self.create_timer(period, self._tick_led_blink)
            self._tick_led_blink()
            return

        if command == "set_pixel" or pixels:
            self._led_set_pixels([int(pixel) for pixel in pixels], color)
            return

        self._led_fill(color)

    def _tick_led_blink(self) -> None:
        if self._led is None and self._led_service_client is None:
            return
        self._led_blink_on = not self._led_blink_on
        if self._led_blink_on:
            self._led_fill(self._led_blink_color)
        else:
            self._led_clear()

    def _stop_led_blink(self) -> None:
        if self._led_blink_timer is not None:
            self._led_blink_timer.cancel()
            self._led_blink_timer = None
        self._led_blink_on = False

    def _led_fill(self, color: Tuple[int, int, int]) -> None:
        if self._dispatch_led_service("fill", color=color):
            return
        if self._led is None:
            raise RuntimeError("No LED backend available.")
        self._led.fill(color)

    def _led_clear(self) -> None:
        if self._dispatch_led_service("clear"):
            return
        if self._led is None:
            raise RuntimeError("No LED backend available.")
        self._led.clear()

    def _led_set_pixels(self, pixels: List[int], color: Tuple[int, int, int]) -> None:
        if self._dispatch_led_service("set_pixel", color=color, pixels=pixels):
            return
        if self._led is None:
            raise RuntimeError("No LED backend available.")
        for pixel in pixels:
            self._led.set_pixel(int(pixel), color)
        self._led.show()

    def _dispatch_led_service(
        self,
        command: str,
        *,
        color: Optional[Tuple[int, int, int]] = None,
        pixels: Optional[List[int]] = None,
    ) -> bool:
        if self._led_service_client is None or PinkySetLed is None:
            return False
        if not self._led_service_client.wait_for_service(timeout_sec=self.led_service_wait_sec):
            if not self._warned_led_service_unavailable:
                self.get_logger().warn(
                    f"LED service {self.led_service_name} is unavailable; falling back to local backend."
                )
                self._warned_led_service_unavailable = True
            return False
        self._warned_led_service_unavailable = False
        request = PinkySetLed.Request()
        request.command = command
        request.pixels = [int(pixel) for pixel in (pixels or [])]
        if color is not None:
            request.r = int(color[0])
            request.g = int(color[1])
            request.b = int(color[2])
        future = self._led_service_client.call_async(request)
        future.add_done_callback(self._on_led_service_result)
        return True

    def _on_led_service_result(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"LED service call failed: {exc}")
            return
        if response is None:
            self.get_logger().error("LED service call returned no response.")
            return
        if not bool(response.success):
            self.get_logger().error(f"LED service rejected command: {response.message}")

    @staticmethod
    def _decode_payload(raw: str) -> Dict[str, Any]:
        if not isinstance(raw, str):
            return {"text": str(raw)}
        try:
            payload = json.loads(raw)
            return payload if isinstance(payload, dict) else {"value": payload}
        except Exception:
            return {"text": raw}

    def _resolve_led_color(self, params: Dict[str, Any]) -> Tuple[int, int, int]:
        color_name = str(params.get("color", "")).strip().upper()
        if color_name in COLOR_NAME_MAP:
            return COLOR_NAME_MAP[color_name]
        if all(key in params for key in ("r", "g", "b")):
            return (
                int(params.get("r", 255)),
                int(params.get("g", 255)),
                int(params.get("b", 255)),
            )
        return (255, 255, 255)

    @staticmethod
    def _parse_rgb_string(raw: str, default: Tuple[int, int, int]) -> Tuple[int, int, int]:
        try:
            parts = [int(token.strip()) for token in str(raw).split(",")]
            if len(parts) != 3:
                return default
            return tuple(max(0, min(255, value)) for value in parts)
        except Exception:
            return default

    def destroy_node(self) -> bool:
        self._stop_led_blink()
        if self.led_clear_on_shutdown:
            try:
                self._led_clear()
            except Exception:
                pass
        if self._lcd is not None and self.lcd_clear_on_shutdown:
            try:
                self._lcd.clear()
            except Exception:
                pass
        if self._lcd is not None and hasattr(self._lcd, "close"):
            try:
                self._lcd.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OfficeRobotUiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
