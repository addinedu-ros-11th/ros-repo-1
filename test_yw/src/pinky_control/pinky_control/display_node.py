import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
import sys
from PIL import Image, ImageDraw, ImageFont

# pinkylib 경로 추가 (LCD 드라이버 로드용)
sys.path.append("/home/ywpc/Projects_ws/final_ros_project/ros-repo-1/test_yw/pinkylib/lcd")
try:
    from pinky_lcd.pinky_lcd import LCD
except ImportError:
    LCD = None

class DisplayNode(Node):
    def __init__(self):
        super().__init__("display_node")
        
        if LCD is None:
            self.get_logger().error("Pinky LCD Library not found!")
            return

        try:
            self.lcd = LCD()
            self.lcd.clear(0x0000) # 검정색 초기화
        except Exception as e:
            self.get_logger().error(f"Failed to initialize LCD hardware: {e}")
            self.lcd = None

        self.display_sub = self.create_subscription(
            String,
            "display",
            self._display_callback,
            10
        )
        
        # 폰트 설정 (한글 폰트 경로 확인 필요)
        self.font_path = "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"
        if not os.path.exists(self.font_path):
            self.font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"

        self.get_logger().info("Display Node initialized and waiting for messages...")

    def _display_callback(self, msg):
        if self.lcd is None: return
        
        try:
            data = json.loads(msg.data)
            text = data.get("text", "")
            icon_type = data.get("icon", "info")
            
            # 320x240 이미지 생성 (LCD 해상도에 맞춤)
            # pinky_lcd.py 기준 w=240, h=320 이므로 세로 모드 기준
            img = Image.new("RGB", (self.lcd.w, self.lcd.h), "black")
            draw = ImageDraw.Draw(img)
            
            # 아이콘 및 배경 색상 결정
            bg_color = (0, 0, 0)
            if icon_type == "qr": bg_color = (40, 40, 80)
            elif icon_type == "qr_success": bg_color = (0, 80, 0)
            elif icon_type == "qr_failed": bg_color = (80, 0, 0)
            
            draw.rectangle([0, 0, self.lcd.w, self.lcd.h], fill=bg_color)

            # 텍스트 그리기
            try:
                font = ImageFont.truetype(self.font_path, 25)
                # 텍스트 중앙 정렬 계산
                bbox = draw.textbbox((0, 0), text, font=font)
                w, h = bbox[2] - bbox[0], bbox[3] - bbox[1]
                draw.text(((self.lcd.w - w) / 2, (self.lcd.h - h) / 2), text, font=font, fill="white")
            except:
                draw.text((10, 150), text, fill="white")

            self.lcd.img_show(img)
            
        except Exception as e:
            self.get_logger().warn(f"Display update failed: {e}")

def main():
    rclpy.init()
    node = DisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
