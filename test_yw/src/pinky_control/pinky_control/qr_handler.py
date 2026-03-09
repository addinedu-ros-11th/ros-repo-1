import cv2
import numpy as np
from typing import Optional

class QRHandler:
    """Handles QR code detection and decoding using OpenCV."""
    
    def __init__(self):
        # Check if OpenCV is available
        if cv2 is None:
            self.detector = None
        else:
            self.detector = cv2.QRCodeDetector()

    def decode(self, compressed_image_data: bytes) -> Optional[str]:
        """Decodes QR code from compressed image bytes."""
        if not self.detector or not compressed_image_data:
            return None

        try:
            # Convert bytes to cv2 image
            np_arr = np.frombuffer(compressed_image_data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if img is None:
                return None

            # Detect and decode
            data, _, _ = self.detector.detectAndDecode(img)
            return data.strip() if data else None
        except Exception:
            return None
