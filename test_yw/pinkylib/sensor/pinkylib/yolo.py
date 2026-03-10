import cv2
import time

from ultralytics import YOLO

class Yolo():
    def __init__(self): 
        self.model = None

    def set_model(self, train_model="best.pt"):
        self.model = YOLO(train_model)

    def detect_yolo(self, frame, conf=0.5):
        results = self.model(frame, verbose=False, conf=conf)
        clss = results[0].boxes.cls.numpy()
        
        annotated_frame = results[0].plot()
        
        return results, annotated_frame

        