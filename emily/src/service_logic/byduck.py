from picamera2 import Picamera2
import cv2
import numpy as np
from ultralytics import YOLO
import pygame

class DuckDetector:
    def __init__(self):
        # Initialize camera
        self.camera = Picamera2()
        config = self.camera.create_preview_configuration(main={"format": "XRGB8888", "size": (320, 240)})
        self.camera.configure(config)
        print("Camera initialized with XRGB8888 format at 320x240 resolution")
        self.camera.start()
        print("Camera started")

        # Load YOLOv11 model
        self.model = YOLO("models/yolo11n.pt")
        print("YOLOv11n model loaded")

        # Flag to track if person is detected
        self.person_detected = False

    def process_frame(self, draw_labels=True):
        # Capture frame
        frame = self.camera.capture_array()
        print(f"Frame shape: {frame.shape}")

        # Process frame
        if len(frame.shape) == 3 and frame.shape[2] in [3, 4]:
            if frame.shape[2] == 4:
                frame = frame[:, :, :3]
                print("Dropped alpha channel from XRGB8888 frame")
            frame_bgr = frame.copy()

            # Detect persons
            results = self.model.predict(frame_bgr, conf=0.25, classes=[0])
            print("YOLO prediction completed for person class")

            # Check if person is detected
            self.person_detected = len(results) > 0 and len(results[0].boxes) > 0
            print(f"Person detected: {self.person_detected}")

            # Draw YOLO labels if requested
            if draw_labels:
                for r in results:
                    boxes = r.boxes
                    for box in boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        conf = box.conf[0]
                        cls = int(box.cls[0])
                        if cls == 0:  # Person
                            label = f'Person {conf:.2f}'
                            cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.putText(frame_bgr, label, (x1, y1 - 10), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                print("YOLO labels drawn on frame")

            # Convert frame to Pygame surface
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            frame_surface = pygame.surfarray.make_surface(frame_rgb.swapaxes(0, 1))
            print("Frame converted to Pygame surface")

            return frame_surface
        else:
            print("Invalid frame format: Expected 3 or 4 color channels")
            return None

    def is_person_detected(self):
        return self.person_detected

    def cleanup(self):
        self.camera.stop()
        print("Camera stopped")
        self.camera.close()
        print("Camera closed")