from picamera2 import Picamera2
import cv2
import numpy as np

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (320, 240)}))
picam2.start()

# Capture one frame and save it to a file
frame = picam2.capture_array()
frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
cv2.imwrite("test_frame.jpg", frame)
print("Frame saved to test_frame.jpg")

picam2.stop()
