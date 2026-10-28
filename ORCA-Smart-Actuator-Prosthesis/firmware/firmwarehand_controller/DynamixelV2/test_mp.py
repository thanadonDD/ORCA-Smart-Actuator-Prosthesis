import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import cv2
import numpy as np

model_path = 'hand_landmarker.task'
BaseOptions = python.BaseOptions
HandLandmarker = vision.HandLandmarker
HandLandmarkerOptions = vision.HandLandmarkerOptions
VisionRunningMode = vision.RunningMode

options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=model_path),
    running_mode=VisionRunningMode.IMAGE,
    num_hands=1)

with HandLandmarker.create_from_options(options) as landmarker:
    print("Landmarker created successfully")
    
    # Create a dummy image
    dummy_img = np.zeros((480, 640, 3), dtype=np.uint8)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=dummy_img)
    
    result = landmarker.detect(mp_image)
    print("Result:", result)
