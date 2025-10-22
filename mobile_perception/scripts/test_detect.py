import cv2
from pupil_apriltags import Detector

image_path = 'sample_tag.jpg'
image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

if image is None:
    raise ValueError(f"Image not loaded correctly. Check the path: {image_path}")

at_detector = Detector(families='tag36h11')
tags = at_detector.detect(image, estimate_tag_pose=False, camera_params=None, tag_size=None)

for tag in tags:
    print('Detected tag:', tag)
