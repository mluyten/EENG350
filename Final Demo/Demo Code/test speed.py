from ComputerVision import *
import io
import matplotlib.pyplot as plt
from PIL import Image
import time
import numpy as np

cam = ComputerVision()
#cam.startCapture()
#time.sleep(4)
#past = cam.image
cam.startCapture()
time.sleep(2)

while True:
    print(cam.arucoExist())
cam.stopCapture()