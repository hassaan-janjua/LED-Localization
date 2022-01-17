from time import sleep
from time import time
from picamera import PiCamera
import shutil
import argparse
import os


# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-x", "--width", type=int, default=1920,
	help="Frame width")
ap.add_argument("-y", "--height", type=int, default=1088,
	help="Frame height")
ap.add_argument("-w", "--wait", type=int, default=1,
	help="Wait between images")
ap.add_argument("-c", "--count", type=int, default=50,
	help="Number of images to be captured")

args = vars(ap.parse_args())

width = args["width"]
height = args["height"]
wt = args["wait"]
count = args["count"]

camera = PiCamera()
camera.resolution = (width, height)


sleep(2)
camera.start_preview()

if os.path.exists('im'):
  shutil.rmtree('im/')

os.mkdir('im') 

i = 0
for filename in camera.capture_continuous('im/img{counter:03d}.jpg'):
  end = time()
  if i >= count:
    break
  i = i + 1
  print('Captured %s' % filename)
  sleep(wt) # wait wt seconds
