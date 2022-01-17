# import the necessary packages
from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import imutils
import time
import cv2

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-x", "--width", type=int, default=1920,
	help="Frame width")
ap.add_argument("-y", "--height", type=int, default=1088,
	help="Frame height")
ap.add_argument("-f", "--format", default="bgr",
	help="Image format")
ap.add_argument("-n", "--num-frames", type=int, default=100,
	help="# of frames to loop over for FPS test")
ap.add_argument("-d", "--display", type=int, default=-1,
	help="Whether or not frames should be displayed")
ap.add_argument("-c", "--count", type=int, default=0,
	help="Whether or not frames should be displayed")

args = vars(ap.parse_args())

width = args["width"]
height = args["height"]
format = args["format"]
count = args["count"]

count = 0
with open('count', 'r') as f:
    count = int(f.readlines()[0])

count += 1
with open('count', 'w') as f:
    f.write("%d" % count)

camera = PiCamera()

camera.resolution = (width, height)

time.sleep(2)

camera.shutter_speed = 10000
camera.framerate = 25
camera.exposure_mode = 'off'
g = camera.awb_gains
camera.awb_mode = 'off'
camera.awb_gains = g

print "start recording!"
camera.start_recording("cam1_%d_10ms.h264" % count)
camera.wait_recording(10)
# time.sleep(40)
camera.stop_recording()
camera.close()

print "done"

quit()
