import argparse
import os
import shutil
import cv2

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-x", "--width", type=int, default=1920,
	help="Frame width")
ap.add_argument("-y", "--height", type=int, default=1088,
	help="Frame height")
ap.add_argument("-v", "--video", default="cam14_21.h264",
	help="Video file")
ap.add_argument("-f", "--folder", default="im",
	help="Video file")

args = vars(ap.parse_args())

width = args["width"]
height = args["height"]
video = args["video"]
folder = args["folder"]


if os.path.exists(folder):
  shutil.rmtree(folder + '/')

os.mkdir(folder) 

cap = cv2.VideoCapture(video)

if cap is None:
  print('Failed to load video file\n')
counter = 0
while (True):
  ret,image = cap.read()
  if(ret is False):
    break
  pos = cap.get(cv2.CAP_PROP_POS_FRAMES)
  counter = counter + 1
  cv2.imwrite(folder + "/" + str(pos) + ".jpg", image)

print ("%d images saved" % counter)
cap.release()