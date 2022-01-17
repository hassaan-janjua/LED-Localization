#!/usr/bin/python3

from subprocess import Popen, PIPE
from numpy.linalg import inv
import numpy as np
import re
import os
import cv2
import signal
import sys
import serial
import time
import threading
import queue
import datetime
import logging
import logging.handlers

def wifi_off():
  global sleepy_pi_logger
  print("wifi_off")
  sleepy_pi_logger.info("Turning WiFi Off")
  os.system("ifconfig wlan0 down")

def wifi_on():
  global sleepy_pi_logger
  print("wifi_on")
  sleepy_pi_logger.info("Turning WiFi On")
  os.system("ifconfig wlan0 up")

def handle_action(cmd):
  if cmd.startswith("02"):
    wifi_off()
  elif cmd.startswith("03"):
    wifi_on()

def uart_receiver():
  global is_active
  global lock
  global sleepy_pi_logger
  global serialHandle
  
  # Never stop the uart_receiver Thread
  while True:
    tlen = 0
    with lock:
      msg = ""
      raw = []
      while (serialHandle.inWaiting()>0):
        rec = serialHandle.read(1)
        tlen += 1
        raw.append(rec)
        msg += rec.decode('UTF-8')
        time.sleep(0.001)
        
    if tlen > 0:
      messages = msg.split("\r\n")
      for m in messages:
        if m.startswith("04") or m.startswith("05"):
          sleepy_pi_logger.debug(m)
        else:
          sleepy_pi_logger.info(m)
        if m.startswith("01"):
          handle_action(m[2:])
            
    time.sleep(2)

def run(command):
  process = Popen(command.split(' '), stdout=PIPE, shell=False)
  yield process
  while True:
    line = process.stdout.readline().decode('UTF-8').rstrip()
    if not line:
      break
    yield line

def loadIntrinsicParameters( intrinsicParametersFile):
  cm = np.zeros(9, dtype = "float64")
  cmpoints = 0
  
  cmi = np.zeros(9, dtype = "float64")
  cmipoints = 0
  
  dc = None
  dcpoints = 0
  
  status = True

  with open(intrinsicParametersFile) as fl:
      for line in fl:
        line = line.rstrip()
        if not line.startswith("#") and len(line) > 0:
          data = line.split(",")
          
          if (cmpoints < 9):
            cm[cmpoints] = data[0]
            cmpoints += 1
            cm[cmpoints] = data[1]
            cmpoints += 1
            cm[cmpoints] = data[2]
            cmpoints += 1
          elif (dcpoints < 1):
            dc = np.array([data[0], data[1], data[2], data[3], data[4] ], dtype = "float64")
            dcpoints += 1
          elif (cmipoints < 9):
            cmi[cmipoints] = data[0]
            cmipoints += 1
            cmi[cmipoints] = data[1]
            cmipoints += 1
            cmi[cmipoints] = data[2]
            cmipoints += 1
  cmx = None
  if cmpoints == 9:
    cmx = np.float64([  [cm[0], cm[1], cm[2]],
                        [cm[3], cm[4], cm[5]],
                        [cm[6], cm[7], cm[8]]])
  else:
    status = False

  cmxi = None
  if cmipoints == 9:
    cmxi = np.float64([ [cmi[0], cmi[1], cmi[2]],
                        [cmi[3], cmi[4], cmi[5]],
                        [cmi[6], cmi[7], cmi[8]]])
  if dc is None:
    status = False
  return status, cmx, cmxi, dc

def loadExtrinsicParameters(extrinsicParametersFile):
  global rpi_logger
  
  cr = np.zeros(9, dtype = "float64")
  crn = 0
  ctn = 0
  cameraRotation = None
  cameraTranslation = None
  status = True
  
  try:
    with open(extrinsicParametersFile) as fl:
        for line in fl:
          line = line.rstrip()
          if not line.startswith("#") and len(line) > 0:
            data = line.split(",")
            
            if (crn < 9):
              cr[crn] = data[0]
              crn += 1
              cr[crn] = data[1]
              crn += 1
              cr[crn] = data[2]
              crn += 1
            elif (ctn < 1):
              cameraTranslation = np.array([[float(data[0])], [float(data[1])], [float(data[2])]], dtype = "float64")
              ctn += 1
    if (crn == 9):
      cameraRotation = np.float64([ [cr[0], cr[1], cr[2]],
                                    [cr[3], cr[4], cr[5]],
                                    [cr[6], cr[7], cr[8]]])
  except:
    rpi_logger.error("Error in loadExtrinsicParameters")
    status = False

  return (status, cameraRotation, cameraTranslation)

def loadLedWorldCoordinates(ledWorldCoordinatesFile):
  global rpi_logger
  
  ledWorldCoordinates = {}
  status = True

  try:
    with open(ledWorldCoordinatesFile) as fl:
        for line in fl:
          line = line.rstrip()
          if not line.startswith("#") and len(line) > 0:
            data = line.split(",")
            ledWorldCoordinates[int(data[0])] = np.array([data[1], data[2], data[3]], dtype = "float64")
  except:
    rpi_logger.error("Error in loadLedWorldCoordinates")
    status = False

  return (status, ledWorldCoordinates)


def saveExtrinsicParameters(extrinsicParametersFile, cameraRotation, cameraTranslation) :
  global rpi_logger
  status = True
  
  try:
    if os.path.exists(extrinsicParametersFile): 
      os.remove(extrinsicParametersFile) 

    with open(extrinsicParametersFile, "w+") as fl:
      fl.write("# Camera Rotation\n")

      l = [", ", ", ", "\n"]

      for i in range(3):
        for j in range(3):
          fl.write(str(cameraRotation[i,j]) +  l[j])
        fl.write("\n")

      fl.write("# Camera Translation\n")

      for i in range(3):
        fl.write(str(cameraTranslation[i][0]) +  l[i])
  except:
    rpi_logger.error("Error in saveExtrinsicParameters")
    status = False

  return status
      
def saveIntrinsicParameters(intrinsicParametersFile, cameraMatrix, cameraMatrixInv, distortionCoefficients) :

  global rpi_logger
  status = True

  try:
    
    if os.path.exists(intrinsicParametersFile): 
      os.remove(intrinsicParametersFile) 

    with open(intrinsicParametersFile, "w+") as fl:

      l = [", ", ", ", "\n"]
      l5 = [", ",", ",", ", ", ", "\n"]

      fl.write("# Camera Matrix\n")
      for i in range(3):
        for j in range(3):
          fl.write(str(cameraMatrix[i,j]) +  l[j])
      fl.write("\n")

      fl.write("# distortionCoefficients\n")
      for i in range(5):
        fl.write(str(distortionCoefficients[i]) +  l5[i])
      fl.write("\n")


      fl.write("# Camera Matrix Inverse\n")
      for i in range(3):
        for j in range(3):
          fl.write(str(cameraMatrixInv[i,j]) +  l[j])
      fl.write("\n")
  except:
    rpi_logger.error("Error in saveIntrinsicParameters")
    status = False
    
  return status

def mapLedImageToWorldCoordinates(ledWorldCoordinates, ledImageCoordinates):
  status = True
  calibPointsCount = 0
  imagePoints = None
  worldPoints = None
  
  try:
    imagePoints = np.empty((4,2), dtype = "float64")
    worldPoints = np.empty((4,3), dtype = "float64")
    for key, value in ledWorldCoordinates.items():
      if (key in ledImageCoordinates and calibPointsCount < 4):
        imagePoints[calibPointsCount] = np.array(ledImageCoordinates[key])
        worldPoints[calibPointsCount] = np.array(value)
        calibPointsCount = calibPointsCount + 1
  except:
    status = False

  return (status, calibPointsCount, imagePoints, worldPoints)

def calculateExtrinsicParameters(ledImageCoordinates, ledWorldCoordinates, cameraMatrix, distortionCoefficients):
  status = True
  cameraTranslation = None
  cameraRotation = None

  try:
    (status, calibPointsCount, imagePoints, worldPoints) = mapLedImageToWorldCoordinates(ledWorldCoordinates, ledImageCoordinates)
    
    if status:
      if calibPointsCount == 4:

        distortionCoefficients = None

        (status, rotationVector, translationVector)  = cv2.solvePnP(worldPoints, imagePoints, cameraMatrix, distortionCoefficients)

        if (status):
          cameraRotation, _ = cv2.Rodrigues(rotationVector)

          cameraRotation = np.transpose(cameraRotation)
          _cameraRotation =  np.multiply( cameraRotation, -1)
          cameraTranslation = np.matmul( _cameraRotation, translationVector )
      else:
        status = False
  except:
    status = False

  return status, cameraRotation, cameraTranslation

def setupIntrinsicCalibration(intrinsicParametersFile):
  global rpi_logger
  
  status = True
  cameraMatrix = None
  cameraMatrixInv = None
  distortionCoefficients = None

  try:
    status, cameraMatrix, cameraMatrixInv, distortionCoefficients = loadIntrinsicParameters(intrinsicParametersFile)

    if status and cameraMatrixInv is None and cameraMatrix is not None:
      cameraMatrixInv = inv(cameraMatrix)
      status = saveIntrinsicParameters(intrinsicParametersFile, cameraMatrix, cameraMatrixInv, distortionCoefficients)
      if status:
        rpi_logger.warning("Extrinsic parameter calibration complete.")
  except:
    status = False
  
  return (status, cameraMatrix, cameraMatrixInv, distortionCoefficients)

  
def parseImageCoordinates(line):
  status = True
  id = None
  p = None
  x = None
  y = None
  ts = None
  errors = None
  qsize = None
  
  try:
    result = pattern.match(line)
    if result:
      id = int(result.group(1))
      p = int(result.group(2))
      x  = int(result.group(3))
      y  = int(result.group(4))
      ts  = float(result.group(5))
      errors  = int(result.group(6))
      qsize  = int(result.group(7))
  except:
    status = False
  
  return (status, id, p, x, y, ts, errors, qsize)

  
def setupExtrinsicCalibration(extrinsicParametersFile, ledImageCoordinates, ledWorldCoordinates, cameraMatrix, distortionCoefficients):
  global rpi_logger
  status = True

  try:
    (status, cameraRotation, cameraTranslation) = loadExtrinsicParameters(extrinsicParametersFile)
    if (not status):

      (status, cameraRotation, cameraTranslation) = calculateExtrinsicParameters(ledImageCoordinates, ledWorldCoordinates, cameraMatrix, distortionCoefficients)
      
      if status:
        status = saveExtrinsicParameters(extrinsicParametersFile, cameraRotation, cameraTranslation)
        if (not status):
          rpi_logger.error ("Failed to save saveExtrinsicParameters in %s." % extrinsicParametersFile)
  except:
    status = False
  
  return (status, cameraRotation, cameraTranslation)

def addImageLed(ledImageCoordinates, id, x, y):
  ledImageCoordinates[id] = np.array([x, y], dtype = "float64")

def addWorldLed(ledWorldCoordinates, id, x, y, z):
  ledWorldCoordinates[id] = np.array([x, y, z], dtype = "float64")

def calculateWorldCoorinates(x, y, height, cameraMatrix, cameraMatrixInv, distortionCoefficients, cameraRotation, cameraTranslation):

    status = True
    X = None
    Y = None
    Z = None

    try:

      inputPoints = np.array([[[x, y]]], dtype = "float64")
      
      #undistortedPoints = cv2.undistortPoints(inputPoints, cameraMatrix, distortionCoefficients) # No distortion coefficients, so no need to undistort
      undistortedPoints = inputPoints

      los = np.zeros((3, 1), dtype = "float64") 

      # Initially put the pixel coordinates in the los varaible.
      los[0] = undistortedPoints[0][0][0]
      los[1] = undistortedPoints[0][0][1]
      los[2] = 1

      los = np.matmul(cameraMatrixInv, los)

      # Apply camera rotation to the line of sight vector.
      los = np.matmul(cameraRotation, los)

      # Calculate scale to take projection of the LOS vector on the z=0 plane.
      scale = (-1*cameraTranslation[2] + height)/los[2]
      scaledLOS = np.multiply( los, scale)
      point3D = scaledLOS + cameraTranslation

      X = point3D[0]
      Y = point3D[1]
      Z = point3D[2]
    except:
      status = False

    return (status, X, Y, Z)
    
def getLedHeight(id, ledHeights):
  if id in ledHeights:
    return ledHeights[id]
  elif 0 in ledHeights:
    return ledHeights[0]
  else:
    return None
  
def reportLedCoordinatesToUART(cameraMatrix, cameraMatrixInv, distortionCoefficients, extrinsicParametersFile, ledWorldCoordinatesFile, ledHeights, base_folder, localizerArgs):
  global localizerProcess
  global data_to_send
  global rpi_logger

  status = True
  ledWorldCoordinatesLoaded = False
  ledWorldCoordinates = None
  ledImageCoordinates = {}

  
  (externalCalibrationDone, cameraRotation, cameraTranslation) = loadExtrinsicParameters(extrinsicParametersFile)
  if externalCalibrationDone:
    rpi_logger.warning("Loaded extrinsicParametersFile: %s." % extrinsicParametersFile)
  else:
    rpi_logger.critical("Could not load extrinsicParametersFile: %s." % extrinsicParametersFile)
          
  localizerBin = base_folder + "localizer " + localizerArgs
    
  for line in run(localizerBin):
    if localizerProcess is None:
      localizerProcess = line
      rpi_logger.warning("localizer process started.")
      continue
    if line is None:
      rpi_logger.critical("Error in localizer process.")
      break

    (status, id, p, x, y, ts, errors, qsize) = parseImageCoordinates(line)
    
    if (not status):
      rpi_logger.critical("Error in parseImageCoordinates.")
      break
    
    if (id is None):
      print ("%s" % line, flush = True)
      rpi_logger.debug("%s" % line);
      continue

    print ("Detected: %s" % line, flush = True)
    rpi_logger.warning("Detected: %s" % line);
    
    if (externalCalibrationDone):
      #getLedHeight
      height = getLedHeight(id, ledHeights)

      if height is None:
        rpi_logger.error("Error in getting height for LED: %d - (%d, %d)." % (id, x, y))
        continue

      # Convert image coordnates to world coordinates
      (status, X, Y, Z) = calculateWorldCoorinates(x, y, height, cameraMatrix, cameraMatrixInv, distortionCoefficients, cameraRotation, cameraTranslation)

      if (not status):
        rpi_logger.error("Error in calculating coordinates of LED: %d - (%d, %d)." % (id, x, y))
        continue


      # Print the coordinates to the UART
      # Convert to binary
      idid = p.to_bytes(2, byteorder='big', signed=False) 
      if (X < 0):
        # rpi_logger.info("Clamping X to zero: X = %d" % int(X*10 + 0.5))
        X = 0
      if (Y < 0):
        # rpi_logger.info("Clamping Y to zero: Y = %d" % int(Y*10 + 0.5))
        Y = 0
      xx = (int(X*10 + 0.5)).to_bytes(2, byteorder='big', signed=False) 
      yy = (int(Y*10 + 0.5)).to_bytes(2, byteorder='big', signed=False) 
      send_data = idid + xx + yy
      if data_to_send.full():
        data_to_send.get()
      data_to_send.put({"id": id , "data": send_data, "time": datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')})
      print("%04d: (%f, %f, %f), (%08d, %08d, %08d)" % (id, X, Y, Z, int(X*10 + 0.5), int(Y*10 + 0.5), int(Z*10 + 0.5)), flush = True)
      rpi_logger.warning("%04d: (%f, %f, %f), (%08d, %08d, %08d)" % (id, X, Y, Z, int(X*10 + 0.5), int(Y*10 + 0.5), int(Z*10 + 0.5)))
      
    else:
      addImageLed(ledImageCoordinates, id, x, y)
      print("LED - %04d: (%08d, %08d)" % (id, int(x), int(y)), flush = True)
      rpi_logger.debug("LED - %04d: (%08d, %08d)" % (id, int(x), int(y)))
      if (len(ledImageCoordinates) >= 4):
        (status, ledWorldCoordinates) = loadLedWorldCoordinates(ledWorldCoordinatesFile)
        if (status):
          (externalCalibrationDone, cameraRotation, cameraTranslation) = setupExtrinsicCalibration(extrinsicParametersFile, ledImageCoordinates, ledWorldCoordinates, cameraMatrix, distortionCoefficients)
        else:
          rpi_logger.error ("Faild to load ledWorldCoordinatesFile %s." % ledWorldCoordinatesFile)
      
  if localizerProcess is not None:
    localizerProcess.send_signal(signal.SIGINT)
    rpi_logger.critical("Terminating localization process.")
    time.sleep(3)
    localizerProcess.kill()

  return status
    
def loadLedHeights(ledHeightsFile):
  leds = {}
  status = True
  
  try:
    with open(ledHeightsFile) as fl:
        for line in fl:
          line = line.rstrip()
          if not line.startswith("#") and len(line) > 0:
            data = line.split(",")
            leds[int(data[0])] = float(data[1])
  except:
    status = False

  return status, leds

  
def cleanup():
  global is_active
  global rpi_logger
  global localizerProcess

  is_active = False

  rpi_logger.warning("cleanup()")

  if localizerProcess is not None:
    localizerProcess.send_signal(signal.SIGINT)
    rpi_logger.warning("cleanup(): Terminating localization process.")
    time.sleep(3)
    localizerProcess.kill()

  sys.exit(0)
  

def signal_handler(sig, frame):
  global rpi_logger
  rpi_logger.warning("signal_handler()")
  cleanup()

def uart_sender():
  global data_to_send
  global is_active
  global lock
  global rpi_logger
  
  while is_active:
    ct = 0
    while ct < 5 and not data_to_send.empty():
      ct += 1
      d = data_to_send.get()
      
      data = d["data"]
      rpi_logger.debug("Sending data:: time: %s, ID: %d, Data Length: %d" % (d["time"], d["id"], len(data)))
      
      with lock:
        serialHandle.write(data)
    
    if ct == 0:
      d = (int(65535)).to_bytes(2, byteorder='little', signed=False)
      data = d + d + d
      with lock:
        serialHandle.write(data)
    
    time.sleep(10)

def setup_logs():
  global sleepy_pi_logger
  global rpi_logger
  
  SLEEPYPI_LOG_FILENAME = 'localization.log'

  SLEEPY_PI_LOG = 'SLEEPYPI'
  RPI_LOG = 'RPI'

  # Set up a specific logger with our desired output level
  sleepy_pi_logger = logging.getLogger(SLEEPY_PI_LOG)
  rpi_logger = logging.getLogger(RPI_LOG)

  # Add the log message handler to the logger
  log_handler = logging.handlers.RotatingFileHandler(SLEEPYPI_LOG_FILENAME, maxBytes=1048576, backupCount=5)
  log_formatter = logging.Formatter('%(asctime)-15s %(name)-8s %(levelname)-9s %(message)s')
  log_handler.setFormatter(log_formatter)

  sleepy_pi_logger.addHandler(log_handler)
  rpi_logger.addHandler(log_handler)


def main():
  global pattern
  global format
  global data_to_send
  global lock
  global sleepy_pi_logger
  global rpi_logger
  global ledHeightsFile
  global intrinsicParametersFile
  global ledWorldCoordinatesFile
  global extrinsicParametersFile
  global localizerArgs
  global serialHandle

  setup_logs()
  sleepy_pi_logger.setLevel(logging.INFO)
  rpi_logger.setLevel(logging.INFO)
  
  rpi_logger.critical("Starting up Localization")
  
  signal.signal(signal.SIGINT, signal_handler)

  (status, cmx, cmxi, dc) = setupIntrinsicCalibration(intrinsicParametersFile)
  if status:
    print("Loaded intrinsicParametersFile: %s." % intrinsicParametersFile)
    rpi_logger.warning("Loaded intrinsicParametersFile: %s." % intrinsicParametersFile)
  else:
    print("Could not load intrinsicParametersFile: %s." % intrinsicParametersFile)
    rpi_logger.critical("Could not load intrinsicParametersFile: %s." % intrinsicParametersFile)
    exit()
  
  (status , ledHeights) = loadLedHeights(ledHeightsFile)
  if status:
    print("Loaded ledHeightsFile: %s." % ledHeightsFile)
    rpi_logger.warning("Loaded ledHeightsFile: %s." % ledHeightsFile)
  else:
    print("Could not load ledHeightsFile: %s." % ledHeightsFile)
    rpi_logger.critical("Could not load ledHeightsFile: %s." % ledHeightsFile)
    exit()
    
  os.system("killall -9 localizer")
  
  pattern = re.compile(format)
  
  data_to_send = queue.Queue(50)
  
  lock = threading.Lock()
  
  serialHandle = serial.Serial('/dev/ttyS0', 9600)
  
  if serialHandle is not None:
    print("Serial Handle opened successfully.")
    rpi_logger.warning("Serial Handle opened successfully.")
  else:
    print("Error in opening serial handle.")
    rpi_logger.critical("Error in opening serial handle.")
    exit()
  
  sender_thread = threading.Thread(target=uart_sender)
  receiver_thread = threading.Thread(target=uart_receiver)

  sender_thread.start()
  receiver_thread.start()

  status = reportLedCoordinatesToUART(cmx, cmxi, dc, extrinsicParametersFile, ledWorldCoordinatesFile, ledHeights, base_folder, localizerArgs)
  
  cleanup();

  sender_thread.join()
  receiver_thread.join()
  
  if (not status):
    print("Error while executing localization.")
    rpi_logger.critical("Error while executing localization.")
  
  
localizerProcess = None
pattern = None
format = r"(\d+): \((\d+), (\d+), (\d+)\).*Timeshift: (\-?\d+\.?\d*).*errors: (\d+).*qsize: (\d+)"
data_to_send = None
lock = None
is_active = True
sleepy_pi_logger = None
rpi_logger = None
serialHandle = None

base_folder             = "/home/pi/localization/"
ledHeightsFile          = base_folder + "ledHeightsFile.txt"
intrinsicParametersFile = base_folder + "intrinsicParametersFile.txt"
ledWorldCoordinatesFile = base_folder + "ledWorldCoordinatesFile.txt"
extrinsicParametersFile = base_folder + "extrinsicParametersFile.txt"
localizerArgs           = "-b 10 -t 2 -l 0.1 -f 50 -r 50"
  
if __name__ == '__main__':
  main()
