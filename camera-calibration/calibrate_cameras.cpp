//============================================================================
// Name        : calibrate_cameras.cpp
// Author      : Hassaan
// Version     :
// Copyright   : 
// Description : Camera calibration application
//============================================================================

#include <opencv2/core.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <fstream>
#include <sstream>

using namespace cv;
using namespace std;

static const float calibrationSquareDimention = 0.025f; // meters
static const Size chessboardDimentions = Size(6, 9);

static void createKnownBoardPositions(Size boardSize, float squareEdgeLength, vector<Point3f>& corners)
{
  for (int i = 0; i < boardSize.height; i++)
  {
    for (int j = 0; j < boardSize.width; j++)
    {
      corners.push_back(Point3f(j* squareEdgeLength, i * squareEdgeLength, 0.0f));
    }
  }
}

//static void getChessboardCorners ( vector<Mat> images, vector<vector<Point2f>>& allFoundCorners, bool showResults = true)
//{
//  for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
//  {
//    vector<Point2f> pointBuf;
//    bool found = findChessboardCorners(*iter, chessboardDimentions, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
//
//    if (found)
//    {
//      allFoundCorners.push_back(pointBuf);
//    }
//
//    if (showResults)
//    {
//      drawChessboardCorners(*iter, chessboardDimentions, pointBuf, found);
//      imshow("Looking for Corners", *iter);
//      waitKey(0);
//    }
//  }
//}

static bool getImageChessboardCorners ( Mat image, vector<vector<Point2f>>& allFoundCorners, bool showResults = true)
{
  vector<Point2f> pointBuf;
  bool found = findChessboardCorners(image, chessboardDimentions, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

  if (found)
  {
    allFoundCorners.push_back(pointBuf);
  }

  return found;
}

//static void cameaCalibration(vector<Mat> calibrationImages, Size boardSize, float squareEgdeLength, Mat& cameraMatrix, Mat& distanceCoefficients)
//{
//  vector<vector<Point2f>> checkerboardImageSpacePoints;
//
//  getChessboardCorners(calibrationImages, checkerboardImageSpacePoints, false);
//
//  vector<vector<Point3f>> worldSpaceCornetPoints(1);
//
//  createKnownBoardPositions(boardSize, squareEgdeLength, worldSpaceCornetPoints[0]);
//  worldSpaceCornetPoints.resize(checkerboardImageSpacePoints.size(), worldSpaceCornetPoints[0]);
//
//  vector<Mat> rVectors, tVectors;
//  distanceCoefficients = Mat::zeros(8, 1, CV_64F);
//
//  calibrateCamera(worldSpaceCornetPoints, checkerboardImageSpacePoints, boardSize, cameraMatrix, distanceCoefficients, rVectors, tVectors);
//
//}

static void cameaCalibrationLowMemory(vector<vector<Point2f>> checkerboardImageSpacePoints, Size boardSize, float squareEgdeLength, Mat& cameraMatrix, Mat& distanceCoefficients)
{

  vector<vector<Point3f>> worldSpaceCornetPoints(1);

  createKnownBoardPositions(boardSize, squareEgdeLength, worldSpaceCornetPoints[0]);
  worldSpaceCornetPoints.resize(checkerboardImageSpacePoints.size(), worldSpaceCornetPoints[0]);

  vector<Mat> rVectors, tVectors;
  distanceCoefficients = Mat::zeros(8, 1, CV_64F);

  calibrateCamera(worldSpaceCornetPoints, checkerboardImageSpacePoints, boardSize, cameraMatrix, distanceCoefficients, rVectors, tVectors);

}

static bool saveCameraCalibration(string name, Mat cameraMatrix, Mat distanceCoefficients, int im_count)
{
  ofstream outStream(name);

  if (outStream)
  {
    uint16_t rows = cameraMatrix.rows;
    uint16_t columns = cameraMatrix.cols;

    outStream << "# Camera Matrix"<< endl;

    for (int r = 0; r < rows; r++)
    {
      for (int c = 0; c< columns; c++)
      {
        double value = cameraMatrix.at<double>(r, c);
        outStream << value;
        if (c != (columns-1))
          outStream << ", ";
      }
      outStream << endl;
    }

    rows = distanceCoefficients.rows;
    columns = distanceCoefficients.cols;


    outStream << endl;

    outStream << "# distortionCoefficients" << endl;

    for (int r = 0; r < rows; r++)
    {
      for (int c = 0; c < columns; c++)
      {
        double value = distanceCoefficients.at<double>(r, c);
        outStream << value;
      }
      if (r != (rows-1))
        outStream << ", ";
    }
    outStream << endl;
    outStream.close();
    return true;
  }

  return false;
}



static void calibrateCameraFromImagesLowMemory(String folderpath, const char* cam, const char* images, size_t im_count)
{
  vector<String> filenames;
  cv::glob(folderpath + "/" + String(images) + "/*.jpg", filenames);
  Mat drawToFrame;

  Mat cameraMatrix = Mat::eye(3, 3, CV_64F);

  Mat distanceCoefficients;

  vector<Mat> savedImages;

  vector<vector<Point2f>> markerCorners, rejectedCandidates;

  //String window = String("image_") + cam;

  vector<vector<Point2f>> checkerboardImageSpacePoints;


  unsigned int count = 0;
  for (size_t i=0; count < im_count && i < filenames.size(); i++)
  {
      Mat frame = imread(filenames[i]);

      bool  found = getImageChessboardCorners(frame, checkerboardImageSpacePoints, false);

      if (found) {
        count++;
      }

      cout << "Camera: " << cam << "; total: " << i << "; has corners: " << count << endl;
      cout.flush();
      //waitKey(1);
  }

  //cout << "Camera: " << cam << "; Calibrating on " << count << " images. " << filenames[i] << endl;
  cameaCalibrationLowMemory(checkerboardImageSpacePoints, chessboardDimentions, calibrationSquareDimention, cameraMatrix, distanceCoefficients);
  String file = (folderpath + "/" + String(cam) + "_" + String(images) + "_intrinsicParametersFile.txt");
  cout << "File: " << file << endl;
  saveCameraCalibration(file , cameraMatrix, distanceCoefficients, count);
}
//
//static void calibrateCameraFromImages(String folderpath, const char* cam, const char* images, size_t im_count)
//{
//  vector<String> filenames;
//  cv::glob(folderpath + String(images) + "/*.jpg", filenames);
//  Mat drawToFrame;
//
//  Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
//
//  Mat distanceCoefficients;
//
//  vector<Mat> savedImages;
//
//  vector<vector<Point2f>> markerCorners, rejectedCandidates;
//
//  //String window = String("image_") + cam;
//
//  unsigned int count = 0;
//  for (size_t i=0; count < im_count && i < filenames.size(); i++)
//  {
//      Mat frame = imread(filenames[i]);
//
//      bool found = true;
//      vector<Vec2f> foundPoints;
//
//      found = findChessboardCorners(frame, chessboardDimentions, foundPoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
//
//      //frame.copyTo(drawToFrame);
//
//      //drawChessboardCorners(drawToFrame, chessboardDimentions, foundPoints, found);
//
//      if (found)
//      {
//        Mat temp;
//        //imshow(window, frame);
//        frame.copyTo(temp);
//        savedImages.push_back(temp);
//        count++;
//      }
//      else
//      {
//        frame.deallocate();
//        //imshow(window, frame);
//      }
//
//      cout << "Camera: " << cam << "; total: " << i << "; has corners: " << count << endl;
//      cout.flush();
//      //waitKey(1);
//  }
//
//  cout << "Camera: " << cam << "; Calibrating on " << count << " images." << endl;
//  cameaCalibration(savedImages, chessboardDimentions, calibrationSquareDimention, cameraMatrix, distanceCoefficients);
//  cout << "File: " << (folderpath + String(images) + "_intrinsicParametersFile.txt") << endl;
//  saveCameraCalibration(folderpath + String(images) + cam + "_intrinsicParametersFile.txt", cameraMatrix, distanceCoefficients, count);
//}
VideoCapture   m_cap;
// main6
int main(int argc, char** argv)
{
  const char *images = "/im1/";
  const char *cam = "40";
  const char *folder = "C:/Work/Ford/Localization_Project/Calibration_images/";
  int im_count = 300;

  if (argc == 2) {
    folder = argv[1];
  }

  if (argc == 3) {
    folder = argv[1];
    cam = argv[2];
  }

  if (argc == 4) {
    folder = argv[1];
    cam = argv[2];
    images = argv[3];
  }

  if (argc == 5) {
    folder = argv[1];
    cam = argv[2];
    images = argv[3];
    im_count = atoi(argv[4]);
  }

  cout << "Images: " << images << endl;

  calibrateCameraFromImagesLowMemory(String(folder) + "/" + cam , cam, images, im_count);
  //calibrateCameraFromImages(String(folder) + cam , cam, images, im_count);


  //  calibrateCameraFromImages(String("C:/Work/Ford/Localization_Project/Calibration_images/") + "13" + String("/im/"), "13", 50);
  //  calibrateCameraFromImages(String("C:/Work/Ford/Localization_Project/Calibration_images/") + "13" + String("/im_no_filter/"), "13", 50);
//  calibrateCameraFromImages("C:/git/vehicle_localization_python/demo2/4/im/", 4);
}

