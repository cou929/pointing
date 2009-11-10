#ifndef __CAMERA_IMAGES_H_
#define __CAMERA_IMAGES_H_

#include <iostream>
#include <cstdio>
#include <vector>
#include <algorithm>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include "libusbSR.h"
#include "definesSR.h"

namespace point {

class cameraImages
{
  // camera image class
  // contain images capture from sr4000 camera
  // image is OpenCV format
  // present interface to access images, depth value, coordinate of each pixel, confidence of each pixel an so on

 private:
  SRCAM srCam;
  IplImage* cvDepth;
  IplImage* cvInt;
  IplImage* cvConf;
  ImgEntry* imgEntryArr;
  short* X;                   // X coordinates of each pixel
  short* Y;                   // X coordinates of each pixel
  unsigned short int* Z;      // Z coordinates of each pixel
  int ampImgThreshold;
  int width;
  int height;
  int pich;

  int checkCoordinateRange(int column, int row)
  {
    if(column >= 0 && row >= 0 && column < width && row < height)
      return 0;
    else 
      return -1;
  }

 public:
  cameraImages();
  ~cameraImages();

  int initialize();
  // input: none
  // return: 0 if succeed
  // open camera, initialize, allocate memory for IplImages and set pointer
  //
  // TODO:
  // - error handling
  // - contain window making process, if it can be

  int acquire();
  // input: none
  // return: 0 if succeed
  // acquire images from camera to PC, calcurate XYZ coordinate of each pixel and convert amplitude image to IplImage.
  // call this function at beginning of while loop.
  //
  // TODO:
  // - value check

  IplImage* getDepthImg();
  IplImage* getIntensityImg();
  IplImage* getConfidenceMap();
  int getIntensityVal(int x, int y);
  CvPoint3D32f getCoordinate(int x, int y);
  CvPoint3D32f getCoordinate(CvPoint point);
  int getConfidenceVal(int x, int y);
  int setAmpImgThreshold(int th);
  CvSize getImageSize();
};

}

#endif
