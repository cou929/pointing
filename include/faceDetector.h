#ifndef __FACE_DETECTOR_H_
#define __FACE_DETECTOR_H_

#include <iostream>
#include <cstdio>
#include <vector>
#include <algorithm>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include "libusbSR.h"
#include "definesSR.h"

namespace point
{

class faceDetector
{
// class for face detect

private:
  char* cascadeName;
  CvHaarClassifierCascade* cascade;
  CvMemStorage* storage;
  IplImage *smallImg;
  double scale;

public:
  faceDetector(CvSize size);
  ~faceDetector();
  int setCascadeName(char *name);
  int faceDetect( IplImage* img, CvPoint *center, int *radius );
};

}

#endif
