#ifndef __REGION_DETECTOR_H_
#define __REGION_DETECTOR_H_

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

class regionDetector
{
// Detect ragion from depth, intensity, binary or etc images.

private:
  const char NONE;
  const char LEFT;
  const char RIGHT;
  const char TOP;
  const char BOTTOM;
  const int WHITE;
  IplImage *original;
  IplImage *result;
  int threshold;           // threshold of gradient
  IplConvKernel* element;  // for morphology calcuration

  int traverse(int x, int y, int direction);
  // Traverse pixels and then label.
  // When search process reaches the pixel, calcurate gradient value 
  // between current pixel and around of current pixel, 
  // total these gradient values, and then if total of gradient is 
  // smaller than threshold (default 20 or 2000), current pixel is setted as inner region
  // and next searches around pixels,
  // if larger than threshold, current pixel is setted as inner region
  // but not traverse around pixels.

  int calcGradient(CvPoint arg1, CvPoint arg2);
  // Calcurate gradient value, the absolute value of 
  // difference between 'arg1' and 'arg2'

public:
  regionDetector(CvSize size);
  ~regionDetector();

  int getRegion(IplImage *src, int x, int y, IplImage *dst);
  // Take (depth, intensity, binary and so on) image, classify the image by the region.
  // The region is pixels which has almost same value between that pixel and around pixels.
  // And return the region which contain pixel (x, y).

  int setThreshold(int th);
};

}

#endif
