#ifndef __IMAGE_CLASSIFIER_H_
#define __IMAGE_CLASSIFIER_H_

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

class imageClassifier
{
  // take binary image, classify regions (white region in image), and output a specify region

private:
  const char NONE;
  const char LEFT;
  const char RIGHT;
  const char TOP;
  const char BOTTOM;
  int WHITE;
  int LABEL_MAX;
  IplImage *classImg;
  int label;

  int classify(IplImage *src);
  // Classify 'src' image by putting "labels" (1 to 0xff-1 (8bit image case) value, inclusive) to region in IplImage format.

  int labelling(int x, int y, int direction);
  // Traverse white pixels and labell

  int labelIncrement();
  // Increment label number.
  // If 'label' is larger than 0xff, print error message and return NULL. (this is 8bit image case).

  IplImage* getRegionByLabel(int labelNumber);
  // return region image which labelled 'labelNumber'

  IplImage* getRegionByCoordinate(int x, int y);
  // return region image which contain the (x, y) coordinate pixel

public:
  imageClassifier();
  ~imageClassifier();
  IplImage* getRegionImg(IplImage *src, int x, int y);
  // Take binary image, classify sorce image's white regions separated by black pixels,
  // and return the region which contain pixel (x, y).
};

}

#endif
