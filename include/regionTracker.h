#ifndef __REGION_TRACKER_H_
#define __REGION_TRACKER_H_

#include <iostream>
#include <cstdio>
#include <vector>
#include <algorithm>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include "cameraImages.h"
#include "regionDetector.h"
#include "faceDetector.h"

namespace point
{

class regionTracker
{
  // tracking human in the image sequence

private:
  cameraImages *ci;
  char initializeFlag;
  CvPoint centroid;
  int centroidDepth;
  int area;
  IplImage *result;
  IplImage *contractedResult; // used in centroid calculation process
  IplConvKernel* element;     // for morphology calcuration
  regionDetector *human;
  IplImage *intensity;
  IplImage *depth;
  faceDetector *fd;

  int initialize();
  // take normal intensity image and generate silhouette binary image
  // silhouette is human region in the input image

  int trackRegion();
  // Track the region.
  // Get region by starting centroid point.
  // If depth value of centroid or area value is obviously difference from previous frame's one,
  // set flag 1 (re-initialize) and return -1.

  int calcCentroidAndArea();
  // Calculate controid and area of region

public:
  regionTracker(cameraImages *cam);
  ~regionTracker();

  int track();
  // Controler of regionTracker.

  IplImage* getResult();
  // Return tracking result, the binary image.
};

}

#endif
