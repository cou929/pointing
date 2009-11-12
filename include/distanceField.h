#ifndef __DISTANCE_FIELD_H_
#define __DISTANCE_FIELD_H_

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include "cameraImages.h"

namespace point
{

class distanceField
{
 private:
  cameraImages * ci;
  IplImage * field;
  IplImage * mask;
  IplImage * vis;
  CvPoint origin;
  int depth;
  double calcDistance(CvPoint3D32f a, CvPoint3D32f b) { return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z)); }

 public:
  distanceField(cameraImages * c);
  ~distanceField() { cvReleaseImage(&field); }
  IplImage * calculate(CvPoint origin = cvPoint(0, 0));
  IplImage * getVisibleImage() { return vis; }
  int setMask(IplImage * m) { cvCopy(m, mask); return 0; }
};

}

#endif
