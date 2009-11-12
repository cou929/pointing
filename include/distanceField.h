#ifndef __DISTANCE_FIELD_H_
#define __DISTANCE_FIELD_H_

#include <map>
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
  std::map <double, CvPoint> distances;
  double calcDistance(CvPoint3D32f a, CvPoint3D32f b) { return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z)); }
  int adjustDistImgRange(double nearest, double farthest);
  int isValidCoord(CvPoint3D32f p) {
    if (p.x != -1 && p.y != -1 && p.z != -1)
      return 1;
    else
      return 0;
  }

 public:
  distanceField(cameraImages * c);
  ~distanceField() { cvReleaseImage(&field); }
  IplImage * calculate(CvPoint origin = cvPoint(0, 0));
  IplImage * getVisibleImage() { return vis; }
  int setMask(IplImage * m) { cvCopy(m, mask); return 0; }
  std::map <double, CvPoint> getDistances() { return distances; }
};

}

#endif
