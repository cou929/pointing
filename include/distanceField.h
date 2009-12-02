#ifndef __DISTANCE_FIELD_H_
#define __DISTANCE_FIELD_H_

#include <vector>
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
  std::vector <std::vector <int> > distances;
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

  std::vector <std::vector <int> > & getDistances() { std::sort(distances.rbegin(), distances.rend()); return distances; }
  // Returns 2 dimension array. Each row has exactry 3 elements, 0th element is distance, 1st element is column number and
  // 2nd element is row number.
  // The vector is sorted as descending order.
};

}

#endif
