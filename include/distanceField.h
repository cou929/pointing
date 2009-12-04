#ifndef __DISTANCE_FIELD_H_
#define __DISTANCE_FIELD_H_

#include <vector>
#include <queue>
#include <algorithm>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include "cameraImages.h"

namespace point
{

class distanceField
{
 private:
  cameraImages *ci;
  IplImage *field;
  IplImage *mask;
  IplImage *distance;
  IplImage *path_count;
  CvPoint origin;
  int depth;
  std::vector <std::vector <int> > distances;
  typedef std::vector <int> node;

  double calcDistance(CvPoint3D32f a, CvPoint3D32f b) { return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z)); }

  int adjustDistImgRange(double nearest, double farthest);
  // Convert range of distance value for adjusting image depth
  // to view the result visibly

  int countPathes(CvPoint origin, int *come_from);

  int isValidCoord(CvPoint3D32f p) {
    if (p.x != -1 && p.y != -1 && p.z != -1)
      return 1;
    else
      return 0;
  }

  bool isInRange(int x, int y) {
    if (0 <= x && x < ci->getImageSize().height && 0 <= y && y < ci->getImageSize().width)
      return true;
    return false;
  }

  node make_node(int distance, int x, int y) {
    node tmp(3, 0);
    tmp[0] = distance, tmp[1] = x, tmp[2] = y;
    return tmp;
  }

 public:
  distanceField(cameraImages *c);
  ~distanceField() { cvReleaseImage(&field); }

  IplImage *calculate(CvPoint origin = cvPoint(0, 0));
  // calculate shortest paths for each pixels in the region from origin.
  // using dijkstra algorithm

  IplImage *getDistanceImage() { return distance; }
  // Returns distance image. If certain pixel is farther from origin pixel,
  // the intensity value of the distance image is larger.

  IplImage *getPathCountImage() { return path_count; }
  // Returns path counting iamge, which intensity value is large when
  // more shortest paths passes the pixel.

  int setMask(IplImage *m) { cvCopy(m, mask); return 0; }
  // Set mask iamge for calculation.
  // Take a binary image, and calculation process skips the pixel
  // which value of mask image is equal to zero.

  std::vector <std::vector <int> > & getDistances() { std::sort(distances.rbegin(), distances.rend()); return distances; }
  // Returns 2 dimension array. Each row has exactry 3 elements, 0th element is distance, 1st element is column number and
  // 2nd element is row number.
  // The vector is sorted as descending order.
};

}

#endif
