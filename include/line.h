#ifndef __LINE_H_
#define __LINE_H_

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

template <typename T> class line
{
protected:
  T point;

public:
  T directionVecror;
  virtual ~line(void){};
  T getPoint(void) {return point;}
  T getDirectionVector(void) {return directionVecror;}
  virtual int setLine(T point1, T point2) = 0;
};

template <typename T3D, typename Tscolar> class line3D : public line<T3D>
{
public:
  virtual T3D getLinePointByZ(Tscolar v) = 0;
};

class line3DCv : public line3D <CvPoint3D32f, double>
{
private:
  int flg;
public:
  line3DCv(void);

  int init(void);
  // Initialize member variables. All values are setted to -1.

  int setLine(CvPoint3D32f point1, CvPoint3D32f point2);
  // Attributes 'point1' and 'point2' is the point which is on a straight line on 3D space.
  // Calcurate direction vector from two point and set it to member variable.

  CvPoint3D32f getLinePointByZ(double v);
  // Return a coordinate which is on the straight line and it's Z value is equal to attribute 'v'.

  bool isValid(void);
  // Validate the line.
};

}

#endif
