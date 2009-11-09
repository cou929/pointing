#include "line.h"

namespace point {

line3DCv::line3DCv(void)
{
  flg = 0;
}

int line3DCv::setLine(CvPoint3D32f point1, CvPoint3D32f point2)
{
  CvPoint3D32f tmp;
  double z;
  int ret = -1;

  this->init();

  tmp.x = point2.x - point1.x;
  tmp.y = point2.y - point1.y;
  tmp.z = point2.z - point1.z;

  z = sqrt(tmp.x*tmp.x + tmp.y*tmp.y + tmp.z*tmp.z);
  if (z != 0)
    {
      directionVecror.x = tmp.x / z;
      directionVecror.y = tmp.y / z;
      directionVecror.z = tmp.z / z;
      point = point2;
      flg = 1;
      ret = 0;
    }

  return ret;
}

CvPoint3D32f line3DCv::getLinePointByZ(double v)
{
  CvPoint3D32f ret;

  ret.x = -1;
  ret.y = -1;
  ret.z = -1;

  if (flg)
    {
      ret.x = directionVecror.x * ( v - point.z) / directionVecror.z + point.x;
      ret.y = directionVecror.y * ( v - point.z) / directionVecror.z + point.y;
      ret.z = v;
    }

  return ret;
}

int line3DCv::init(void)
{
  point.x = -1;
  point.y = -1;
  point.z = -1;
  directionVecror.x = -1;
  directionVecror.y = -1;
  directionVecror.z = -1;
  flg = 0;

  return 0;
}

bool line3DCv::isValid(void)
{
  bool ret = false;

  if (flg == 1 && directionVecror.x != -1)
    ret = true;

  return ret;
}

}
