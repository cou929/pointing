#include "distanceField.h"

namespace point
{

distanceField::distanceField(cameraImages * c)
{
  ci = c;
  field = cvCreateImage(ci->getImageSize(), IPL_DEPTH_16U, 1);
  mask = cvCreateImage(ci->getImageSize(), IPL_DEPTH_8U, 1);
  vis = cvCreateImage(ci->getImageSize(), IPL_DEPTH_16U, 1);
  cvSetZero(field);
  cvSet(mask, cvScalarAll(255));
  cvSetZero(vis);
  origin = cvPoint(0, 0);
  depth = 16;
}

IplImage * distanceField::calculate(CvPoint origin)
{
  int row, col;
  CvSize size = ci->getImageSize();
  CvPoint3D32f current, origin3d;
  double nearest = DBL_MAX, farthest = 0;

  cvSetZero(field);

  origin3d = ci->getCoordinate(origin);

  if (!isValidCoord(origin3d))
    return field;
  
  for (row=0; row<size.height; row++)
    for (col=0; col<size.width; col++)
      {
	current = ci->getCoordinate(col, row);
	CvScalar maskPix = cvGet2D(mask, row, col);

	if (isValidCoord(current) &&  maskPix.val[0] != 0)
	  {
	    double d = calcDistance(current, origin3d);
	    cvSet2D(field, row, col, cvScalarAll(d));

	    nearest = std::min(nearest, d);
	    farthest = std::max(farthest, d);

	    distances[d] = cvPoint(col, row);
	  }
      }

  adjustDistImgRange(nearest, farthest);

  return field;
}

int distanceField::adjustDistImgRange(double nearest, double farthest)
{
  // Convert range of distance value for adjusting image depth
  // to view the result visibly

  int row, col;
  CvSize size = ci->getImageSize();
  double maxNum = pow((double)2, (double)depth);
  double ratio = maxNum / (farthest - nearest);

  for (row=0; row<size.height; row++)
    for (col=0; col<size.width; col++)
      {
	CvScalar cur = cvGet2D(field, row, col);
	cur = cvScalarAll((cur.val[0] - nearest) * ratio);
	cvSet2D(vis, row, col, cur);
      }

  return 0;
}

}
