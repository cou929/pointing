#include <cstdio>
#include <cstdlib>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include "libusbSR.h"
#include "definesSR.h"

#include "cameraImages.h"
#include "regionTracker.h"
#include "faceDetector.h"
#include "funcPointing.h"
#include "pointProjector.h"
#include "coordinateShifter.h"

using namespace point;
using namespace prj;
using namespace cor;

int main(void)
{
  cameraImages *ci = new cameraImages();
  line3DCv *pointingLine = new line3DCv();
  CvPoint fingertip2D, elbow2D, subject2D;
  CvPoint3D32f fingertip3D, elbow3D, subject3D;

  coordinateShifter *cs = new coordinateShifter();
  double Tx, Ty, Tz, Rx, Ry, Rz, F;
  FILE *fp;
  char *filename = "data/param.txt";
  char line[100];

  int width = 1280;
  int height = 1024;
  onePointProjector *prj = new onePointProjector(width, height);

  IplImage *img;
  char key;
  int res;

  // initialize camera image class
  ci->initialize();

  // prepare human region tracker
  regionTracker *human = new regionTracker(ci);

  // prepare IplImage (contain result of process)
  img = cvCreateImage(ci->getImageSize(), IPL_DEPTH_8U, 1);

  // prepare coordinate shifter
  if((fp = fopen(filename, "r")) == NULL)
    {
      fprintf(stderr, "ERROR: cannot open parameter\n");
      return -1;
    }

  if(fgets(line, 1000, fp) != NULL)
    if((res = sscanf(line, "(#f(%lf %lf %lf) #f(%lf %lf %lf) %lf)\n", &Tx, &Ty, &Tz, &Rx, &Ry, &Rz, &F)) < 1)
      {
	fprintf(stderr, "ERROR: cannot read parameter\nres: %d\n", res);
	return -1;
      }
  fclose(fp);

  // set parameter
  cs->setParameter(Tx, Ty, Tz, Rx, Ry, Rz, F);

  // make windows
  CvPoint windowOrigin = {10, 10};
  cvNamedWindow("Depth", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("Intensity", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("Result", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("centroid", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("arm", CV_WINDOW_AUTOSIZE);
  cvMoveWindow ("Depth",  windowOrigin.x, windowOrigin.y);
  cvMoveWindow ("Intensity",  windowOrigin.x, windowOrigin.y + 200);
  cvMoveWindow ("Result",  windowOrigin.x + 200, windowOrigin.y);
  cvMoveWindow ("centroid",  windowOrigin.x + 200, windowOrigin.y + 200);
  cvMoveWindow ("arm",  windowOrigin.x + 200, windowOrigin.y + 400);

  // set callback funtions
  cvSetMouseCallback ("Depth", on_mouse_getDepth, ci);
  cvSetMouseCallback ("Intensity", on_mouse_pointing, ci->getDepthImg());

  faceDetector * fd = new faceDetector(ci->getImageSize());
  IplImage *arm = cvCreateImage(ci->getImageSize(), IPL_DEPTH_8U, 1); // FIX LATER
  CvPoint center;
  int radius;

  while(1)
    {
      // acquire current frame
      ci->acquire();

      /////////////////
      CvPoint3D32f current, face;
      long long maxDistance = 0;
      CvPoint farPoint;

      fd->faceDetect(ci->getIntensityImg(), &center, &radius);
      getArmImage(img, arm);

      face = ci->getCoordinate(center);

      human->track();
      cvShowImage("arm", human->getResult());

      for (int i=0; i<arm->height; i++)
	for (int j=0; j<arm->width; j++)
	  {
	    CvScalar regionChecker = cvGet2D(human->getResult(), i, j);
	    if (regionChecker.val[0] == 255)
	      {
		current = ci->getCoordinate(i, j);

		long long tmp = (current.x - face.x)*(current.x - face.x) + (current.y - face.y)*(current.y - face.y) + (current.z - face.z)*(current.z - face.z);
		if (maxDistance < tmp)
		  {
		    maxDistance = tmp;
		    farPoint = cvPoint(i, j);
		  }
	      }
	  }

      cvCircle(ci->getIntensityImg(), center, 30, CV_RGB(127, 127, 127));
      cvCircle(ci->getIntensityImg(), farPoint, 30, CV_RGB(255, 255, 255));
      cvCircle(ci->getIntensityImg(), farPoint, 5, CV_RGB(255, 255, 255));
      /////////////////////////////

      /*
      // track human region
      res = human->track();

      if(res == 0)
      {
      // get human region
      img = human->getResult();

      ////
      ////

      // get arm points
      getArmPoints(img, &fingertip2D, &elbow2D);

      // get 3D coordinate of arm points
      fingertip3D = ci->getCoordinate(fingertip2D);
      elbow3D = ci->getCoordinate(elbow2D);
      if(fingertip3D.x == -1 || elbow3D.x == -1)
      continue;

      // calculate pointing direction
      pointingLine->setLine(elbow3D, fingertip3D);
      if(!pointingLine->isValid())
      continue;

      // calculate intersection of pointing line and subject object
      subject3D = getMarkCoord(pointingLine, ci);
      if (subject3D.x == -1)
      continue;

      // calcurate coordinate where to projector points
      subject2D = cs->world2img((-1)*subject3D.z, (-1)*subject3D.x, subject3D.y);

      // project mark
      prj->showPoint(subject2D);

      cvShowImage("Result", img);
      }
      */

      // show images
      cvShowImage("Depth", ci->getDepthImg());
      cvShowImage("Intensity", ci->getIntensityImg());

      // key handling
      key = cvWaitKey(10);
      if(key == 'q')
	break;
    }

  // release memory
  cvDestroyWindow("Depth");
  cvDestroyWindow("Intensity");
  cvDestroyWindow("Result");
  cvDestroyWindow("arm");
  cvDestroyWindow("centroid");
  delete ci;
  delete pointingLine;
  delete cs;
  delete prj;
  delete human;

  return 0;
}
