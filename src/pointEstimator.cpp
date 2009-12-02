#include <cstdio>
#include <cstdlib>
#include <cfloat>
#include <vector>
#include <utility>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include "cameraImages.h"
#include "regionTracker.h"
#include "faceDetector.h"
#include "funcPointing.h"
#include "pointProjector.h"
#include "coordinateShifter.h"
#include "distanceField.h"

using namespace point;
using namespace prj;
using namespace cor;

int main(void) {
  cameraImages *ci = new cameraImages();
  line3DCv *pointingLine = new line3DCv();

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

  ci->initialize();
  regionTracker *human = new regionTracker(ci);
  img = cvCreateImage(ci->getImageSize(), IPL_DEPTH_8U, 1);

  // prepare coordinate shifter
  if((fp = fopen(filename, "r")) == NULL) {
    fprintf(stderr, "ERROR: cannot open parameter\n");
    return -1;
  }
  if(fgets(line, 1000, fp) != NULL)
    if((res = sscanf(line, "(#f(%lf %lf %lf) #f(%lf %lf %lf) %lf)\n", &Tx, &Ty, &Tz, &Rx, &Ry, &Rz, &F)) < 1) {
      fprintf(stderr, "ERROR: cannot read parameter\nres: %d\n", res);
      return -1;
    }
  fclose(fp);
  cs->setParameter(Tx, Ty, Tz, Rx, Ry, Rz, F);

  CvPoint windowOrigin = {10, 10};
  cvNamedWindow("Depth", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("Intensity", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("Result", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("centroid", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("human", CV_WINDOW_AUTOSIZE);
  cvMoveWindow ("Depth",  windowOrigin.x, windowOrigin.y);
  cvMoveWindow ("Intensity",  windowOrigin.x, windowOrigin.y + 200);
  cvMoveWindow ("Result",  windowOrigin.x + 200, windowOrigin.y);
  cvMoveWindow ("centroid",  windowOrigin.x + 200, windowOrigin.y + 200);
  cvMoveWindow ("human",  windowOrigin.x + 200, windowOrigin.y + 400);

  cvSetMouseCallback ("Depth", on_mouse_getDepth, ci);
  cvSetMouseCallback ("Intensity", on_mouse_pointing, ci->getDepthImg());

  faceDetector * fd = new faceDetector(ci->getImageSize());
  IplImage *color = cvCreateImage(ci->getImageSize(), IPL_DEPTH_8U, 3);
  IplImage * distanceImg;
  CvPoint center;
  CvPoint3D32f face;
  int radius;
  int numFar = 1000;
  distanceField * distField = new distanceField(ci);
  std::vector <std::vector <int> > distances;
  cvNamedWindow("confidenceMap", 0);
  cvNamedWindow("colorwin", 0);
  cvNamedWindow("distanceField", 0);

  while(1) {
    ci->acquire();

    cvCvtColor(ci->getIntensityImg(), color, CV_GRAY2BGR);
    fd->faceDetect(ci->getIntensityImg(), &center, &radius);
    face = ci->getCoordinate(center);
    res = human->track();

    if (res == 0) {
      // calculate distance field
      distField->setMask(human->getResult());
      distField->calculate(center);
      distanceImg = distField->getVisibleImage();

      // draw green circles on top numFar-th pixels which are far from face
      distances = distField->getDistances();
      int loopcount = std::min(numFar, (int)distances.size());
      for (int i=0; i<loopcount; i++)
        {
          int greenDepth = 255 - (int)(((double)255/(double)loopcount) * (double)i);
          cvCircle(color, cvPoint(distances[i][1], distances[i][2]), 1, CV_RGB(0, greenDepth, 0));
        }

      // draw circle on face
      cvCircle(color, center, radius, CV_RGB(127, 127, 255));

      cvShowImage("distanceField", distanceImg);
      cvShowImage("colorwin", color);
      cvShowImage("human", human->getResult());

      key = cvWaitKey(10);
      if (key == 'q')
	break;
    }
  }

  cvShowImage("confidenceMap", ci->getConfidenceMap());

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
