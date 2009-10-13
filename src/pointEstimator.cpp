//
// Detect direction of pointing gesture by human, and project mark.
//

#include <cstdio>
#include <cstdlib>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include "libusbSR.h"
#include "definesSR.h"

#include "pointing.h"
#include "pointProjector.h"
#include "coordinateShifter.h"

using namespace std;
using namespace point;
using namespace prj;
using namespace cor;

int main(void)
{
  cameraImages *ci = new cameraImages();
  line3DCv *pointingLine = new line3DCv();

  coordinateShifter *cs = new coordinateShifter();
  double Tx, Ty, Tz, Rx, Ry, Rz, F;
  FILE *fp;
  char *filename = "/home/2007/moriyama/projects/pointing/bin/param.txt";
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
  cvNamedWindow("thinning", CV_WINDOW_AUTOSIZE);
  cvMoveWindow ("Depth",  windowOrigin.x, windowOrigin.y);
  cvMoveWindow ("Intensity",  windowOrigin.x, windowOrigin.y + 200);
  cvMoveWindow ("Result",  windowOrigin.x + 200, windowOrigin.y);
  cvMoveWindow ("centroid",  windowOrigin.x + 200, windowOrigin.y + 200);
  cvMoveWindow ("thinning",  windowOrigin.x + 200, windowOrigin.y + 400);

  // set callback funtions
  cvSetMouseCallback ("Depth", on_mouse_getDepth, ci);
  cvSetMouseCallback ("Intensity", on_mouse_pointing, ci->getDepthImg());

  int counter = 1;
  while(1)
    {
      // acquire current frame
      ci->acquire();

      // track human region
      res = human->track();

      if(res == 0)
	{
	  // get human region
	  img = human->getResult();

	  CvSize imgSize = ci->getImageSize();

	  char k = cvWaitKey(10);
	  if (k == 'c')
	    {
	      printf("\n\n%02d: coordinates\n", counter);
	      for (int i=0; i<imgSize.height; i++)
		for (int j=0; j<imgSize.width; j++)
		  {
		    CvScalar s;
		    s = cvGet2D(img, i, j);
		    if (s.val[0] != 0)
		      {
			CvPoint3D32f tmp = ci->getCoordinate(i, j);
			printf("%f\t%f\t%f\t%d\t%d\n", tmp.x, tmp.y, tmp.z, j, i);
		      }
		  }

	      char intIamgeName[50], depthImageName[50];
	      sprintf(intIamgeName, "%02d_intensity.png", counter);
	      sprintf(depthImageName, "%02d_depth.png", counter);
	      cvSaveImage(intIamgeName, ci->getIntensityImg());
	      cvSaveImage(depthImageName, ci->getDepthImg());
	      counter++;
	    }
	}

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
  cvDestroyWindow("thinning");
  cvDestroyWindow("centroid");
  delete ci;
  delete pointingLine;
  delete cs;
  delete prj;
  delete human;

  return 0;
}
