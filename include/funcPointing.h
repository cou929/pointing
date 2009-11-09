// functions for pointing direction detection

#ifndef __POINTING_H_
#define __POINTING_H_

#include "cameraImages.h"
#include "line.h"

namespace point
{
#define ptMax		10000   // Max number of points which are contained into list
#define LINE3D_X	1
#define LINE3D_Y	2
#define LINE3D_Z	3
#define WINDOW_PX	40
#define WINDOW_PY	120
#define WINDOW_DX	50
#define WINDOW_DY	260

#define WsumCal(dx,dy,n) \
 pm[n] = dst->imageData[dst->widthStep * (dy) + (dx)]; \
 if (pm[n] == 255) { \
   Wsum++; \
   b1 = n; \
}

#define WsumCal2(dx,dy,n) \
 pm[n] = dst->imageData[dst->widthStep * (dy) + (dx)]; \
 if (pm[n] == 255) { \
   Wsum ++; \
   b1 = calth(n, preth, b1, &min_thdel, &yosouth);	\
}

#define ch_dxdy(delx,dely) \
  dx+=delx; \
  dy+=dely;

int getArmImage(const IplImage *src, IplImage *dst);
int getArmPoints(IplImage *src, CvPoint *dstFinger, CvPoint *dstElbow);
CvPoint detectFingertip(IplImage *body, CvPoint candidate1, CvPoint candidate2);
int countAround(IplImage *img, CvPoint point, int range);
void myThinningInit(CvMat** kpw, CvMat** kpb);
IplImage* myTinning(IplImage* src);
IplImage* myOresen(IplImage* src , CvPoint* Ap);
int calth(int _n, int _preth,int _b1, int* min_thdel_ad, int* yosouth_ad);
CvPoint DecideKakupt(CvPoint _pt[] , int _ptNow);
void on_mouse_pointing(int event, int x, int y, int flags, void *param);
void on_mouse_getDepth(int event, int x, int y, int flags, void *param);
int calcDirectionVector( CvPoint3D32f  _P , CvPoint3D32f _Q , CvPoint3D32f* _d);
CvPoint3D32f calcCoordinateOnPanel(int panelDistance, CvPoint3D32f startPoint, CvPoint3D32f directionVec);
int clipImage(IplImage *img, CvPoint origin, int width, int height, char* fileName);
bool farther(CvPoint3D32f p1, CvPoint3D32f p2);
CvPoint3D32f getIntersectionObjAndLine(line3DCv *line, cameraImages *ci);
CvPoint3D32f calcCoordFromZ(int z, CvPoint3D32f startPoint, CvPoint3D32f directionVec);
double getError(CvPoint3D32f v1, CvPoint3D32f v2);
CvPoint3D32f getMarkCoord(line3DCv *line, cameraImages *ci, double threshold = 100);

}

#endif
