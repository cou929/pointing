#include "distanceField.h"

namespace point {

distanceField::distanceField(cameraImages *c) {
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

IplImage *distanceField::calculate(CvPoint origin) {
  std::priority_queue <node, std::vector <node>, std::greater<node> > q;
  CvSize size = ci->getImageSize();
  bool visited[size.height][size.width];
  int dirx[4] = {0, 1, 0, -1};
  int diry[4] = {-1, 0, 1, 0};
  const double FAR_INF = INT_MAX;
  double nearest = DBL_MAX, farthest = 0;

  cvSetZero(field);
  distances.clear();
  memset(visited, false, sizeof(visited));

  if (!isInRange(origin.x, origin.y) || !isValidCoord(ci->getCoordinate(origin)))
    return field;

  for (int row=0; row<size.height; row++)
    for (int col=0; col<size.width; col++)
      distances.push_back(make_node(FAR_INF, row, col));

  q.push(make_node(0, origin.x, origin.y));

  while (!q.empty()) {
    node current = q.top;
    q.pop();

    visited[current[1]][current[2]] = true;

    for (int i=0; i<4; i++) {
      node next = make_node(current[0], current[1] + dirx[i], current[2] + diry[i]);
      CvScalar maskPix = cvGet2D(mask, next[1], next[2]);

      if (isInRange(next[1], next[2]) &&
          isValidCoord(ci->getCoordinate(cvPoint(next[1], next[2]))) &&
          !visited[next[1]][next[2]] &&
          maskPix.val[0] != 0) {
        next[0] = current[0] + calcDistance(current[1], current[2], next[1], next[2]);

        if (distances[next[1]][next[2]] == FAR_INF)
          q.push(next);
        distances[next[1]][next[2]] = std::min(distances[next[1]][next[2]], next[0]);

        nearest = std::min(nearest, next[0]);
        farthest = std::max(farthest, next[0]);
      }
    }
  }

  for (int row=0; row<size.height; row++)
    for (int col=0; col<size.width; col++) {
      int d = 0;
      d = (distances[row][col] == FAR_INF) : 0 ? distances[row][col];
      cvSet2D(field, row, col, cvScalarAll(d));
    }

  adjustDistImgRange(nearest, farthest);

  return field;
}

int distanceField::adjustDistImgRange(double nearest, double farthest) {
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
