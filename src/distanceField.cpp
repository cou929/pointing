#include "distanceField.h"

namespace point {

distanceField::distanceField(cameraImages *c) {
  ci = c;
  field = cvCreateImage(ci->getImageSize(), IPL_DEPTH_16U, 1);
  mask = cvCreateImage(ci->getImageSize(), IPL_DEPTH_8U, 1);
  distance = cvCreateImage(ci->getImageSize(), IPL_DEPTH_16U, 1);
  path_count = cvCreateImage(ci->getImageSize(), IPL_DEPTH_16U, 1);
  cvSetZero(field);
  cvSet(mask, cvScalarAll(255));
  cvSetZero(distance);
  cvSetZero(path_count);
  origin = cvPoint(0, 0);
  depth = 16;
}

IplImage *distanceField::calculate(CvPoint origin) {
  std::priority_queue <node, std::vector <node>, std::greater<node> > q;
  CvSize size = ci->getImageSize();
  bool visited[size.height][size.width];
  int come_from[size.height][size.width][2];
  int dirx[4] = {0, 1, 0, -1};
  int diry[4] = {-1, 0, 1, 0};
  const int FAR_INF = INT_MAX;
  int nearest = INT_MAX, farthest = 0;

  cvSetZero(field);
  distances.clear();
  memset(visited, false, sizeof(visited));
  memset(come_from, -1, sizeof(come_from));

  if (!isInRange(origin.x, origin.y) || !isValidCoord(ci->getCoordinate(origin)))
    return field;

  for (int row=0; row<size.height; row++)
    for (int col=0; col<size.width; col++)
      distances.push_back(make_node(FAR_INF, row, col));

  q.push(make_node(0, origin.x, origin.y));

  while (!q.empty()) {
    node current = q.top();
    q.pop();

    visited[current[1]][current[2]] = true;

    for (int i=0; i<4; i++) {
      node next = make_node(current[0], current[1] + dirx[i], current[2] + diry[i]);
      CvScalar maskPix = cvGet2D(mask, next[1], next[2]);

      if (isInRange(next[1], next[2]) &&
          isValidCoord(ci->getCoordinate(cvPoint(next[1], next[2]))) &&
          !visited[next[1]][next[2]] &&
          maskPix.val[0] != 0) {
        next[0] = current[0] + (int)calcDistance(ci->getCoordinate(current[1], current[2]), ci->getCoordinate(next[1], next[2]));

	if (distances[next[1]][next[2]] > next[0]) {
	  distances[next[1]][next[2]] = next[0];
	  come_from[next[1]][next[2]][0] = current[1], come_from[next[1]][next[2]][0] = current[2];
	  if (distances[next[1]][next[2]] == FAR_INF)
	    q.push(next);
	}

        nearest = std::min(nearest, next[0]);
        farthest = std::max(farthest, next[0]);
      }
    }
  }

  for (int row=0; row<size.height; row++)
    for (int col=0; col<size.width; col++) {
      int d = 0;
      d = (distances[row][col] == FAR_INF) ? 0 : distances[row][col];
      cvSet2D(field, row, col, cvScalarAll(d));
    }

  adjustDistImgRange(nearest, farthest);

  countPathes(origin, &come_from[0][0][0]);

  return field;
}

int distanceField::adjustDistImgRange(double nearest, double farthest) {
  CvSize size = ci->getImageSize();
  double maxNum = pow((double)2, (double)depth);
  double ratio = maxNum / (farthest - nearest);

  for (int row=0; row<size.height; row++)
    for (int col=0; col<size.width; col++) {
      CvScalar cur = cvGet2D(field, row, col);
      cur = cvScalarAll((cur.val[0] - nearest) * ratio);
      cvSet2D(distance, row, col, cur);
    }

  return 0;
}

int distanceField::countPathes(CvPoint origin, int *come_from) {
  std::vector <std::vector <int> > end_points;
  CvSize size = ci->getImageSize();
  int path_count_array[size.height][size.width];

  memset(path_count_array, 0, sizeof(path_count_array));

  // search end points of each shortest path
  for (int row=0; row<size.height; row++)
    for (int col=0; col<size.width; col++) {
      CvScalar mask_point = cvGet2D(mask, row, col);
      if (mask_point.val[0] != 0 && come_from[row * size.width + col + 0] == -1 && row != origin.x && col != origin.y) {
	std::vector <int> tmp(2, 0);
	tmp[0] = come_from[row * size.width + col + 0];
	tmp[1] = come_from[row * size.width + col + 1];
	end_points.push_back(tmp);
      }
    }

  // for each pixel, count how many path passes this pixel
  for (int i=0; i<end_points.size(); i++) {
    int cur_row = come_from[end_points[i][0] * size.width + end_points[i][1] + 0];
    int cur_col = come_from[end_points[i][0] * size.width + end_points[i][1] + 1];

    while (come_from[cur_row * size.width + cur_col + 0] != -1) {
      path_count_array[cur_row][cur_col]++;
      cur_row = come_from[cur_row * size.width + cur_col + 0];
      cur_col = come_from[cur_row * size.width + cur_col + 1];
    }
  }

  // store calculation results into "path_count" image
  int max_count = 0;
  for (int row=0; row<size.height; row++)
    for (int col=0; col<size.width; col++)
      std::max(max_count, path_count_array[row][col]);

  double ratio = pow((double)2, (double)depth) / max_count;
  for (int row=0; row<size.height; row++)
    for (int col=0; col<size.width; col++)
      cvSet2D(path_count, row, col, cvScalarAll(ratio * path_count_array[row][col]));

  return 0;
}

}
