#ifndef TRACKMANAGER_H
#define TRACKMANAGER_H

#include "assignment.h"
#include "TrackTarget.h"
#include <octomap/octomap.h>
#include <Eigen/Dense>
#include <algorithm>
#include <math.h>
#include <fstream>
#include <stdlib.h>

using namespace octomap;
using namespace Eigen;
using namespace std;

class TrackManager {
public:
  vector<TrackTarget> targets;

  vector<TrackTarget> disappearedTargets;

  TrackManager(std::vector<Pointcloud> clusters);

  TrackManager();

  void update(std::vector<Pointcloud> clusters);

  void addTarget(Pointcloud cluster);

  void associate(int *&assignment, std::vector<Pointcloud> clusters);

  float calcScore(Pointcloud p1, Pointcloud p2);

  void saveTargets();

};

#endif
