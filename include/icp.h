#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include "boost/filesystem.hpp"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

using namespace std;
using namespace Eigen;
using namespace octomap;
// using namespace PointMatcherSupport;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PointMatcher<float>::DataPoints::Label Label;
typedef PointMatcher<float>::DataPoints::Labels Labels;
typedef PM::TransformationParameters TransformMatrix;


Matrix<float, 4, Dynamic> getFeaturesMatrix(Pointcloud P);

Pointcloud icp(Pointcloud Q, Pointcloud P, TransformMatrix & Tacc);
// Pointcloud icp(Pointcloud Q, Pointcloud P);
