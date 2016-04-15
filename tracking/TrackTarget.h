#ifndef TRACKTARGET_H
#define TRACKTARGET_H
#include <octomap/octomap.h>
#include <Eigen/Dense>

using namespace octomap;
using namespace octomath;
using namespace Eigen;
using namespace std;

class TrackTarget {
public:
  vector<Pointcloud> frames;
  vector<point3d> trajectory;
  Matrix<float, 4, 1> state;
  float massZ;

  TrackTarget(Pointcloud cluster) {
    F << 1, 0, 1, 0,
         0, 1, 0, 1,
         0, 0, 1, 0,
         0, 0, 0, 1;
    P << 0, 0, 0, 0,
         0, 0, 0, 0,
         0, 0, 1000, 0,
         0, 0, 0, 1000;
    H << 1, 0, 0, 0,
         0, 1, 0, 0;
    R.setIdentity();
    Q.setIdentity();
    Q *= 0.1;

    point3d mass = getMass(cluster);
    frames.push_back(cluster);
    massZ = mass.z();
    state << mass.x(),
             mass.y(),
             0,
             0;
    trajectory.push_back(point3d(state(0), state(1), massZ));
  };

  Pointcloud predict();

  void update(Pointcloud);

  static point3d getMass(Pointcloud cluster) {
    float x = 0, y = 0, z = 0;
    int n = cluster.size();
    for (int i = 0; i < n; i++) {
      x += cluster[i].x();
      y += cluster[i].y();
      z += cluster[i].z();
    }
    return point3d(x/n, y/n, z/n);
  };

  float getAverageSpeed();
private:
  Matrix4f F;
  Matrix4f P;
  Matrix4f Q;
  Matrix2f R;
  Matrix<float, 2, 4> H;
};

#endif
