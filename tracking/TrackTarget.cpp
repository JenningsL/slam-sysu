#include "TrackTarget.h"

// init static member
// Matrix4f TrackTarget::F, TrackTarget::P, TrackTarget::Q;
// Matrix2f TrackTarget::R;
// Matrix<float, 2, 4> H;
// TrackTarget::F << 1, 0, 1, 0,
//                   0, 1, 0, 1,
//                   0, 0, 1, 0,
//                   0, 0, 0, 1;
// TrackTarget::P << 0, 0, 0, 0,
//                   0, 0, 0, 0,
//                   0, 0, 1000, 0,
//                   0, 0, 0, 1000;
// TrackTarget::H << 1, 0, 0, 0,
//                   0, 1, 0, 0;
// TrackTarget::R.setIdentity();
// TrackTarget::Q.setIdentity();
// TrackTarget::Q *= 0.1;


Pointcloud TrackTarget::predict() {
  float lastX = state(0), lastY = state(1);
  state = F * state;
  P = F * P * F.transpose() + Q;

  Pose6D t(state(0) - lastX, state(1) - lastY, 0, 0, 0, 0);
  Pointcloud predictPc(frames.back());
  predictPc.transform(t);

  // return point3d(state(0), state(1), massZ);
  return predictPc;
}

void TrackTarget::update(Pointcloud cluster) {
  point3d mass = TrackTarget::getMass(cluster);
  Matrix<float, 2, 1> measurement, y;
  Matrix<float, 4, 2> K;
  Matrix2f S;
  measurement << mass.x(),
                 mass.y();
  Matrix4f I;
  I.setIdentity();
  y = measurement - H * state;
  S = H * P * H.transpose() + R;
  K = P * H.transpose() * S.inverse();
  state += K * y;
  P = (I - K * H) * P;

  trajectory.push_back(point3d(state(0), state(1), massZ));
  frames.push_back(cluster);
}

float TrackTarget::getAverageSpeed() {
  int n = trajectory.size();
  if (n <= 1) return 0;
  float speed = 0;
  for (int i = 1; i < n; i++) {
    speed += trajectory[i].distanceXY(trajectory[i-1]);
  }
  return speed / (n - 1);
}
