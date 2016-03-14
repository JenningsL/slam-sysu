#include "ransac.h"
#include "math.h"
#include <stdlib.h>
#include <time.h>
#include <algorithm>

using namespace Eigen;

bool *ransacFitPlane(Pointcloud p, float THROTTLE, int MIN_INLIERS, int MAX_ITER) {
  // const int MAX_ITER = 1000;
  // const float THROTTLE = 0.5;
  // const int MIN_INLIERS = 6000;
  PlaneParam bestFit;
  float bestErr = -1;
  Pointcloud bestFitInliers;
  std::vector<int> bestInliersIdx;

  int size = p.size();
  std::vector<int> idx;
  for (int i = 0; i < size; i++) {
    idx.push_back(i);
  }

  int iter;
  for (iter = 0; iter < MAX_ITER; iter++) {
    random_shuffle(idx.begin(), idx.end());
    // randomly select 3 points
    PlaneParam hyp_plane = getPlane(p[idx[0]], p[idx[1]], p[idx[2]]);
    // find inliers
    Pointcloud inliers;
    std::vector<int> inliersIdx;
    inliers.push_back(p[idx[0]]);
    inliers.push_back(p[idx[1]]);
    inliers.push_back(p[idx[2]]);
    for (int x = 3; x < size; x++) {
      if (dis2Plane(p[idx[x]], hyp_plane) < THROTTLE) {
        inliers.push_back(p[idx[x]]);
        inliersIdx.push_back(idx[x]);
      }
    }
    if (inliers.size() > MIN_INLIERS) {
      PlaneParam fitPlane = leastSquareFit(inliers);
      float err = calcErr(inliers, fitPlane);
      if (bestErr < 0 || err < bestErr) {
        bestErr = err;
        bestFit = fitPlane;
        bestFitInliers = inliers;
        bestInliersIdx = inliersIdx;
      }
      if (bestErr < 0.5) break;
    }
  }

  // cout<< "bestErr:" << bestErr <<endl;
  // cout<< "iteration:" << iter <<endl;

  // for matlab test
  // ofstream out;
  // out.open("ground.csv");
  // for (Pointcloud::iterator it = bestFitInliers.begin(); it != bestFitInliers.end(); it++ ) {
  //   out << (*it).x() << " " << (*it).y() <<  " " << (*it).z() <<endl;
  // }
  // out.close();

  // generate an array to show inliers(1) and outliers(0)
  bool *io=new bool[size];
  for (int i = 0; i < size; i++) io[i] = 0;
  for (std::vector<int>::iterator it = bestInliersIdx.begin(); it != bestInliersIdx.end(); it++) {
    io[*it] = 1;
  }
  return io;
}



float calcErr(Pointcloud p, PlaneParam param) {
  float err = 0;
  for (Pointcloud::iterator it = p.begin(); it != p.end(); it++) {
    err += dis2Plane(*it, param);
  }
  return err / p.size();
}

PlaneParam getPlane(point3d p1,point3d p2,point3d p3) {
  double a, b, c, d;
  PlaneParam param;
  a = ( (p2.y()-p1.y())*(p3.z()-p1.z())-(p2.z()-p1.z())*(p3.y()-p1.y()) );
  b = ( (p2.z()-p1.z())*(p3.x()-p1.x())-(p2.x()-p1.x())*(p3.z()-p1.z()) );
  c = ( (p2.x()-p1.x())*(p3.y()-p1.y())-(p2.y()-p1.y())*(p3.x()-p1.x()) );
  d = ( 0-(a*p1.x()+b*p1.y()+c*p1.z()) );
  param.a = a;
  param.b = b;
  param.c = c;
  param.d = d;
  return param;
}

double dis2Plane(point3d pt, PlaneParam pl) {
  double a, b, c, d;
  a = pl.a;
  b = pl.b;
  c = pl.c;
  d = pl.d;
  return fabs(a*pt.x()+b*pt.y()+c*pt.z()+d)/sqrt(a*a+b*b+c*c);
}

PlaneParam leastSquareFit(Pointcloud points) {
  Matrix<float, Dynamic, Dynamic> A;
  A.resize(3, 3);
  Matrix<float, Dynamic, Dynamic> b;
  b.resize(3, 1);
  int size = points.size();
  for (int i = 0; i < size; i++) {
    float x = points[i].x();
    float y = points[i].y();
    float z = points[i].z();
    A(0, 0) += x * x;
    A(0, 1) += x * y;
    A(0, 2) += x;
    A(1, 0) += x * y;
    A(1, 1) += y * y;
    A(1, 2) += y;
    A(2, 0) += x;
    A(2, 1) += y;
    A(2, 2) += 1;

    b(0, 0) += x * z;
    b(1, 0) += y * z;
    b(2, 0) += z;
  }
  Matrix<float, 3, 1> p = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
  // cout << "Here is the matrix A:\n" << A << endl;
  // cout << "Here is the right hand side b:\n" << b << endl;
  // cout << "The least-squares solution is:\n"
  //      << p << endl;
  PlaneParam pp;
  pp.a = p(0, 0);
  pp.b = p(1, 0);
  pp.c = -1;
  pp.d = p(2, 0);
  return pp;
}
