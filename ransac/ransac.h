#include <octomap/octomap.h>
#include <Eigen/Dense>

using namespace octomap;
using namespace std;

struct PlaneParam {
  double a;
  double b;
  double c;
  double d;
};

bool randomCompare(int a,int b);

float calcErr(Pointcloud p, PlaneParam param);

PlaneParam getPlane(point3d p1,point3d p2,point3d p3);

double dis2Plane(point3d pt, PlaneParam pl);

PlaneParam leastSquareFit(Pointcloud points);

// void ransacFitPlane(Pointcloud p);
bool* ransacFitPlane(Pointcloud p, float THROTTLE, int MIN_INLIERS, int MAX_ITER);
