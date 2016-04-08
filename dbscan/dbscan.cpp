#include <octomap/octomap.h>
#include "nabo/nabo.h"
#include <algorithm>
#include <cmath>
using namespace octomap;
using namespace std;
using namespace Nabo;
using namespace Eigen;

NNSearchF* nns;
MatrixXi indicesTable;
MatrixXf dists2Table;

int getRequiredMinPointAtDist(int baseMinPoint, float dist) {
  const int baseDist = 10;
  // const int baseMinPoint = 10;
  const int atLeast = 5;
  int density = ceil(baseMinPoint * pow(atan(1/dist), 2) / pow(atan(1/baseDist), 2));
  return std::max(density, atLeast);
}

/**
 * due to performance matter, this function is no longer used
 * find all points within a sphere, store it in @paramnear Points
 * @param  epsilon    [description]
 * @param  centerIdx  [description]
 * @param  visited    [description]
 * @param  dataset    [description]
 * @param  nearPoints [description]
 * @return            [size of the found points]
 */
int regionQuery(float epsilon, int centerIdx, bool *visited, const Pointcloud dataset, std::list<int> & nearPoints) {
  point3d cPoint = dataset[centerIdx];
  int size = dataset.size();
  // 1 query points
  VectorXf q;
  q.resize(3);
  q(0) = cPoint.x();
  q(1) = cPoint.y();
  q(2) = cPoint.z();
  // const int K = 1000;
  // const int K = size - 1;
  const int K = min(50, size - 1);
  VectorXi indices(K);
  VectorXf dists2(K);
  nns->knn(q, indices, dists2, K, 0, 0, epsilon);
  for (int i = 0; i < indices.size(); i++) {
    if (indices(i)) {
      nearPoints.push_back(indices(i));
    }
  }

  return nearPoints.size();

  /*********** new implement ***********/
  // nearPoints.clear();
  // VectorXi indices = indicesTable.col(centerIdx);
  // for (int i = 0; i < indices.rows(); i++) {
  //   if (indices(i)) {
  //     nearPoints.push_back(indices(i));
  //   }
  // }
  //
  // return nearPoints.size();
}

void expandCluster(int *&cluster_nos, const Pointcloud dataset, bool *visited, int cluster_no, std::list<int> sphere_points, int centerIdx, float epsilon, int min_points) {
  cluster_nos[centerIdx] = cluster_no;
  while (!sphere_points.empty()) {
    int idx = sphere_points.front();
    sphere_points.pop_front();
    if (!visited[idx]) {
      visited[idx] = 1;

      std::list<int> nearPoints;
      // int nearPointsNum = regionQuery(epsilon, idx, visited, dataset, nearPoints);
      nearPoints.clear();
      VectorXi indices = indicesTable.col(idx);
      for (int j = 0; j < indices.rows(); j++) {
        if (indices(j)) {
          nearPoints.push_back(indices(j));
        }
      }
      int nearPointsNum = nearPoints.size();
      // cout << nearPointsNum << " ";
      // calculate min_points based on distance
      int min_points_at_dist = getRequiredMinPointAtDist(min_points, dataset[idx].distance(point3d(0, 0, 0)));
      if (nearPointsNum >= min_points_at_dist) {
        // add this point to this cluster
        cluster_nos[idx] = cluster_no;

        // concatenate two vector
        sphere_points.insert(sphere_points.begin(), nearPoints.begin(), nearPoints.end());
      }
    }
  }
}

/**
 * perform dbscan algorithm
 * @param  dataset    [3d point cloud]
 * @param  min_points [min points number within the sphere with radius epsilon]
 * @param  epsilon    [radius]
 * @return            [cluster indexs indicating which cluster the corresponding point belong to]
 */
int* dbscan(const Pointcloud dataset, const int min_points, const float epsilon) {
  const int K = 50;
	int next_cluster = 1;
  int DATASET_SIZE = dataset.size();
  bool *visited = new bool[DATASET_SIZE];
  int *cluster_nos = new int[DATASET_SIZE];

  // not enough points
  if (DATASET_SIZE < K) {
    for (int i = 0; i < DATASET_SIZE; i++) cluster_nos[i] = 0;
    return cluster_nos;
  }

  // init kd-tree nearsearch
  MatrixXf M;
  M.resize(3, DATASET_SIZE);
  for (int i = 0; i < DATASET_SIZE; i++) {
    visited[i] = 0;
    cluster_nos[i] = 0;
    M(0, i) = dataset[i].x();
    M(1, i) = dataset[i].y();
    M(2, i) = dataset[i].z();
  }
  nns = NNSearchF::createKDTreeLinearHeap(M);

  indicesTable.resize(K, M.cols());
  dists2Table.resize(K, M.cols());
  nns->knn(M, indicesTable, dists2Table, K, 0, 0, epsilon);

	for(int i = 0; i < DATASET_SIZE; i++)
	{
		if(!visited[i])
		{
			visited[i] = 1;

      std::list<int> nearPoints;
			// int num_npoints = regionQuery(epsilon, i, visited, dataset, nearPoints);
			// regionQuery
      nearPoints.clear();
      VectorXi indices = indicesTable.col(i);
      for (int j = 0; j < indices.rows(); j++) {
        if (indices(j)) {
          nearPoints.push_back(indices(j));
        }
      }
      int num_npoints = nearPoints.size();
      // calculate min_points based on distance
      int min_points_at_dist = getRequiredMinPointAtDist(min_points, dataset[i].distance(point3d(0, 0, 0)));
			if(num_npoints > min_points_at_dist) {
				expandCluster(cluster_nos, dataset, visited, next_cluster, nearPoints, i, epsilon, min_points);
				next_cluster++;
			} else {
        cluster_nos[i] = 0; // not belong to any cluster
      }
		}
	}

  delete []visited;
  /* save result in file */
  // ofstream out;
  // out.open("cluster.csv");
  // for (int i = 0; i < DATASET_SIZE; i++) {
  //   out << dataset[i].x() << " " << dataset[i].y() << " " << dataset[i].z() << " " << cluster_nos[i] << endl;
  // }
  // out.close();
  return cluster_nos;
}
