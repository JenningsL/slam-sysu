#include <octomap/octomap.h>
#include "nabo/nabo.h"
using namespace octomap;
using namespace std;
using namespace Nabo;
using namespace Eigen;

NNSearchF* nns;
/**
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
  const int K = size - 1;
  VectorXi indices(K);
  VectorXf dists2(K);
  nns->knn(q, indices, dists2, K, 0, 0, epsilon);
  for (int i = 0; i < indices.size(); i++) {
    if (indices(i)) {
      nearPoints.push_back(indices(i));
    }
  }

  return nearPoints.size();
}

void expandCluster(int *&cluster_nos, const Pointcloud dataset, bool *visited, int cluster_no, std::list<int> sphere_points, int centerIdx, float epsilon, int min_points) {
  cluster_nos[centerIdx] = cluster_no;
  while (!sphere_points.empty()) {
    int idx = sphere_points.front();
    sphere_points.pop_front();
    if (!visited[idx]) {
      visited[idx] = 1;
      std::list<int> nearPoints;
      int nearPointsNum = regionQuery(epsilon, idx, visited, dataset, nearPoints);
      // cout << nearPointsNum << " ";
      if (nearPointsNum >= min_points) {
        // concatenate two vector
        sphere_points.insert(sphere_points.begin(), nearPoints.begin(), nearPoints.end());
        // add this point to this cluster
        cluster_nos[idx] = cluster_no;
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
	int next_cluster = 1;
  int DATASET_SIZE = dataset.size();
  bool *visited = new bool[DATASET_SIZE];
  int *cluster_nos = new int[DATASET_SIZE];

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

	for(int i = 0; i < DATASET_SIZE; i++)
	{
		if(!visited[i])
		{
			visited[i] = 1;

      std::list<int> nearPoints;
			int num_npoints = regionQuery(epsilon, i, visited, dataset, nearPoints);
			if(num_npoints > min_points) {
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
