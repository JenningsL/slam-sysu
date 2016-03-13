#include <octomap/octomap.h>
using namespace octomap;
using namespace std;

// TODO: use kd-tree
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
  for (int i = 0; i < size; i++) {
    if (visited[i]) continue;
    // cout << "dist: " << cPoint.distance(dataset[i]) << "epsilon: " << epsilon << endl;
    if (cPoint.distance(dataset[i]) < epsilon) {
      nearPoints.push_back(i);
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

  for (int i = 0; i < DATASET_SIZE; i++) {
    visited[i] = 0;
    cluster_nos[i] = 0;
  }

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
