#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include <fstream>
#include <time.h>
#include <map>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include "icp.h"
#include "dbscan/dbscan.h"

using namespace std;
using namespace octomap;
using namespace Eigen;

#define MAX_RANGE 30

class CSVRow
{
    public:
        std::string const& operator[](std::size_t index) const
        {
            return m_data[index];
        }
        std::size_t size() const
        {
            return m_data.size();
        }
        void readNextRow(std::istream& str)
        {
            std::string         line;
            std::getline(str,line);

            std::stringstream   lineStream(line);
            std::string         cell;

            m_data.clear();
            while(std::getline(lineStream,cell,','))
            {
                m_data.push_back(cell);
            }
        }
    private:
        std::vector<std::string>    m_data;
};

std::istream& operator>>(std::istream& str,CSVRow& data)
{
    data.readNextRow(str);
    return str;
}

Pointcloud readPointCloud(const char* filename) {
  Pointcloud Q;
  std::ifstream in(filename);
  CSVRow row;
  while(in >> row)
  {
    float x, y, z;
    x = atof(row[0].c_str());
    y = atof(row[1].c_str());
    z = atof(row[2].c_str());
    Q.push_back(x, y, z);
  }
  return Q;
}

void initMap(ColorOcTree &tree, Pointcloud P) {
  tree.insertPointCloud(P, point3d(0,0,0), MAX_RANGE, true); // maxrange

  for (Pointcloud::iterator it = P.begin(); it != P.end(); it++) {
    tree.setNodeColor((*it).x(), (*it).y(), (*it).z(), 0, 0, 255);
  }

  tree.updateInnerOccupancy();
}

Pointcloud dynamicFilter(ColorOcTree &tree, Pointcloud P) {
  typedef std::map<int, Pointcloud> MAP;
  typedef std::pair<int, Pointcloud> PAIR;
  Pointcloud dcs; // dynamic candidates
  Pointcloud stationary;
  for (Pointcloud::iterator it = P.begin(); it != P.end(); it++) {
    point3d endPoint((*it).x(), (*it).y(), (*it).z());
    OcTreeNode* node = tree.search (endPoint);
    if (node != NULL && node->getOccupancy() < 0.5) {
      dcs.push_back(endPoint);
    } else {
      stationary.push_back(endPoint);
    }
  }
  int size = dcs.size();
  int* clusters_idxs = new int[size];
  clusters_idxs = dbscan(dcs, 10, 1); // -min_points -epsilon
  MAP clusterMap;
  for (int i = 0; i < size; i++) {
    int cluster_idx = clusters_idxs[i];
    if (cluster_idx == 0) continue; // noise
    MAP::iterator it = clusterMap.find(cluster_idx);
    float x = dcs[i].x();
    float y = dcs[i].y();
    float z = dcs[i].z();
    if (it != clusterMap.end()) {
      (it->second).push_back(point3d(x, y, z));
    } else {
      Pointcloud v;
      v.push_back(point3d(x, y, z));
      clusterMap.insert(PAIR(cluster_idx, v));
    }
  }

  for (MAP::iterator it = clusterMap.begin(); it != clusterMap.end(); it++) {
    Pointcloud cluster = it->second;
    cout<< "cluster-" << it->first << " has " << cluster.size() << " points" << endl;
    if (cluster.size() < 100) {
      stationary.push_back(cluster);
    } else {
      // clear points
      Pointcloud tmp(P);
      point3d lowerBound, upperBound;
      cluster.calcBBX(lowerBound, upperBound);
      lowerBound -= point3d(0.5, 0.5, 0.5);
      upperBound += point3d(0.5, 0.5, 0.5);
      tmp.crop(lowerBound, upperBound);
      for (Pointcloud::iterator it = tmp.begin(); it != tmp.end(); it++) {
        tree.updateNode((*it), false);
        // tree.setNodeColor((*it).x(), (*it).y(), (*it).z(), 255, 0, 0);
      }
    }
  }

  return stationary;
}

void updateMap(ColorOcTree &tree, Pointcloud P, Pointcloud lastP) {

  P = icp(lastP, P);
  // P = dynamicFilter(tree, P);
  long beginTime = clock();
  tree.insertPointCloud(P, point3d(0, 0, 0), MAX_RANGE);
  for (Pointcloud::iterator it = P.begin(); it != P.end(); it++) {
    tree.setNodeColor((*it).x(), (*it).y(), (*it).z(), 0, 0, 255);
  } // color
  long endTime = clock();
  cout << "consume time : " << (endTime - beginTime) / 1000000 << "s" <<endl;

}

int main(int argc, char** argv) {
  ColorOcTree tree (0.1);  // create empty tree with resolution 0.1
  int from  = atoi(argv[1]);
  int to = atoi(argv[2]);
  int step;
  if (argc == 3) {
    step = 1;
  } else {
    step = atoi(argv[3]);
  }

  // init
  char baseFile[50];
  sprintf(baseFile, "./data/dynamic_segment (Frame %04d).csv", from);
  Pointcloud base = readPointCloud(baseFile);
  initMap(tree, base);

	Pointcloud P, lastP;
  lastP = base;
  char file[50];
  for (int i = from + 1; i <= to; i += step) {
    sprintf(file, "./data/dynamic_segment (Frame %04d).csv", i);
    P = readPointCloud(file);
    updateMap(tree, P, lastP);
    lastP = P;
  }

  string result = "map.bt";
  tree.write(result);
  cout << "wrote example file " << result << endl;

  return 0;
}
