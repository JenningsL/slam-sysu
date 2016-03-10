#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include <fstream>
#include <time.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include "icp.h"

using namespace std;
using namespace octomap;
using namespace Eigen;

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

int main(int argc, char** argv) {
  OcTree tree (0.1);  // create empty tree with resolution 0.1
  int from  = atoi(argv[1]);
  int to = atoi(argv[2]);
  int step;
  if (argc == 3) {
    step = 1;
  } else {
    step = atoi(argv[3]);
  }
	Pointcloud P, lastP;
  char file[50];
  for (int i = from; i < to; i += step) {
    sprintf(file, "./data/dynamic_segment (Frame %04d).csv", i);
    P = readPointCloud(file);

    if (i > from) {
      P = icp(lastP, P);
    }
    long beginTime = clock();
    tree.insertPointCloud(P, point3d(0,0,0), 30); // maxrange = 30m
    long endTime = clock();
    cout << "inserted" << file << endl;
    cout << "consume time : " << (endTime - beginTime) / 1000000 << "s" <<endl;
    lastP = P;
  }
  tree.writeBinary("simple_tree.bt");
  cout << "wrote example file simple_tree.bt" << endl << endl;
}
