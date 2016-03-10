/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <stdio.h>

using namespace std;
using namespace octomap;

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

void print_query_info(point3d query, OcTreeNode* node) {
  if (node != NULL) {
    cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
  }
  else
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;
}

int main(int argc, char** argv) {

  cout << endl;
  cout << "generating example map" << endl;

  OcTree tree (0.1);  // create empty tree with resolution 0.1


  // // insert some measurements of occupied cells
  //
  // for (int x=-20; x<20; x++) {
  //   for (int y=-20; y<20; y++) {
  //     for (int z=-20; z<20; z++) {
  //       point3d endpoint ((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
  //       tree.updateNode(endpoint, true); // integrate 'occupied' measurement
  //     }
  //   }
  // }
  //
  // // insert some measurements of free cells
  //
  // for (int x=-30; x<30; x++) {
  //   for (int y=-30; y<30; y++) {
  //     for (int z=-30; z<30; z++) {
  //       point3d endpoint ((float) x*0.02f-1.0f, (float) y*0.02f-1.0f, (float) z*0.02f-1.0f);
  //       tree.updateNode(endpoint, false);  // integrate 'free' measurement
  //     }
  //   }
  // }

  // cout << endl;
  // cout << "performing some queries:" << endl;
  //
  // point3d query (0., 0., 0.);
  // OcTreeNode* result = tree.search (query);
  // print_query_info(query, result);
  //
  // query = point3d(-1.,-1.,-1.);
  // result = tree.search (query);
  // print_query_info(query, result);
  //
  // query = point3d(1.,1.,1.);
  // result = tree.search (query);
  // print_query_info(query, result);
  //
  //
  // cout << endl;
  // tree.writeBinary("simple_tree.bt");
  // cout << "wrote example file simple_tree.bt" << endl << endl;
  // cout << "now you can use octovis to visualize: octovis simple_tree.bt"  << endl;
  // cout << "Hint: hit 'F'-key in viewer to see the freespace" << endl  << endl;

  // Pointcloud Q, P;
  // Q = readPointCloud("dynamic_segment (Frame 0015).csv");
  // tree.insertPointCloud(Q, point3d(0,0,0));
  // P = readPointCloud("dynamic_segment (Frame 0020).csv");
  // ofstream out;
  // out.open("result.csv");
  // for (Pointcloud::iterator it = P.begin(); it != P.end(); it++) {
  //   OcTreeNode* node = tree.search (*it);
  //   if (node != NULL && node->getOccupancy() < 0.5) {
  //     out << (*it).x() <<","<<(*it).y()<<","<<(*it).z() << endl;
  //   }
  // }
  // out.close();

  Pointcloud P;
  char file[50];
  for (int i = 1; i < 100; i ++) {
    sprintf(file, "./data/dynamic_segment (Frame %04d).csv", i);
    P = readPointCloud(file);
    tree.insertPointCloud(P, point3d(0,0,0));
    cout << "inserted" << file << endl;
  }
  tree.writeBinary("simple_tree.bt");
  cout << "wrote example file simple_tree.bt" << endl << endl;
}
