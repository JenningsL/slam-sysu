SET (octomap_SRCS
  AbstractOcTree.cpp
  AbstractOccupancyOcTree.cpp
  Pointcloud.cpp
  ScanGraph.cpp
  CountingOcTree.cpp
  OcTree.cpp
  OcTreeNode.cpp
  OcTreeStamped.cpp
  OcTreeLUT.cpp
  ColorOcTree.cpp
  )

include_directories(include)

ADD_LIBRARY( octomap ${octomap_SRCS})

target_link_libraries(octomap octomath)

