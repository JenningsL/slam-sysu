SET (octomath_SRCS
  Vector3.cpp
  Quaternion.cpp
  Pose6D.cpp
 )

include_directories(include)

ADD_LIBRARY( octomath ${octomath_SRCS})

 