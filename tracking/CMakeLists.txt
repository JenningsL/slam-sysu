aux_source_directory(. tracking_SRCS)

ADD_LIBRARY(tracking ${tracking_SRCS})

target_link_libraries(tracking octomap)
