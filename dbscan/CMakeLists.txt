aux_source_directory(. dbscan_SRCS)

ADD_LIBRARY( dbscan ${dbscan_SRCS})

target_link_libraries(dbscan octomap)
