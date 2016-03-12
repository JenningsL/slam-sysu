aux_source_directory(. ransac_SRCS)

ADD_LIBRARY( ransac ${ransac_SRCS})

target_link_libraries(ransac octomap)
