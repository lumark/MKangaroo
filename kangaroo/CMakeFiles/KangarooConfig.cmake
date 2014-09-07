# Compute paths
get_filename_component( PROJECT_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH )
SET( Kangaroo_INCLUDE_DIRS "${Kangaroo_CMAKE_DIR}/../../../include;/usr/local/cuda/include;/opt/local/include/eigen3;/usr/local/include;/usr/local/include/opencv2;/usr/local/include/opencv2/core;/usr/local/include/opencv2/imgproc;/usr/local/include/opencv2/features2d;/usr/local/include/opencv2/flann;/usr/local/include/opencv2/calib3d;/usr/local/include/opencv2/objdetect;/usr/local/include/opencv2/legacy;/usr/local/include/opencv2/contrib;/usr/local/include/opencv2/highgui;/usr/local/include/opencv2/ml;/usr/local/include/opencv2/video;/usr/local/include/opencv2/gpu" )

# Library dependencies (contains definitions for IMPORTED targets)
if( NOT TARGET kangaroo AND NOT Kangaroo_BINARY_DIR )
  include( "${PROJECT_CMAKE_DIR}/KangarooTargets.cmake" )
endif()

SET( Kangaroo_LIBRARIES kangaroo )
