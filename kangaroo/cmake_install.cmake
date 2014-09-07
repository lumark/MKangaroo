# Install script for directory: /Users/luma/Code/RobotGroup/MKangaroo/kangaroo

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/Kangaroo/config.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/Kangaroo" TYPE FILE FILES "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/config.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/Kangaroo/BoundedVolume.h;/usr/local/include/Kangaroo/MarchingCubesTables.h;/usr/local/include/Kangaroo/cu_deconvolution.h;/usr/local/include/Kangaroo/cu_painting.h;/usr/local/include/Kangaroo/kangaroo.h;/usr/local/include/Kangaroo/BoundingBox.h;/usr/local/include/Kangaroo/Mat.h;/usr/local/include/Kangaroo/cu_dense_stereo.h;/usr/local/include/Kangaroo/cu_plane_fit.h;/usr/local/include/Kangaroo/launch_utils.h;/usr/local/include/Kangaroo/CostVolElem.h;/usr/local/include/Kangaroo/MatUtils.h;/usr/local/include/Kangaroo/cu_depth_tools.h;/usr/local/include/Kangaroo/cu_raycast.h;/usr/local/include/Kangaroo/patch_score.h;/usr/local/include/Kangaroo/CudaTimer.h;/usr/local/include/Kangaroo/Pyramid.h;/usr/local/include/Kangaroo/cu_heightmap.h;/usr/local/include/Kangaroo/cu_remap.h;/usr/local/include/Kangaroo/pixel_convert.h;/usr/local/include/Kangaroo/Divergence.h;/usr/local/include/Kangaroo/Sdf.h;/usr/local/include/Kangaroo/cu_index_buffer.h;/usr/local/include/Kangaroo/cu_resample.h;/usr/local/include/Kangaroo/reduce.h;/usr/local/include/Kangaroo/Image.h;/usr/local/include/Kangaroo/Volume.h;/usr/local/include/Kangaroo/cu_integral_image.h;/usr/local/include/Kangaroo/cu_rof_denoising.h;/usr/local/include/Kangaroo/reweighting.h;/usr/local/include/Kangaroo/ImageApron.h;/usr/local/include/Kangaroo/cu_anaglyph.h;/usr/local/include/Kangaroo/cu_lookup_warp.h;/usr/local/include/Kangaroo/cu_sdffusion.h;/usr/local/include/Kangaroo/sampling.h;/usr/local/include/Kangaroo/ImageIntrinsics.h;/usr/local/include/Kangaroo/cu_bilateral.h;/usr/local/include/Kangaroo/cu_manhattan.h;/usr/local/include/Kangaroo/cu_segment_test.h;/usr/local/include/Kangaroo/variational.h;/usr/local/include/Kangaroo/ImageKeyframe.h;/usr/local/include/Kangaroo/cu_blur.h;/usr/local/include/Kangaroo/cu_median.h;/usr/local/include/Kangaroo/cu_semi_global_matching.h;/usr/local/include/Kangaroo/InvalidValue.h;/usr/local/include/Kangaroo/cu_census.h;/usr/local/include/Kangaroo/cu_model_refinement.h;/usr/local/include/Kangaroo/cu_tgv.h;/usr/local/include/Kangaroo/LeastSquareSum.h;/usr/local/include/Kangaroo/cu_convert.h;/usr/local/include/Kangaroo/cu_normals.h;/usr/local/include/Kangaroo/disparity.h;/usr/local/include/Kangaroo/cu_convolution.h;/usr/local/include/Kangaroo/cu_operations.h;/usr/local/include/Kangaroo/hamming_distance.h;/usr/local/include/Kangaroo/VolumeGrid.h;/usr/local/include/Kangaroo/cu_raycast_grid.h;/usr/local/include/Kangaroo/cu_sdffusion_grid.h;/usr/local/include/Kangaroo/cu_rolling_sdf.h;/usr/local/include/Kangaroo/BoundedVolumeGrid.h;/usr/local/include/Kangaroo/RollingGridSDF.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/Kangaroo" TYPE FILE FILES
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/BoundedVolume.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/MarchingCubesTables.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_deconvolution.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_painting.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/kangaroo.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/BoundingBox.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/Mat.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_dense_stereo.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_plane_fit.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/launch_utils.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/CostVolElem.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/MatUtils.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_depth_tools.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_raycast.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/patch_score.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/CudaTimer.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/Pyramid.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_heightmap.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_remap.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/pixel_convert.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/Divergence.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/Sdf.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_index_buffer.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_resample.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/reduce.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/Image.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/Volume.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_integral_image.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_rof_denoising.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/reweighting.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/ImageApron.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_anaglyph.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_lookup_warp.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_sdffusion.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/sampling.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/ImageIntrinsics.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_bilateral.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_manhattan.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_segment_test.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/variational.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/ImageKeyframe.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_blur.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_median.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_semi_global_matching.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/InvalidValue.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_census.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_model_refinement.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_tgv.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/LeastSquareSum.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_convert.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_normals.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/disparity.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_convolution.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/cu_operations.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/hamming_distance.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/GridSDF/VolumeGrid.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/GridSDF/cu_raycast_grid.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/GridSDF/cu_sdffusion_grid.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/GridSDF/cu_rolling_sdf.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/GridSDF/BoundedVolumeGrid.h"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/GridSDF/RollingGridSDF.h"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libkangaroo.dylib")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib" TYPE SHARED_LIBRARY FILES "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/libkangaroo.dylib")
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/libkangaroo.dylib" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libkangaroo.dylib")
    execute_process(COMMAND "/usr/bin/install_name_tool"
      -id "libkangaroo.dylib"
      "$ENV{DESTDIR}/usr/local/lib/libkangaroo.dylib")
    execute_process(COMMAND /usr/bin/install_name_tool
      -delete_rpath "/usr/local/cuda/lib"
      "$ENV{DESTDIR}/usr/local/lib/libkangaroo.dylib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/lib/libkangaroo.dylib")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Kangaroo" TYPE FILE FILES
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/CMakeFiles/KangarooConfig.cmake"
    "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/KangarooConfigVersion.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Kangaroo/KangarooTargets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Kangaroo/KangarooTargets.cmake"
         "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/CMakeFiles/Export/lib/cmake/Kangaroo/KangarooTargets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Kangaroo/KangarooTargets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/Kangaroo/KangarooTargets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Kangaroo" TYPE FILE FILES "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/CMakeFiles/Export/lib/cmake/Kangaroo/KangarooTargets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/Kangaroo" TYPE FILE FILES "/Users/luma/Code/RobotGroup/MKangaroo/kangaroo/CMakeFiles/Export/lib/cmake/Kangaroo/KangarooTargets-release.cmake")
  endif()
endif()

