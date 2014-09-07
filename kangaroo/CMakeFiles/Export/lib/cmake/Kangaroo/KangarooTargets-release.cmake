#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "kangaroo" for configuration "Release"
set_property(TARGET kangaroo APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(kangaroo PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "/usr/local/cuda/lib/libcudart.dylib;-Wl,-rpath;-Wl,/usr/local/cuda/lib;/usr/local/cuda/lib/libnpp.dylib;/opt/local/lib/libassimp.dylib;/usr/local/lib/libopencv_core.dylib;/usr/local/lib/libopencv_imgproc.dylib;/usr/local/lib/libopencv_features2d.dylib;/usr/local/lib/libopencv_flann.dylib;/usr/local/lib/libopencv_calib3d.dylib;/usr/local/lib/libopencv_objdetect.dylib;/usr/local/lib/libopencv_legacy.dylib;/usr/local/lib/libopencv_contrib.dylib;/usr/local/lib/libopencv_highgui.dylib;/usr/local/lib/libopencv_ml.dylib;/usr/local/lib/libopencv_video.dylib;/usr/local/lib/libopencv_gpu.dylib"
  IMPORTED_LOCATION_RELEASE "/usr/local/lib/libkangaroo.dylib"
  IMPORTED_SONAME_RELEASE "libkangaroo.dylib"
  )

list(APPEND _IMPORT_CHECK_TARGETS kangaroo )
list(APPEND _IMPORT_CHECK_FILES_FOR_kangaroo "/usr/local/lib/libkangaroo.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
