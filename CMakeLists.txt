cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/ )

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  FIND_PACKAGE(OpenCV REQUIRED)
  pkg_check_modules(Eigen REQUIRED eigen3)
  pkg_check_modules(Config++ REQUIRED libconfig++)
  FIND_PACKAGE(Pangolin REQUIRED)
endif()

# Set definitions
SET(CMAKE_CXX_FLAGS "-fpermissive -Wall -g -O3 -D_LINUX -D_REENTRANT -march=native -msse3 --std=c++11")

# Include dirs
INCLUDE_DIRECTORIES(
  ./
  ./ui/
  ./extra/
  /usr/local/include
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Config++_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

# Set sourcefiles
LIST(APPEND SOURCEFILES
  #Main
  main.cc sdvl.cc video_source.cc camera.cc point.cc frame.cc feature.cc config.cc
  #Mapping
  map.cc homography_init.cc
  #Tracking
  image_align.cc feature_align.cc matcher.cc
  #UI
  ui/drawimage.cc ui/drawscene.cc ui/ui.cc
  #Extra classes
  extra/se3.cc extra/fast_detector.cc extra/orb_detector.cc extra/utils.cc extra/bundle.cc
)

# Set link libraries
LIST(APPEND LINK_LIBS 
  ${OpenCV_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Config++_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/extra/g2o/lib/libg2o.so
)

ADD_EXECUTABLE(SDVL ${SOURCEFILES})
TARGET_LINK_LIBRARIES(SDVL ${LINK_LIBS})
