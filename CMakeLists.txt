cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/ )
SET(USE_GUI TRUE)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  FIND_PACKAGE(Threads REQUIRED)
  FIND_PACKAGE(OpenCV REQUIRED)
  pkg_check_modules(Eigen REQUIRED eigen3)
  if(USE_GUI)
    FIND_PACKAGE(Pangolin REQUIRED)
  endif()
endif()

# Set definitions
SET(CXX_DEFS "")
if(USE_GUI)
  SET(CXX_DEFS "${CXX_DEFS} -DUSE_GUI")
endif()
SET(CMAKE_CXX_FLAGS "-fpermissive -Wall -g -O3 -D_LINUX -D_REENTRANT -march=native -msse3 --std=c++11 ${CXX_DEFS}")

# Include dirs
INCLUDE_DIRECTORIES(
  ./
  ./ui/
  ./extra/
  /usr/local/include
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  if(USE_GUI)
    ${Pangolin_INCLUDE_DIRS}
  endif()
)

# Set sourcefiles
LIST(APPEND SOURCEFILES
  #Main
  main.cc sdvl.cc video_source.cc camera.cc point.cc frame.cc feature.cc config.cc
  #Mapping
  map.cc homography_init.cc
  #Tracking
  image_align.cc feature_align.cc matcher.cc
  #Extra classes
  extra/se3.cc extra/fast_detector.cc extra/orb_detector.cc extra/utils.cc extra/bundle.cc
)

if(USE_GUI)
  LIST(APPEND SOURCEFILES
    #UI
    ui/drawimage.cc ui/drawscene.cc ui/ui.cc
  )
endif()

# Set link libraries
LIST(APPEND LINK_LIBS 
  ${CMAKE_THREAD_LIBS_INIT}
  ${OpenCV_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/extra/g2o/lib/libg2o.so
)

ADD_EXECUTABLE(SDVL ${SOURCEFILES})
TARGET_LINK_LIBRARIES(SDVL ${LINK_LIBS})
