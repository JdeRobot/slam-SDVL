cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/ )

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  FIND_PACKAGE(OpenCV REQUIRED)
  FIND_PACKAGE(Boost REQUIRED COMPONENTS thread system)
  pkg_check_modules(GTKMM REQUIRED gtkmm-2.4)
  pkg_check_modules(Glade REQUIRED libglademm-2.4)
  pkg_check_modules(GThread REQUIRED gthread-2.0)
  pkg_check_modules(GTK REQUIRED gtkglextmm-1.2 gtkglextmm-x11-1.2)
  pkg_check_modules(Eigen REQUIRED eigen3)
  FIND_PACKAGE(OpenGL REQUIRED)
  FIND_PACKAGE(GLUT REQUIRED)
  FIND_PACKAGE(Threads REQUIRED)
  pkg_check_modules(Config++ REQUIRED libconfig++)
endif()

# Set definitions
SET(CMAKE_CXX_FLAGS "-fpermissive -Wall -g -O3 -D_LINUX -D_REENTRANT -march=native -msse3 --std=c++11")

# Include dirs
INCLUDE_DIRECTORIES(
  ./
  ./ui/
  ./extra/
  /usr/local/include
  ${OpenCV_INCLUDE_DIRS} ${GTKMM_INCLUDE_DIRS} ${Glade_INCLUDE_DIRS}
  ${GThread_INCLUDE_DIRS} ${GTK_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS}
  ${Threads_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${Config++_INCLUDE_DIRS}
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
  ui/drawimage.cc ui/drawarea.cc ui/ui.cc
  #Extra classes
  extra/se3.cc extra/fast_detector.cc extra/orb_detector.cc extra/utils.cc extra/bundle.cc
)

# Set link libraries
LIST(APPEND LINK_LIBS 
  ${OpenCV_LIBRARIES} ${GTKMM_LIBRARIES} ${Glade_LIBRARIES}
  ${GThread_LIBRARIES} ${GTK_LIBRARIES}
  ${Eigen_LIBRARIES} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES}
  ${Threads_LIBRARIES} ${Boost_LIBRARIES} ${Config++_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/extra/g2o/lib/libg2o.so
)

ADD_EXECUTABLE(SDVL ${SOURCEFILES})
TARGET_LINK_LIBRARIES(SDVL ${LINK_LIBS})
