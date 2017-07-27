# Semi-Direct Visual Localization (SDVL)

## 1. License

This code is released under a [GPLv3 license](https://www.gnu.org/licenses/gpl-3.0.en.html).

For commercial purposes contact to: eperdices (at) gsyc (dot) es.

## 2. Dependencies

The following dependencies are necessary for running SDVL in debian-based distributions. Installation has been tested in Ubuntu 16.04 and Debian 9.

Install all dependencies (except Pangolin and g2o) using this command:

```
sudo apt-get install cmake libeigen3-dev libopencv-dev freeglut3-dev libglew-dev
```

### 2.1 CMake
---

Tool designed to build software.

```
sudo apt-get install cmake
```

### 2.2 Eigen
---

Library for linear algebra, used for matrices and vectors calculations.

```
sudo apt-get install libeigen3-dev 
```

### 2.3 OpenCV
---

Computer vision library employed to manipulate images.

```
sudo apt-get install libopencv-dev
```

### 2.4 Pangolin
---

Library for managing OpenGL display, employed for visualization and user interface.

First, OpenGL dependencies must be installed:

```
sudo apt-get install freeglut3-dev libglew-dev
```

Then, download from [github](https://github.com/stevenlovegrove/Pangolin) and install using these commands:

```
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
cmake --build .
sudo make install
```

### 2.5 G2o

C++ framework for optimizing graph-based nonlinear error functions. It has been used to optimize map with Bundle Adjustment.

This library is included in ''extra'' folder with some modifications made by [ORBSLAM authors](https://github.com/raulmur/ORB_SLAM2).

## 3. Install

Download from [github](https://github.com/JdeRobot/slam)

```
git clone https://github.com/JdeRobot/slam-SDVL.git
```

### G2o compilation
---

Install g2o dependency from ''extra'' folder:

```
cd slam-SDVL/extra/g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

### SDVL compilation
---

Run these commands:

```
cd slam-SDVL
mkdir build
cd build
cmake ..
make
```

### Usage
---

Change configuration parameters in config.cfg. Some examples are provided in '''config'' folder. Then, just run:

```
./SDVL
```

## 4. Coding Style

We use Google C++ Style Guide:

https://google.github.io/styleguide/cppguide.html
