Semi-Direct Visual Localization

# Dependencies (Ubuntu 16.04 and Debian 9)
```
sudo apt-get install cmake libeigen3-dev libopencv-dev libboost-thread-dev libboost-system-dev libconfig++-dev libgtkmm-2.4-dev libglademm-2.4-dev libgtkglextmm-x11-1.2-dev freeglut3-dev
```

# Install
```
git clone git@github.com:JdeRobot/slam.git 
```

- G2o compilation
```
cd sdvl/extra/g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

- SDVL compilation
```
cd sdvl
mkdir build
cd build
cmake ..
make
```

- Usage
```
# Change configuration parameters in config.cfg. Some examples are provided in config folder
./SDVL
```

### Coding Style ###

https://google.github.io/styleguide/cppguide.html
