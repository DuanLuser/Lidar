# Lidar
Lidar with raspberrypi

## 实现
基于激光雷达提供的C++代码进行实现，连接到树莓派

## 指令
mkdir build

cd build

cmake ../ -G "CodeBlocks - Unix Makefiles"

make

sudo chmod 777 /dev/ttyUSB0(NOTE:please select correct serial in “app\*.cpp”)

./empty

./detect
