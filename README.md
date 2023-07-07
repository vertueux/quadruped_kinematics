Quadruped kinematics is a C++ library created to handle operations on a quadruped robot. There are no external dependencies like eigen3 or another C++ math library needed.
This project uses `std::vector` to create matrices. You can adapt this library to your needs.

> **Here are the last steps to complete in order to enter a stable version of this repository:**
> * ~~Program an algorithm to find the inverse of a 4x4 matrix based on the std::vector~~
> * ~~Run tests to verify that the entire program is working properly (90% test finished)~~

![license](https://img.shields.io/badge/license-AGPL_3.0-important)
![discord](https://img.shields.io/badge/Contact%20me%20on%20Discord-now%239470-informational)

# Installing CMake 
You need to have CMake installed to build the project on your computer. You can go to the direct link to download by clicking [here](https://cmake.org/download/). Follow all instructions on the CMake site.

# Clone the Project
You can clone and run this package by copying the command below :
```bash
git clone --recursive https://github.com/vertueux/quadruped_kinematics.git
```

# Build the Project
These commands should be able to build the project on any platform that CMake supports, otherwise follow the instructions for each platform from the [CMake website](https://cmake.org).
```bash
cmake --build
make 
```
