# Kalman Filter

## Overview

Kalman filter (KF) estimator library, with the following implementations:
- Linear Kalman Filter (LKF)
- Extended Kalman Filter (EKF)
- Unscent Kalman Filter (UKF)


## Setup

### Dependencies
- Eigen
    ```sh
    sudo apt install libeigen3-dev
    ```
- SDL2
    ```sh
    sudo apt install libsdl2-dev libsdl2-ttf-dev
    ```


## Build

### CMake
- Build
    ```sh
    ./build.bash
    ```

### ROS2
- Build
    ```sh
    colcon build --symlink-install
    ```
- Source
    ```sh
    source install/local_setup.bash
    ```


## Execution

- Linear Kalman Filter
  ```sh
  ros2 run kalman_filter linear_kalman_filter_node
  ```

### Launch

```sh
ros2 launch kalman_filter kalman_filter.launch.py
```


## Tests

```sh
./test.bash
```


## Reference

