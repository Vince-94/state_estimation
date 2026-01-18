# State Estimation

- [State Estimation](#state-estimation)
  - [Overview](#overview)
  - [Setup](#setup)
    - [Dependencies](#dependencies)
  - [Docker](#docker)
  - [Reference](#reference)


## Overview

This project implements various state estimation :
- Kalman Fitler
  - Linear Kalman Fitler
  - Extended Kalman Fitler
  - Unscented Kalman Fitler
- Particle Filter


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

## Docker
- Build docker image
    ```sh
    docker compose build
    ```
- Run and enter the docker container
    ```sh
    docker compose run --rm state_estimation bash
    ```


## Reference

