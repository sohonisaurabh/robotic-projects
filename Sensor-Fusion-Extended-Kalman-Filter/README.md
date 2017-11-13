# CarND-Extended-Kalman-Filter

This repository contains C++ code for implementation of Extended Kalman Filter project. This project was partially fulfills the goals of Term-II of Udacity's self driving car engineer nanodegree program.

## Background

Self driving cars make use of Laser sensor (LIDAR) and/or Radial distance and angle sensor (RADAR) for tracking moving objects such as vehicles, pedestrians, animals, etc. Data received from LIDAR and RADAR is fused to best estimate the trajectory of motion of an object. In this project, trajectory of an object moving in the shape of numerical figure 8 is estimated using LIDAR and RADAR measurements. This is achieved with the help of Kalman filter.

## State vector and model

For this project, a Constant Velocity (CV) model is assumed and state vector is built. The state vector contains following components:

1. Position of object in X axis (px)
2. Position of object in Y axis (py)
3. Velocity of object in X axis (vx)
4. Velocity of object in Y axis (vy)

where X and Y axis are relative to the direction in which the self driving car moves, shown below:

![EKF axes definition](https://raw.githubusercontent.com/sohonisaurabh/CarND-Extended-Kalman-Filter/master/image-resources/EKF_axes_definition.png)


## Kalman Filter Implementation Algorithm:

Following goals were achieved as a part of implementation:

1. Build the state vector and the state transition matrix. Derive state transition equation. This represents the deterministic part of motion model. Stochastic part of motion is modelled by assuming Gaussian noise with zero mean and standard deviation as σax (For acceleration in X component) and σay (For acceleration in Y component). The state transition equation then derived is shown below:

![CV model state transition equation](https://raw.githubusercontent.com/sohonisaurabh/CarND-Extended-Kalman-Filter/master/image-resources/CV_state-transition-equation.png)

2. LIDAR measures the distance between self driving car and an object in X and Y axis. Hence, the measurement function for LASER updates, given by H_laser, is linear transform shown below:

![LASER measurement function](https://raw.githubusercontent.com/sohonisaurabh/CarND-Extended-Kalman-Filter/master/image-resources/H_laser.png)

3. RADAR measures the radial distance, the bearing (or angle of orientation w.r.t car) and the radial velocity. This is represented below:

![RADAR measurement](https://raw.githubusercontent.com/sohonisaurabh/CarND-Extended-Kalman-Filter/master/image-resources/radar_measurement.png)

Hence, the measurement function for RADAR updates, given by H_radar, is non-linear transform given by:

![RADAR measurement function](https://raw.githubusercontent.com/sohonisaurabh/CarND-Extended-Kalman-Filter/master/image-resources/H_radar.png)

4. Now that the state transition and measurements functions are derived, use Kalman filter to estimate the path of moving object. Upon receiving a measurement for timestamp k+1, following processes are triggered:
  a. Kalman filter predict to use the state vector at timestamp k, Xk, and **predict** the state vector at timestamp k, Xk+1. This is the updated belief.
  b. Use the measurement and update the belief using Kalman filter **update**.
  
5. Kalman filter predict step is same for LASER and RADAR measurements.

6. In case of LASER measurement, use normal Kalman filter equations. In case of RADAR update, use Extended Kalman Filter equations which assumes linear approximation of measurement function around mean.

7. Calculate the root mean squared error (RMSE) for after Kalman filter update at each time step. This is given by:

![RMSE equation](https://raw.githubusercontent.com/sohonisaurabh/CarND-Extended-Kalman-Filter/master/image-resources/rmse.png)


## Goals of the project

1. Implement Kalman filter algorithm in C++

2. Build the project and run the executable on dataset of LASER and RADAR measurements returned by [Udacity simulator](https://github.com/udacity/self-driving-car-sim/releases).

3. Take a note of RMSE values at the last time step of dataset. Minimize the RMSE to bring it in the range of RMSE <= [.11, .11, 0.52, 0.52] for px, py, vx and vy respectively. RMSE values achieved are given below:

Run on dataset I

![RMSE on dataset I](https://raw.githubusercontent.com/sohonisaurabh/CarND-Extended-Kalman-Filter/master/image-resources/EKF_dataset_1_result.png)

Run on dataset II

![RMSE on dataset II](https://raw.githubusercontent.com/sohonisaurabh/CarND-Extended-Kalman-Filter/master/image-resources/EKF_dataset_2_result.png)


## Steps for building the project in Ubuntu

1. Execute every step from install-ubuntu.sh. This will install gcc, g++, cmake, make and uWebsocketIO API.

2. Build project
  a. mkdir build && cd build
  b. cmake ..
  c. make
  d. ./ExtendedKF

3. Run the Udacity simulator and test the implementation on dataset I and II.
