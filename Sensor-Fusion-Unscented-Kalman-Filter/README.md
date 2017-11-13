# CarND-Unscented-Kalman-Filter
This repository contains C++ code for implementation of Unscented Kalman Filter project. This task was implemented to partially fulfill Term-II goals of Udacity's self driving car nanodegree program.

## Background

Self driving cars make use of Laser sensor (LIDAR) and/or Radial distance and angle sensor (RADAR) for tracking moving objects such as vehicles, pedestrians, animals, etc. Data received from LIDAR and RADAR is fused to best estimate the trajectory of motion of the object. In this project, trajectory of a bicycle moving in the shape of numerical figure 8 is estimated using LIDAR and RADAR measurements. This is achieved with the help of vanilla Kalman filter and Unscented version of Kalman filter.

## State vector and model

For this project, a Constant Turn Rate and Velocity (CTRV) model is assumed while building the state vector. The state vector contains following components:

1. Position of the object in X axis (px)
2. Position of the object in Y axis (py)
3. Speed, or magnitude of velocity (v)
4. Yaw angle (psi)
5. Yaw rate, or rate of change of yaw angle (psidot)

where X and Y axis are relative to the direction in which the self driving car moves, shown below:

![UKF state vector and model](https://raw.githubusercontent.com/sohonisaurabh/CarND-Unscented-Kalman-Filter/master/image-resources/ctrv-state-vector.png)

## Unscented Kalman Filter Implementation Algorithm:

Following goals were achieved as a part of implementation:

1. Build the state vector and the state transition matrix. Derive state transition equation. This represents the deterministic part of motion model. Stochastic part of motion is represented by νak (For magnitude of acceleration) and νpsik (For yaw acceleration or the change in yaw rate). The state transition equation then derived is shown below:

![CTRV model state transition equation](https://raw.githubusercontent.com/sohonisaurabh/CarND-Unscented-Kalman-Filter/master/image-resources/CTRV-state-transition-equation.png)

where the noise is modelled by assuming Gaussian noise with zero mean and standard deviation. Hence, standard deviations are σa (For magnitude of acceleration) and σyaw (For yaw acceleration). This is shown below:

![CTRV process noise modelled as Gaussian](https://raw.githubusercontent.com/sohonisaurabh/CarND-Unscented-Kalman-Filter/master/image-resources/ctrv-noise-modelling.png)

2. LIDAR measures the distance between self driving car and an object in X and Y axis. Hence, the measurement function for LASER updates, given by H_laser, is a linear transform shown below:

![LASER measurement function](https://raw.githubusercontent.com/sohonisaurabh/CarND-Extended-Kalman-Filter/master/image-resources/H_laser.png)

3. RADAR measures the radial distance, the bearing (or angle of orientation w.r.t car) and the radial velocity. This is represented below:

![RADAR measurement](https://raw.githubusercontent.com/sohonisaurabh/CarND-Unscented-Kalman-Filter/master/image-resources/radar_measurement.png)

Hence, the measurement function for RADAR updates, given by H_radar, is a non-linear transform.

4. Now that the state transition and measurement functions are derived, Kalman filter is used to estimate the path of moving object. Upon receiving a measurement for timestamp k+1, following processes are triggered:

  a. Kalman filter Predict: To use the state vector at timestamp k (Xk) and **predict** the state vector at timestamp k (Xk+1). This is the updated belief after motion.
  b. Use the measurement and update the belief using Kalman filter **update** once measurement is received.
  
5. Kalman filter predict step is same for LASER and RADAR measurements. This step involves use of Unscented Kalman Filter algorithm to predict the mean and covariance for the next step. Given the belief of state and covariance matrix at state k, Unscented Kalman Filter algorithm consists of following steps:

  a. Generate sigma points: In this step, 2n + 1 sigma points are generated, where n is the number of states in state vector. Here, one sigma point is the mean of state while rest all are chosen on the uncertainty ellipse of the Gaussian distribution.
  b. Predict sigma points: The sigma points generated in step a are then passed through the process model and to predict the sigma points for the state k+1.
  c. Predict the mean and covariance: Predicted sigma points from step b are then used to calculate the predicted mean and covariance for the state k+1.

6. When a LASER measurement at step k+1 is received, use vanilla Kalman filter (since the measurement function is linear) equations to update the predicted belief in step 5.

7. When a RADAR measurement at step k+1 is received, again use Unscented Kalman filter (since the measurement function is non-linear) equations to update the predicted belief in step 5.

8. Take a note of Normalized Innovation Squared (NIS) at each step for both LASER and RADAR. Plot the NIS distribution and fine tune the process noise parameters (σa and σyaw) to attain consistency in the system. Equation for calculating NIS is given below:

![NIS equation](https://raw.githubusercontent.com/sohonisaurabh/CarND-Unscented-Kalman-Filter/master/image-resources/nis-equation.png)

9. Calculate the root mean squared error (RMSE) after Kalman filter update at each time step. This is given by:

![RMSE equation](https://raw.githubusercontent.com/sohonisaurabh/CarND-Unscented-Kalman-Filter/master/image-resources/rmse.png)


## Goals of the project

1. Implement Kalman filter algorithm in C++

2. Build the project and run the executable on dataset of LASER and RADAR measurements returned by [Udacity simulator](https://github.com/udacity/self-driving-car-sim/releases).

3. Calculate and capture the Normalized Innovation Squared (NIS) for LASER and RADAR. Plot the values and check for consistency. If not consistent, tune the process noise parameters σa and σyaw. The final tuned values for σa was 2 m/s2 and σyaw was 2 rad/s2. This resulted in NIS distributions shown below:

![LASER NIS distribution](https://raw.githubusercontent.com/sohonisaurabh/CarND-Unscented-Kalman-Filter/master/image-resources/ukf-nis-laser-dataset-1.png.png)

![RADAR NIS distribution](https://raw.githubusercontent.com/sohonisaurabh/CarND-Unscented-Kalman-Filter/master/image-resources/ukf-nis-radar-dataset-1.png.png)

4. Take a note of RMSE values at the last time step of dataset. Minimize the RMSE to bring it in the range of RMSE <= [.09, .10, 0.40, 0.30] for px, py, vx and vy respectively. RMSE values achieved after fine minimization are shown below:

**Run on dataset 1**

![RMSE on dataset I](https://raw.githubusercontent.com/sohonisaurabh/CarND-Unscented-Kalman-Filter/master/image-resources/ukf-rmse-dataset-1.png)




**Run on dataset 2**

![RMSE on dataset II](https://raw.githubusercontent.com/sohonisaurabh/CarND-Unscented-Kalman-Filter/master/image-resources/ukf-rmse-dataset-2.png)


## Steps for building the project in Ubuntu

1. Execute every step from install-ubuntu.sh. This will install gcc, g++, cmake, make and uWebsocketIO API.

2. Build project
  a. mkdir build && cd build
  b. cmake ..
  c. make
  d. ./UnscentedKF

3. Run the Udacity simulator and test the implementation on dataset 1 and 2.
