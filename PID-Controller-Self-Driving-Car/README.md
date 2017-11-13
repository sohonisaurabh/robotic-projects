# CarND-PID-Control-Project
This repository contains C++ code for implementation of PID controller to derive steering angles for a car to drive around a circular track. This task was implemented to partially fulfill Term-II goals of Udacity's self driving car nanodegree program


## Background

A critical module in the working of any robotic system is the control module. Control module defines the action which the robotic system performs in order to achieve a task. These actions can vary on the type of system and the type of task. For e.g.: A simple mixer grinder's control module only controls the speed of rotating more. A little more complex system such as a Remote Controlled (RC) car needs a control module to move forward, backward and turn. Highly complex systems such as prosthetic arms, self driving cars, product manufacturing factories require control modules for multiple tasks at the same time.

One of the basic implementation of a control system is a Proportional (P), Differential (D), Integral (I), together, PID controller. PID controller is the most popular controller and is used in applications across domains. 


## Working of PID Controller

The basic principle of working of a PID controller is to satify a boundary value problem. Most common examples of such problems  are either minimization of the total error in the system or maximiation of the total gain in the system. These error or gain problems are represented mathematically and are used to govern the effect to P, I and D components of a PID controller. These components are described below:

  1. Proportional (P) component:
    Mathematically, the P component establishes linear relationship between the problem. The effect of P component is in proportional to the value of problem at the current time step. For e.g.:, if the problem is to mimimize the error in the system at current time step, the value of P component is given by the formula:
    
    αp = -Kp * error
    
    where, Kp is a tuning parameter known as the proportional gain. The negative sign in the beginining signifies that P component is used to cancel the effect or error, or in other words, reduce it.
    
  2. Differential (D) component:
    Mathematically, the D component establishes linear relationship between the rate of change of problem. The effect of D component is in proportional to the rate of change problem from last time step to current time step. For e.g.:, if the problem is to mimimize the uncertainty in the system, the value of D component is given by the formula:
    
    αd = -Kd * d(error)/dt
    
    where, Kd is a tuning parameter known as the differential gain. The negative sign in the beginining signifies that D component is used to cancel the effect or rate of change of error, or in other words, reduce it. The D component is used to minimize sudden changes in the system.
    
  3. Integral (D) component:
    Mathematically, the I component establishes linear relationship between the average value problem over time. The effect of I component is in proportional to the average value of problem from the beginining of time to the current time step. For e.g.:, if the problem is to mimimize the error in the system, the value of I component is given by the formula:
    
    αi = -Ki * ∑ error
    
    where, Ki is a tuning parameter known as the integral gain. The negative sign in the beginining signifies that I component is used to cancel the effect or average error, or in other words, reduce it. The I component is used to correct systemic bias.
    
  When all components are used, the mathematical equation is given by:
  
  α = (-Kp * error) + (-Kd * d(error)/dt) + (Ki * ∑ error)
  
  where, α is the control input to the system, often known as the **actuator** input.
  

## Project Goal

In this project, a PID controller was implemented to drive a car around circular track having sharp left and right turns. The good solution would help the car stay in the center portion of the lane and take smooth left and right turns without touching or running over the edges of the lane (considered as risky in case humans were travelling in such a car).


## Project Implementation

Simulation of a circular track was achieved in the [Udacity's self driving car simulator](https://github.com/udacity/self-driving-car-sim/releases). The simulator measured the cross track error (cte) between the lateral position of car and the center of the lane. This error was communicated to C++ code with the help of [uWebSockets library](https://github.com/uNetworking/uWebSockets). The cte was then used to calculate P, I and D components of the controller governing the steering angle of the car.

The final implementation consisted of following major steps:

  1. Cross track erro (cte) recorded by the simulator was used to develop the mathematical equation for calculating steering angle of the car. This is given below:
  
![steering angle equation](https://raw.githubusercontent.com/sohonisaurabh/CarND-PID-Control-Project/master/image-resources/steering-angle-equation.png)
  
  2. The major task in this implementation was to tune the Kp, Ki and Kd gain parameters. This was done using manual tuning in the begining and followed by fine tuning by using of Twiddle or [Gradient Descent](https://en.wikipedia.org/wiki/Gradient_descent). This process is listed in the following steps.
  
  3. In the initial step, the I and D components were switched off and only the P component was used to drive the car. This was done by setting the Ki and Kd parameters to 0. The car was allowed to drive along the track. The value of Kp was tuned manually to help the car stay on the track for at least 20% of the total track length, as long as it doesn't cross the edge of the track. This is demonstrated in the video below:
  
  [video](https://www.youtube.com/watch?v=29PmNG7fuuM)
  
  As expected, the car could only stay for around 20% on the track due to oscillations in the run. These oscillations were created due to overshoot phenomenon created by P component, best explained by Sebastian in [this](https://youtu.be/SZ5D2AbWr3s) video. These oscillations were prevented by introducing the D component as described in next step.
  
  4. In this step, a PD controller was used. The I component was still switched off by setting the value of Ki to zero. The value of Kp was as tuned from step 3 and the value of Kd was tuned manually to keep the car on track for most of the length of the track. This is demonstrated in the video below:
  
  [video](https://www.youtube.com/watch?v=29PmNG7fuuM)
  
  As seen, the car was able to track on the track and drive succesfully for most of the portion. However, it was observed that the car wouldn't stay in the center of the lane and often drift to the edges. This caused in very sharp turns which is certainly not desirable in case a human was sitting inside the car. As described in next step, introductio of I component solved this problem.
  
  5. Due to some kind of systemic bias and uncertainty in the system, the car would often drift to the edges of the lane. This undesired behavior was corrected by introducing I component. The value of Kp and Kd were as tuned from step 3 and step 4 respectively. The value of Ki was tuned manually to restrict the car drifting away from center of the lane. The final effect of use of PID controller is demonstrated in the video below:
  
  
