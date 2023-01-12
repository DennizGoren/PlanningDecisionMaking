# -*- coding: utf-8 -*-
"""
Created on Mon Dec 19 18:08:11 2022

@author: Badr_
"""

import numpy as np
import math
import matplotlib.pyplot as plt

class PID:
    def __init__(self,path):
        self.path = path  # Global path with nodes in array
        self.current_pos = [path[0,0], path[0,1], 0, 0]  # (x, y, theta, v)
        self.time_to_complete = 5.0  # Desired time to complete the global path in seconds
        self.time_passed = 0.0

        self.steering_path = []
        self.velocity_path = []
        self.orientation = []


    def unicycle_model(self,current_pos, omega, velocity, dt):
        x, y, theta, v = current_pos
    
        v = velocity
        # Update the heading direction based on the steering angle
        theta += omega*dt

        # Update the position based on the linear velocity
        x += v *dt* math.cos(theta)
        y += v *dt* math.sin(theta)

        return x, y, theta, v

    def unicycle_controller(self,current_pos, goal_pos, time_to_complete,dt,steering_pid,velocity_pid):
        # Calculate the desired heading direction
        heading = math.atan2(goal_pos[1] - current_pos[1], goal_pos[0] - current_pos[0])

        # Calculate the heading error
        heading_error = heading - current_pos[2]

        # Calculate the control output for the steering angle using a PID controller
        Kp = 1.0
        Ki = 0.0
        Kd = 0.5
        #steering_pid[0] += heading_error * dt
        #steering_integral = 0.0
        #steering_derivative = 0.0
        #steering_previous_error = 0.0
        steering_pid[0] += heading_error * dt
        steering_derivative = (heading_error - steering_pid[1]) / dt
        steering = Kp * heading_error + Ki * steering_pid[0] + Kd * steering_derivative
        steering_pid[1] = heading_error
    
    # Calculate the control output for the linear velocity using a PID controller
        distance_to_goal = math.sqrt((goal_pos[0] - current_pos[0])**2 + (goal_pos[1] - current_pos[1])**2)
        desired_velocity = distance_to_goal / (time_to_complete+0.01)
        velocity_error = desired_velocity - current_pos[3]
    
        Kp = 0.5
        Ki = 0.0
        Kd = 0.0
    
        velocity_pid[0] += velocity_error * dt
        velocity_derivative = (velocity_error - velocity_pid[1]) / dt
        velocity = Kp * velocity_error + Ki * velocity_pid[0] + Kd * velocity_derivative
        velocity_pid[1] = velocity_error
    
        print(velocity)

        # Make sure the unicycle is facing the right direction before applying a control input for driving forward
        if abs(heading_error) > math.pi / 32:
            velocity = 0.0

        return steering, velocity,steering_pid,velocity_pid




    def control(self):
        pos = []
        vel = []
        time = []

        for goal_pos in self.path: 
        #in order: integral error, previous error
            steering_pid = [0,0]
            velocity_pid = [0,0]
            node_length = math.sqrt((goal_pos[0] - self.current_pos[0])**2 + (goal_pos[1] - self.current_pos[1])**2)+0.000001 #add small term to make it robust to zero division
            while True:
                remaining_length = math.sqrt((goal_pos[0] - self.current_pos[0])**2 + (goal_pos[1] - self.current_pos[1])**2)
                time_to_complete_remaining = self.time_to_complete*remaining_length/node_length
                dt = 0.05
        
                self.time_passed += dt
                steering, velocity, steering_pid,velocity_pid = self.unicycle_controller(self.current_pos, goal_pos, time_to_complete_remaining,dt,steering_pid,velocity_pid)
                # Update the current position based on the control outputs and the unicycle model dynamics
                self.steering_path.append(steering)
                self.velocity_path.append(velocity)
                self.orientation.append(self.current_pos[2])
                self.current_pos = self.unicycle_model(self.current_pos, steering, velocity,dt)
                # Check if the robot has reached the goal
                threshold = 0.01
                if remaining_length < threshold:
                    break


