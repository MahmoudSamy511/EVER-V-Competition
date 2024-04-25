#!/usr/bin/env python

import matplotlib.pyplot as plt

def plot_linear_acceleration(linear_acceleration_data):
    acceleration_y_values = [pair[0] for pair in linear_acceleration_data]
    acceleration_x_values = [pair[1] for pair in linear_acceleration_data]
    plt.subplot(1, 3, 1)
    plt.plot(acceleration_x_values, acceleration_y_values, linestyle='-')
    plt.xlabel('Simulation time')
    plt.ylabel('Linear Acceleration')
    plt.title('Plot of Linear Acceleration over time')
    plt.grid(True)

def plot_linear_velocity(linear_velocity_data):
    velovity_y_values = [pair[0] for pair in linear_velocity_data]
    velovity_x_values = [pair[1] for pair in linear_velocity_data]
    plt.subplot(1, 3, 2)
    plt.plot(velovity_x_values, velovity_y_values, linestyle='-', color='red')
    plt.xlabel('Simulation time')
    plt.ylabel('Linear Velocity')
    plt.title('Plot of Linear Velocity over time')
    plt.grid(True)

def plot_distance(distance_data):
    distance_y_values = [pair[0] for pair in distance_data]
    distance_x_values = [pair[1] for pair in distance_data]
    plt.subplot(1, 3, 3)
    plt.plot(distance_x_values, distance_y_values, linestyle='-', color='blue')
    plt.xlabel('Simulation time')
    plt.ylabel('Distance')
    plt.title('Plot of distance over time')
    plt.grid(True)