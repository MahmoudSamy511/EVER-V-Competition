#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from csv import reader
from dataclasses import dataclass
from math import radians
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import math

from kinematic_model import KinematicBicycleModel
from libs import CarDescription, StanleyController, generate_cubic_spline

class Simulation:
    def __init__(self):
        self.dt = 0.05  # Maintain the simulation time step
        self.map_size_x = 70
        self.map_size_y = 40
        self.frames = 2500
        self.loop = False
        self.clock_time = 0  # Initialize the clock time
        
    def update_clock(self, clock_time):
        self.clock_time = clock_time


class Path:

    def __init__(self):
        # Read path from 'combined_path.csv'
        with open('square_path copy.csv', newline='') as f:
            rows = list(reader(f, delimiter=','))
        
        # Parse the path
        x, y = [[float(i) for i in row] for row in zip(*rows[1:])]
        # Generate the cubic spline
        ds = 0.05
        self.px, self.py, self.pyaw, _ = generate_cubic_spline(x, y, ds)


class Car:

    def __init__(self, init_x, init_y, init_yaw, px, py, pyaw, delta_time):
        # Initialize ROS node
        rospy.init_node('car_controller', anonymous=True)
        self.steering_pub = rospy.Publisher('/SteeringAngle', Float64, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Float64, queue_size=10)
        rospy.Subscriber('/Imu', Imu, self.log_callback_imu)
        rospy.Subscriber('/simclock', Float64, self.log_callback_simclock)
        rospy.Subscriber('/odom', Odometry, self.log_callback_odom)
        
        # Initialize car parameters
        self.x = init_x
        self.y = init_y
        self.yaw = init_yaw
        self.delta_time = delta_time
        self.time = 0  # Initialize simulation time
        self.velocity = 0.0
        self.wheel_angle = 0.0
        max_steer = radians(30)
        wheelbase = 2.269
        
        # Target velocity and acceleration parameters
        self.required_acceleration = 0
        
        # Tracker parameters
        self.px = px
        self.py = py
        self.pyaw = pyaw
        self.k = 8.0
        self.ksoft = 1.0
        self.kyaw = 0.01
        self.ksteer = 0.0
        self.crosstrack_error = None
        self.target_id = None
        
        # Car description
        self.colour = 'black'
        overall_length = 2.269
        overall_width = 1.649
        tyre_diameter = 0.4826 / 2
        tyre_width = 0.265 / 2
        axle_track = 1.7 / 2
        rear_overhang = 0.5 * (overall_length - wheelbase)
        
        self.tracker = StanleyController(self.k, self.ksoft, self.kyaw, self.ksteer, max_steer, wheelbase, self.px, self.py, self.pyaw)
        self.kinematic_bicycle_model = KinematicBicycleModel(wheelbase, max_steer, self.delta_time)
        self.description = CarDescription(overall_length, overall_width, rear_overhang, tyre_diameter, tyre_width, axle_track, wheelbase)

    def get_required_acceleration(self):
        return self.required_acceleration
    
    def log_callback_odom(self, msg):
        # Extract position
        twist = msg.twist.twist.linear
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.x, self.y= position.x, position.y
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.yaw = yaw
        twist_msg = f"\nPosition: [x={twist.x}, y={twist.y}, z={twist.z}]\n"
        self.velocity = math.sqrt(twist.x**2 + twist.y**2)

    def log_callback_simclock(self, msg):
        if self.time == 0:
            self.start_time = msg.data
            self.time = 0
        self.delta_time = msg.data - self.start_time - self.time
        self.time = msg.data - self.start_time
        self.drive()
        
    def log_callback_imu(self, msg):
        # Calculate linear acceleration
        linear_acceleration = msg.linear_acceleration
        self.required_acceleration = (linear_acceleration.x**2 + linear_acceleration.y**2)**0.5

    def plot_car(self):
        return self.description.plot_car(self.x, self.y, self.yaw, self.wheel_angle)

    def drive(self):
        # Calculate acceleration
        self.required_acceleration = self.get_required_acceleration()
        self.wheel_angle, self.target_id, self.crosstrack_error = self.tracker.stanley_control(self.x, self.y, self.yaw, self.velocity, self.wheel_angle)
        
        # Publish steering angle and acceleration
        steering_angle_deg = self.wheel_angle * 180 / 3.14
        self.steering_pub.publish(Float64(steering_angle_deg))
        
        gas_pedal_msg = Float64()
        gas_pedal_msg.data = 0.2
        self.cmd_vel_pub.publish(gas_pedal_msg)
        
        # # Update the kinematic model
        # self.x, self.y, self.yaw, self.velocity, _, _ = self.kinematic_bicycle_model.update(self.x, self.y, self.yaw, self.velocity, acceleration, self.wheel_angle)


@dataclass
class Fargs:
    ax: plt.Axes
    sim: Simulation
    path: Path
    car: Car
    car_outline: plt.Line2D
    front_right_wheel: plt.Line2D
    front_left_wheel: plt.Line2D
    rear_right_wheel: plt.Line2D
    rear_left_wheel: plt.Line2D
    rear_axle: plt.Line2D
    annotation: plt.Annotation
    target: plt.Line2D


def animate(frame, fargs):
    ax = fargs.ax
    sim = fargs.sim
    path = fargs.path
    car = fargs.car
    car_outline = fargs.car_outline
    front_right_wheel = fargs.front_right_wheel
    front_left_wheel = fargs.front_left_wheel
    rear_right_wheel = fargs.rear_right_wheel
    rear_left_wheel = fargs.rear_left_wheel
    rear_axle = fargs.rear_axle
    annotation = fargs.annotation
    target = fargs.target
    
    # Update car's position
    car.drive()
    outline_plot, fr_plot, rr_plot, fl_plot, rl_plot = car.plot_car()
    
    # Update car plots
    car_outline.set_data(*outline_plot)
    front_right_wheel.set_data(*fr_plot)
    rear_right_wheel.set_data(*rr_plot)
    front_left_wheel.set_data(*fl_plot)
    rear_left_wheel.set_data(*rl_plot)
    rear_axle.set_data(car.x, car.y)
    
    # Update target plot
    target.set_data(path.px[car.target_id], path.py[car.target_id])
    
    # Update annotation
    annotation.set_text(f'{car.x:.1f}, {car.y:.1f}')
    annotation.set_position((car.x, car.y + 5))
    
    # Update title and label
    plt.title(f'{sim.clock_time + sim.dt * frame:.2f}s', loc='right')
    plt.xlabel(f'Speed: {car.velocity:.2f} m/s', loc='left')
    
    return (car_outline, front_right_wheel, rear_right_wheel, front_left_wheel, rear_left_wheel, rear_axle, target)



def main():
    sim = Simulation()
    path = Path()
    car = Car(path.px[0], path.py[0], path.pyaw[0], path.px, path.py, path.pyaw, sim.dt)
    interval = sim.dt * 10 ** 3
    # rospy.spin()
    fig = plt.figure()
    ax = plt.axes()
    ax.set_aspect('equal')

    road = plt.Circle((0, 0), 50, color='gray', fill=False, linewidth=30)
    ax.add_patch(road)
    ax.plot(path.px, path.py, '--', color='gold')

    empty = ([], [])
    target, = ax.plot(*empty, '+r')
    car_outline, = ax.plot(*empty, color=car.colour)
    front_right_wheel, = ax.plot(*empty, color=car.colour)
    rear_right_wheel, = ax.plot(*empty, color=car.colour)
    front_left_wheel, = ax.plot(*empty, color=car.colour)
    rear_left_wheel, = ax.plot(*empty, color=car.colour)
    rear_axle, = ax.plot(car.x, car.y, '+', color=car.colour, markersize=2)
    annotation = ax.annotate(f'{car.x:.1f}, {car.y:.1f}', xy=(car.x, car.y + 5), color='black', annotation_clip=False)

    fargs = [Fargs(
        ax=ax,
        sim=sim,
        path=path,
        car=car,
        car_outline=car_outline,
        front_right_wheel=front_right_wheel,
        front_left_wheel=front_left_wheel,
        rear_right_wheel=rear_right_wheel,
        rear_left_wheel=rear_left_wheel,
        rear_axle=rear_axle,
        annotation=annotation,
        target=target
    )]

    _ = FuncAnimation(fig, animate, frames=sim.frames, init_func=lambda: None, fargs=fargs, interval=interval,repeat=sim.loop)
    # anim.save('animation.gif', writer='imagemagick', fps=50)

    plt.grid()
    plt.show()


if __name__ == '__main__':
    main()
