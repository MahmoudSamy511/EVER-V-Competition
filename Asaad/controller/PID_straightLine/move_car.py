#!/usr/bin/env python

from pid_controller import *
import rospy
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from math import sqrt
import matplotlib.pyplot as plt
from plotter import *

# List of pairs containing the linear_acceleration with simulation_time (linear_acceleration, simulation_time)
linear_acceleration_data = []

# List of pairs containing the linear_velocity with simulation_time (linear_velocity, simulation_time)
linear_velocity_data = []

# List of pairs containing the distance with simulation_time (distance, simulation_time)
distance_data = []

class Move_Car:
    def __init__(self):
        # Init Ros node.
        rospy.init_node('PID_test', anonymous=True)
        
        # A publisher on the cmd_vel topic controls the gas padel
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Float64, queue_size = 10)
        
        # A publisher on the brakes topic controls the brakes padel
        self.brakes_pub = rospy.Publisher('/brakes', Float64, queue_size = 10)
        
        # A Subscriber on the odometry topic, to get linear velocity and current position of the car
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # A Subscriber on the Imu topic, to get the linear acceleration.
        self.imu_sub = rospy.Subscriber('/Imu', Imu, self.Imu_callback)
        
        # A Subscriber on the startSimulation topic, to start the simulation automatically. 
        self.start_simulation = rospy.Publisher('/startSimulation', Bool, queue_size = 10)
        
        # A Subscriber on the stopSimulation topic, to stop the simulation automatically.
        self.stop_simulation = rospy.Publisher('/stopSimulation', Bool, queue_size = 10)
        
        # A Subscriber on the simclock topic, to get the simulation time.
        self.simclock = rospy.Subscriber('/simclock', Float64, self.simclock_callback)
        
        # Wait for the publishers and subscribers to connect.
        rospy.sleep(1)
        
        # Start the simulation
        self.start_simulation.publish(Bool(True))
        
        # Reset the brakes.
        self.brakes_pub.publish(Float64(0))

        # An object of the PID controller.
        self.pid_controller = PIDController(kp = 0.4, ki = 0.01, kd = 0.21)
        
        # The rquired target distance. 
        self.target_distance = 75  
        
        # The current distance of the car.
        self.current_distance = 0

        # The current speed of the car
        self.current_speed = 0

        # The current acceleration of the car.
        self.current_acceleration = 0.0
        
        # The simulation current time
        self.simulation_time = 0.0

        # Define the rate of rospy spinning 
        self.rate = rospy.Rate(10)  # 10 Hz

    def odom_callback(self, msg):
        """
            A callback function, gets the returned value from the /odom topic
            msg: Odometry.
        """
        
        # Calculate the distance traveled by the car. 
        self.current_distance = sqrt(msg.pose.pose.position.x ** 2 + msg.pose.pose.position.y ** 2)
        
        # Get the current speed.
        self.current_speed = msg.twist.twist.linear.y
        
        # Save the current_speed with the current simulation_time for plotting..
        linear_velocity_data.append((self.current_speed, self.simulation_time))
        
        # Save the current distance with the current simulation_time for plotting.
        distance_data.append((self.current_distance, self.simulation_time))
    
    def Imu_callback(self, msg):
        """
            A callback function, gets the returned value from the /Imu topic
            msg: Imu
        """

        # Get the current acceleration.
        self.current_acceleration = msg.linear_acceleration.y

        # Save the current acceleration with the current simulation_time for plotting.
        linear_acceleration_data.append((self.current_acceleration, self.simulation_time))
    
    def simclock_callback(self, msg):
        """
            A callback function, gets the returned value from the /simclock topic
            msg: Float64
        """
        # Get the simulation time.
        self.simulation_time = msg.data

    def move(self):
        """
            A software interface moves the car for distance equal to the required 
            (target distance - 3), with reserved 3 meters for the brakes to stop the car gradually.
        """
        while (not rospy.is_shutdown()) and ((self.current_distance) < (self.target_distance - 3)):
            # Calculate the error for the PID controller
            error = self.target_distance - self.current_distance

            # Get the calculated output from the PID.
            pid_output = self.pid_controller.compute(error, self.rate.sleep_dur.to_sec())

            # Map pid_output to the range [0, 1.0]
            pid_output = max(0, min(pid_output /  self.target_distance, 1.0))

            # Publish pid_output to cmd_vel
            self.cmd_vel_pub.publish(pid_output)

            self.rate.sleep()

        # Gradually apply brakes
        for _ in range(10):
            self.brakes_pub.publish(0.866)
            self.rate.sleep()

        # Stop the car completely
        self.brakes_pub.publish(1.0)

        # Wait and stop the simulation
        if rospy.is_shutdown:
            rospy.sleep(5)
            # Print the travelled distance.
            print("Last distance", self.current_distance)
            
            # Stop the simulation 
            self.stop_simulation.publish(Bool(True))

if __name__ == '__main__':
    try:
        move_robot = Move_Car()
        move_robot.move()

        # Plotting linear acceleration
        plot_linear_acceleration(linear_acceleration_data)

        # Plotting linear velocity
        plot_linear_velocity(linear_velocity_data)

        # Plotting distance
        plot_distance(distance_data)

        plt.suptitle(f'PID Control output\nLast Distance {move_robot.current_distance}')
        plt.show()
    except rospy.ROSInterruptException:
        pass
