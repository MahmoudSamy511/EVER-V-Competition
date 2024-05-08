#!/usr/bin/env python3

from rospy import *
from std_msgs.msg import Float64, Bool

def simclock_callback(data):
    """
        A callback function invoked every time there're published
        data on the topic named /simclock
    """
    global simulation_time
    simulation_time = int(data.data)
    print("simulation time is ", simulation_time)

def move():
    rospy.init_node('test_simTopics', anonymous=True)
    
    # Subscribe on the /simclock topic
    simulationTtime_sub = rospy.Subscriber('/simclock', Float64, simclock_callback)
    # Subscribe on the /cmd_vel
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Float64, queue_size=10)
    # Subscribe on the /brakes
    brakes_pub = rospy.Publisher('/brakes', Float64, queue_size=10)
    # Subscribe on the /SteeringAngle topic
    steering_pub = rospy.Publisher('/SteeringAngle', Float64, queue_size=10)
    # Subscribe on the /startSimulation topic
    startSimulation_pub = rospy.Publisher('/startSimulation', Bool, queue_size=10)

    rospy.sleep(2)  # Wait for publishers to register

    startSimulation_pub.publish(Bool(True))
    rospy.sleep(0.5)        # Wait for the simulation to start
    
    brakes_pub.publish(Float64(0.0))    # Reset the brakes

    cmd_vel_pub.publish(Float64(0.5))   # Start moving 

    steering_pub.publish(Float64(0.0))  # No steering 

    # Stop the car after 5 seconds from starting the simulation
    start_time = simulation_time
    while (not rospy.is_shutdown()) and (simulation_time - start_time < 5):
        rospy.sleep(0.1)
   
    cmd_vel_pub.publish(Float64(0.0))
    brakes_pub.publish(Float64(5.0))
    rospy.loginfo('Car has stopped')
    rospy.signal_shutdown("5 seconds have passed")
        
if __name__=='__main__':
    move()
    rospy.spin()  
