#!/usr/bin/env python

import rclpy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import math
import time
import matplotlib.pyplot as plt

class ThrusterControllerGetPosition:
    def __init__(self):
        rclpy.init()

        # creates node
        self.node = rclpy.create_node('thruster_controller_get_position')

        # Publisher in order to publish to specific topics
        self.right_thruster_pub = self.node.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10000)
        self.left_thruster_pub = self.node.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10000)
        self.right_pos_pub = self.node.create_publisher(Float64, '/wamv/thrusters/right/pos', 100000)
        self.left_pos_pub = self.node.create_publisher(Float64, '/wamv/thrusters/left/pos', 100000)

        # Subscriber to get the models position
        self.sub = self.node.create_subscription(Odometry, '/wamv/sensors/position/ground_truth_odometry', self.callback, 10)
        self.position = None
        self.yaw_degrees = None

    def callback(self, msg):
        # model position
        self.position = msg.pose.pose.position

        # model orientation that is then
        self.orientation = msg.pose.pose.orientation

        x = self.orientation.x
        y = self.orientation.y
        z = self.orientation.z
        w = self.orientation.w

        # convert it to a heading
        yaw = self.quaternion_to_euler(x, y, z, w)
        self.yaw_degrees = math.degrees(yaw)

    def getpos(self):
        return (self.position.x, self.position.y), self.yaw_degrees

    def quaternion_to_euler(self, x, y, z, w):
        # conversion
        t0 = +2.0 * (w * z + x * y)
        t1 = +1.0 - 2.0 * (y**2 + z**2)
        yaw = math.atan2(t0, t1)

        return yaw

    def set_thrusters(self, thruster_value):
        # setting the left and right thruster
        print("Right thruster =", thruster_value)
        right_thruster_msg = Float64()
        right_thruster_msg.data = thruster_value
        self.right_thruster_pub.publish(right_thruster_msg)

        print("Left thruster =", thruster_value)
        left_thruster_msg = Float64()
        left_thruster_msg.data = thruster_value
        self.left_thruster_pub.publish(left_thruster_msg)

    def set_thrusters_pos(self, pos_value):
        # setting the left and right thruster positions
        print("Right thruster to pos=", pos_value)
        right_thruster_pos_msg = Float64()
        right_thruster_pos_msg.data = pos_value
        self.right_pos_pub.publish(right_thruster_pos_msg)

        print("Left thruster to pos=", pos_value)
        left_thruster_pos_msg = Float64()
        left_thruster_pos_msg.data = pos_value
        self.left_pos_pub.publish(left_thruster_pos_msg)

    def spin(self):
        rclpy.spin_once(self.node, timeout_sec=0.1)

def calculate_heading(x_boat, y_boat, x_buoy, y_buoy):
    # calculation for the desired heading usig trigonmetry
    heading_radians = math.atan2(y_buoy - y_boat, x_buoy - x_boat)
    heading_degrees = math.degrees(heading_radians)
    return heading_degrees

class PIDController:
    def __init__(self, heading, kp, ki, kd, interval):
        self.desired_heading = heading
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.interval = interval
        self.prev_error = 0
        self.integral = 0

    def compute(self, current_value):
        # compute the new output
        error = self.desired_heading - current_value
        self.integral += error * interval
        derivative = (error - self.prev_error)/interval

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.prev_error = error
        output = max(min(output, 90), -90)
        return output
    
    def setHeading(self, new_heading):
        self.desired_heading = new_heading

if __name__ == '__main__':

    # init the node
    initial = ThrusterControllerGetPosition()

    # values for the PID controller
    kp = 1.0
    ki = 0.9
    kd = 0.15
    interval = 0.1

    # additional variables
    reached = False
    res_yaw, res_error, res_thruster_pos, boat_pos = [], [], [], []

    # get the starting position
    for _ in range(20):
        initial.spin()
        time.sleep(0.3)
    position, yaw = initial.getpos()
    x,y = position[0], position[1]

    boat_pos.append([x,y])

    # calculate desired heading
    heading = calculate_heading(x, y, -528.0, 187.0)

    # init the PID controller with the correct values
    pid_controller = PIDController(heading, kp, ki, kd, interval)

    # set thrusters to constant speed
    initial.set_thrusters(100.0)


    while not reached:
        # keep node alive
        initial.spin()

        # get model's current position and heading
        position, yaw = initial.getpos()
        x,y = position[0], position[1]
        #print("Current Heading:", x, y, yaw, heading)

        # compute the new thruster position using the PID controller class
        thruster_pos = pid_controller.compute(yaw)
        initial.set_thrusters_pos(-math.radians(thruster_pos))

        # Append values for plotting
        res_yaw.append(yaw)
        res_error.append(heading-yaw)
        res_thruster_pos.append(-thruster_pos)
        boat_pos.append([x,y])
        
        #print(f"Thruster Position Command: {thruster_pos}")

        # check if the model reached the buoy
        if -528.0 <= x <= -524.5 and 184.5 <= y <= 188.5:
            heading = calculate_heading(x, y, -500.0, 200.0)
            pid_controller.setHeading(heading)


        if -500.0 <= x <= -496.5 and 188.5 <= y <= 201.5: 
            initial.set_thrusters(0.0)
            reached = True
        time.sleep(1)

    # Shutdown node
    rclpy.shutdown()


    # plots
    plt.figure(figsize=(10, 8))

    plt.subplot(2, 1, 1)
    plt.plot(res_yaw, label='Yaw', linewidth=2)
    plt.plot(res_error, label='Error', linewidth=2)
    plt.plot(res_thruster_pos, label='Thruster angle', linewidth=2)
    plt.title("Results")
    plt.xlabel("Time in seconds")
    plt.ylabel("Output values")
    plt.legend()

    plt.subplots_adjust(hspace=0.5)

    plt.subplot(2, 1, 2)
    boat_pos = list(zip(*boat_pos))
    plt.plot(boat_pos[0], boat_pos[1], linewidth=2)
    plt.scatter([-528.0, -500.0], [187.0, 200.0], s=100, c=['orange', 'green'] , marker='o', label='Buoy Positions')
    plt.title("Boat Position")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")

    plt.tight_layout()

    plt.show()
