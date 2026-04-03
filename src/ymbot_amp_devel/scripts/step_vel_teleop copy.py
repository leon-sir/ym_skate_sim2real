#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class StepVelTeleop:
    def __init__(self):
        rospy.init_node('step_vel_teleop')
        
        # Publisher
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        # State
        self.linear_x = 0.0
        self.step_size = 1.0
        
        # Axes and Buttons
        self.speed_axis = 6
        self.angular_axis = 4
        self.deadman_button = 4
        
        self.angular_scale = 0.5
        
        # To avoid firing multiple times on a single press, we keep track of the axis state
        self.last_speed_axis_val = 0.0
        
        # Subscriber
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        
        rospy.loginfo("Step Velocity Teleop Node Started.")
        rospy.loginfo("Deadman button: %d", self.deadman_button)
        rospy.loginfo("Speed control axis: %d, Step size: %.2f", self.speed_axis, self.step_size)
        rospy.loginfo("Angular control axis: %d, Scale: %.2f", self.angular_axis, self.angular_scale)

    def joy_callback(self, msg):
        # Check if deadman button is pressed
        if self.deadman_button < len(msg.buttons) and msg.buttons[self.deadman_button] == 1:
            cmd = Twist()
            
            # Update linear.x state based on axis 6 (D-Pad right/left or similar)
            if self.speed_axis < len(msg.axes):
                current_axis_val = msg.axes[self.speed_axis]
                
                # Check for rising edge on positive direction
                if current_axis_val > 0.5 and self.last_speed_axis_val <= 0.5:
                    self.linear_x += self.step_size
                    rospy.loginfo("Speed increased: %.2f", self.linear_x)
                # Check for rising edge on negative direction
                elif current_axis_val < -0.5 and self.last_speed_axis_val >= -0.5:
                    self.linear_x -= self.step_size
                    rospy.loginfo("Speed decreased: %.2f", self.linear_x)
                    
                self.last_speed_axis_val = current_axis_val
            
            cmd.linear.x = self.linear_x
            
            # Handle angular.z
            if self.angular_axis < len(msg.axes):
                cmd.angular.z = msg.axes[self.angular_axis] * self.angular_scale
                
            self.cmd_vel_pub.publish(cmd)
        else:
            # If deadman button is released, we might optionally publish 0 or just do nothing.
            # We'll publish 0 once and reset last_speed_axis_val just in case.
            self.last_speed_axis_val = 0.0

if __name__ == '__main__':
    try:
        node = StepVelTeleop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
