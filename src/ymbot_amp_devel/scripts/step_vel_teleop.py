#!/usr/bin/env python3

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
        
        # Axes (no Deadman Button needed for basic cruise control)
        self.speed_axis = 6
        self.angular_axis = 4
        self.angular_scale = 0.5
        
        # To avoid firing multiple times on a single press, we keep track of the axis state
        self.last_speed_axis_val = 0.0
        
        self.current_joy = None
        
        # Publish continuously at 10Hz
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        
        # Subscriber
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        
        rospy.loginfo("Step Velocity Teleop Node Started.")
        rospy.loginfo("Speed control axis: %d, Step size: %.2f (No Deadman Required)", self.speed_axis, self.step_size)
        rospy.loginfo("Angular control axis: %d, Scale: %.2f", self.angular_axis, self.angular_scale)

    def joy_callback(self, msg):
        self.current_joy = msg
        
        # Update linear.x state based on axis 6 (D-Pad right/left or positive/negative)
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

    def timer_callback(self, event):
        cmd = Twist()
        cmd.linear.x = self.linear_x
        
        # Handle angular.z securely if joy message exists
        if self.current_joy is not None and self.angular_axis < len(self.current_joy.axes):
            cmd.angular.z = self.current_joy.axes[self.angular_axis] * self.angular_scale
            
        self.cmd_vel_pub.publish(cmd)

if __name__ == '__main__':
    try:
        node = StepVelTeleop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
