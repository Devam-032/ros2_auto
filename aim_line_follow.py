import rclpy
from rclpy.node import Node

from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor
from collections import deque
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from nxp_cup_interfaces.msg import PixyVector
from time import sleep
from datetime import datetime
import numpy as np
Kp=0.53   
Ki=0.00012
Kd=0.00013  
global sum_error
sum_error=0.0
global prev_error
prev_error=0.0
global i 
i=0

class LineFollow(Node):

    def __init__(self):
        super().__init__('aim_line_follow')
        self.start_delay = 2.0
        self.camera_vector_topic = "/cupcar0/PixyVector"
        self.linear_velocity = 0.75  #set your desired value
        self.angular_velocity = -0.7    #set your desired value
        self.single_line_steer_scale = 1.0    #set your desired value
        self.q1 = deque()
        
        # Time to wait before running
        self.get_logger().info('Waiting to start for {:s}'.format(str(self.start_delay)))
        sleep(self.start_delay)
        self.get_logger().info('Started')

        self.start_time = datetime.now().timestamp()
        self.restart_time = True

        # Subscribers
        self.pixy_subscriber = self.create_subscription(
            PixyVector,
            self.camera_vector_topic,
            self.listener_callback,
            10)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cupcar0/cmd_vel', 10)
        
        self.speed_vector = Vector3()
        self.steer_vector = Vector3()
        self.cmd_vel = Twist()

        

    def get_num_vectors(self, msg):
        
        num_vectors = 0
        if(not(msg.m0_x0 == 0 and msg.m0_x1 == 0 and msg.m0_y0 == 0 and msg.m0_y1 == 0)):
            num_vectors = num_vectors + 1
        if(not(msg.m1_x0 == 0 and msg.m1_x1 == 0 and msg.m1_y0 == 0 and msg.m1_y1 == 0)):
            num_vectors = num_vectors + 1
        return num_vectors

    # def timer_callback(self):
    #     #TODO

    def listener_callback(self, msg):
        #TODO
        current_time = datetime.now().timestamp()
        frame_width = 72
        frame_height = 52
        window_center = (frame_width / 2)
        x = 0
        y = 0
        steer = 0
        speed = 0
        m1x1 = 0
        m0x1 = 0
        num_vectors = self.get_num_vectors(msg)

        if(num_vectors == 0):
            if self.restart_time:
                self.start_time = datetime.now().timestamp()
                self.restart_time = False
            if (self.start_time+4.0) > current_time:
                speed = self.linear_velocity * (4.0-(current_time-self.start_time))/4.0
            if (self.start_time+4.0) <= current_time:
                speed = -1.0
            steer = 0

        if(num_vectors == 1):
            if not self.restart_time:
                self.start_time = datetime.now().timestamp()
                self.restart_time = True
            if(msg.m0_x1 > msg.m0_x0):
                x = (msg.m0_x1 - msg.m0_x0) / frame_width
                y = (msg.m0_y1 - msg.m0_y0) / frame_height
            else:
                x = (msg.m0_x0 - msg.m0_x1) / frame_width
                y = (msg.m0_y0 - msg.m0_y1) / frame_height
            if(msg.m0_x0!=msg.m0_x1 and y!=0):
                
                steer = (-self.angular_velocity) * (x / y) * (0.6) * self.single_line_steer_scale
                if (self.start_time+4.0) > current_time:
                    speed = 1.2*self.linear_velocity * ((current_time-self.start_time)/4.0)
                if (self.start_time+4.0) <= current_time:
                    speed = 1.2*self.linear_velocity
            else:
                steer = 0
                if (self.start_time+4.0) > current_time:
                    speed = 1.2*self.linear_velocity * ((current_time-self.start_time)/4.0)
                if (self.start_time+4.0) <= current_time:
                    speed = 1.2*self.linear_velocity

        if(num_vectors == 2):
            global Kp
            global Ki
            global Kd
            global sum_error
            global prev_error
            global i
            m0x1 = msg.m0_x1     
            m1x1 = msg.m1_x1    
            
            
            if not self.restart_time:
                self.start_time = datetime.now().timestamp()
                self.restart_time = True
            avg = (m0x1 + m1x1) / 2 
            sum_error=sum_error+avg - window_center     
            if i<20:      
                self.q1.append(sum_error)
                i += 1
            else:         
                self.q1.popleft()
                s=np.sum(self.q1)
                self.q1.append(sum_error)
                s=np.sum(self.q1)
                i += 1
            
            s=np.sum(self.q1)
            self.prev_error = avg - window_center       
            steerp = (avg - window_center)*Kp           
            steeri = sum_error*Ki
            steerd = (-prev_error+(avg - window_center))*Kd
            steer= self.angular_velocity*(steerp+steeri+steerd)/frame_width
            prev_error=avg - window_center
            if((m1x1<36 and m0x1<36) or (m1x1>36 and m0x1>36)):
                steer=(-1)*steer
            if (self.start_time+4.0) > current_time:
                speed = 1.2*self.linear_velocity * ((current_time-self.start_time)/4.0)
            if (self.start_time+4.0) <= current_time:
                speed = 1.2*self.linear_velocity

        self.speed_vector.x = float(speed*(1-np.abs(2.0*steer)))
        if ((m0x1<10 or m0x1>60) and num_vectors==1):
            steer = steer*1.5
            self.speed_vector.x=0.4
        if(num_vectors==2 and ((m1x1<36 and m0x1<36) or (m1x1>36 and m0x1>36))): 
            self.speed_vector.x=1.05
            steer = steer*1.2
        elif(num_vectors==2 and abs(msg.m1_x0-msg.m0_x0)<2):
            self.speed_vector.x=0.12
        self.steer_vector.z = float(steer)
        self.cmd_vel.linear = self.speed_vector
        self.cmd_vel.angular = self.steer_vector
        
        self.cmd_vel_publisher.publish(self.cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    line_follow = LineFollow()
    rclpy.spin(line_follow)
    line_follow.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
