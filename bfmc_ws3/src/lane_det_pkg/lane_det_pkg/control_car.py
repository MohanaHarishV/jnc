from rclpy.node import Node
from sensor_msgs.msg import Image
import rclpy
import matplotlib.pyplot as plt
import numpy as np
import math
from geometry_msgs.msg import Twist
from interface_pkg.msg import Lanedata
from numpy import interp


class Control:

    def __init__(self):
        self.prev_Mode = "Detection"
        self.angle = 0.0


        self.prev_Mode_LT = "Detection"
        self.car_speed = 5
        self.angle_of_car = 0

        self.Left_turn_iterations = 0
        self.Frozen_Angle = 0
        self.Detected_LeftTurn = False
        self.Activat_LeftTurn = False        

        self.TrafficLight_iterations = 0
        self.GO_MODE_ACTIVATED = False
        self.STOP_MODE_ACTIVATED = False


    def follow_Lane(self,Max_Sane_dist,distance,curvature):

        IncreaseTireSpeedInTurns = False
        
        Max_turn_angle_neg = -90
        Max_turn_angle = 90

        CarTurn_angle = 0

        if( (distance > Max_Sane_dist) or (distance < (-1 * Max_Sane_dist) ) ):
            if(distance > Max_Sane_dist):
                CarTurn_angle = Max_turn_angle + curvature
            else:
                CarTurn_angle = Max_turn_angle_neg + curvature
        else:

            Turn_angle_interpolated = interp(distance,[-Max_Sane_dist,Max_Sane_dist],[-90,90])
            CarTurn_angle = (0.65*Turn_angle_interpolated) + (0.35*curvature)

        # Handle Max Limit [if (greater then either limits) --> set to max limit]
        if( (CarTurn_angle > Max_turn_angle) or (CarTurn_angle < (-1 *Max_turn_angle) ) ):
            if(CarTurn_angle > Max_turn_angle):
                CarTurn_angle = Max_turn_angle
            else:
                CarTurn_angle = -Max_turn_angle

        #angle = CarTurn_angle
        # [NEW]: Increase car turning capability by 30 % to accomodate sharper turns
        self.angle = interp(CarTurn_angle,[-90,90],[-45,45])
        curr_speed = self.car_speed

        
        return self.angle , curr_speed
    
    def drive_car(self,Current_State):

        [Distance, Curvature] = Current_State

        current_speed = 0
        
        if((Distance != -1000) and (Curvature != -1000)):

            # [NEW]: Very Important: Minimum Sane Distance that a car can be from the perfect lane to follow is increased to half its fov.
            #                        This means sharp turns only in case where we are way of target XD
            self.angle_of_car , current_speed = self.follow_Lane(640/4, Distance,Curvature)
        else:
            current_speed = 0
            
        self.final_angle = interp(self.angle_of_car,[-45,45],[0.5,-0.5])
        self.final_speed = interp(current_speed,[30,80],[0,1])
        
        return self.final_angle,self.final_speed
        
class Car(Node):
    def __init__(self):
        super().__init__('lane_data_subscriber')
        self.subscription = self.create_subscription(Lanedata,'lanedata_topic',self.listener_callback,10)
        self.Control = Control()
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 20)
        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.finalspeed = 0.0
        self.finalangle =0.0

        
    def listener_callback(self,data):
        Current_state = [data.perpendicular_distance,data.curvature]
        self.finalangle,self.finalspeed = self.Control.drive_car(Current_state)
        self.get_logger().info("Recieving data")
        
    def timer_callback(self):
        cmd_msg = Twist()
        cmd_msg.linear.x = self.finalspeed
        cmd_msg.linear.y = 0.0
        cmd_msg.linear.z = 0.0
        cmd_msg.angular.x = 0.0
        cmd_msg.angular.y = 0.0
        cmd_msg.angular.z = self.finalangle
        self.publisher_.publish(cmd_msg)
        self.get_logger().info('Publishing Velocity and controls')
        
def main(args=None):
    rclpy.init(args=args)
    Carcontrol = Car()
    rclpy.spin(Carcontrol)
    Carcontrol.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()