import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import serial
import math

class SerialPublisher(Node):
    def __init__(self):
        super().__init__('serial_publisher')
        self.cmd_vel_subscription = self.create_subscription(Twist, '/self_cmd_vel', self.listener_callback, 10)
        self.self_subscription = self.create_subscription(Float32MultiArray, '/self_position', self.update_position_callback, 10)
        
        # Set up serial communication
        self.serial_port = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
        
        # Robot parameters
        self.wheel_radius = 0.05  # Adjust as needed
        self.wheel_base = 0.233     # Adjust as needed

        self.x = 0
        self.y = 0
        self.theta = 0
        self.vx = 0
        self.vy = 0
        self.omega = 0
    
    def listener_callback(self, msg):
        self.vx = msg.linear.x * math.cos(self.theta) + msg.linear.y * math.sin(self.theta)
        self.vy = -msg.linear.x * math.sin(self.theta) + msg.linear.y * math.cos(self.theta)
        self.omega = msg.angular.z
        wheel_velocities = self.compute_wheel_velocities()
        
        # Convert velocities to non-negative integers and create binary string
        signs = ['0' if v >= 0 else '1' for v in wheel_velocities]
        wheel_velocities = [abs(round(v)) for v in wheel_velocities]
        binary_string = ''.join(signs)
        
        # Construct message
        message = f"{wheel_velocities[0]},{wheel_velocities[1]},{wheel_velocities[2]},{wheel_velocities[3]},{binary_string}\n"
        
        # Send over serial
        self.serial_port.write(message.encode('utf-8'))
        self.get_logger().info(f"Sent: {message}")

    def compute_wheel_velocities(self): # Omnidirectional four-wheel drive kinematics
        d = 2.5744
        v1 = (d*(self.vx - self.vy) - self.omega * self.wheel_base) /self. wheel_radius
        v2 = (d*(self.vx + self.vy) - self.omega * self.wheel_base) /self. wheel_radius
        v3 = (d*(self.vx + self.vy) + self.omega * self.wheel_base) /self. wheel_radius
        v4 = (d*(self.vx - self.vy) + self.omega * self.wheel_base) /self. wheel_radius
        c = 100
        return [v1*c, v2*c, v3*c, v4*c]
    
    def update_position_callback(self, msg): # self.get_logger().info(f"Received: {msg.data}")
        self.x = msg.data[0]
        self.y = msg.data[1]
        self.theta = msg.data[2]

def main(args=None):
    rclpy.init(args=args)
    serial_publisher = SerialPublisher()
    rclpy.spin(serial_publisher)
    serial_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
