import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math

def compute_wheel_velocities(vx, vy, omega, wheel_radius, wheel_base):
    # Omnidirectional four-wheel drive kinematics
    tempvx = vx
    tempvy = vy
    vx = tempvx * math.cos(math.pi/4) + tempvy * math.sin(math.pi/4)
    d= 2.5744
    v1 = (d*(vx - vy) - omega * wheel_base) / wheel_radius
    v2 = (d*(vx + vy) - omega * wheel_base) / wheel_radius
    v3 = (d*(vx + vy) + omega * wheel_base) / wheel_radius
    v4 = (d*(vx - vy) + omega * wheel_base) / wheel_radius
    c = 100
    return [v1*c, v2*c, v3*c, v4*c]

class SerialPublisher(Node):
    def __init__(self):
        super().__init__('serial_publisher')
        self.subscription = self.create_subscription(
            Twist,
            '/o1/cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning
        
        # Set up serial communication
        self.serial_port = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
        
        # Robot parameters
        self.wheel_radius = 0.05  # Adjust as needed
        self.wheel_base = 0.233     # Adjust as needed
    
    def listener_callback(self, msg):
        vx, vy, omega = msg.linear.x, msg.linear.y, msg.angular.z
        wheel_velocities = compute_wheel_velocities(vx, vy, omega, self.wheel_radius, self.wheel_base)
        
        # Convert velocities to non-negative integers and create binary string
        signs = ['0' if v >= 0 else '1' for v in wheel_velocities]
        wheel_velocities = [abs(round(v)) for v in wheel_velocities]
        binary_string = ''.join(signs)
        
        # Construct message
        message = f"{wheel_velocities[0]},{wheel_velocities[1]},{wheel_velocities[2]},{wheel_velocities[3]},{binary_string}\n"
        
        # Send over serial
        self.serial_port.write(message.encode('utf-8'))
        self.get_logger().info(f"Sent: {message}")


def main(args=None):
    rclpy.init(args=args)
    serial_publisher = SerialPublisher()
    rclpy.spin(serial_publisher)
    serial_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
