import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmlVel(Node):
    def __init__(self):
        super().__init__('cml_vel_node')
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200)  # Adjust port and baud rate
        self.command_in_progress = False  # Initialize command in progress flag
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        # This method is called when a new Twist message is received
        # ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
        self.send_command(msg.linear.x, msg.linear.y, msg.angular.z)

    def send_command(self, linear_x, linear_y, angular_z):
        if self.command_in_progress:
            self.get_logger().warn("Previous command still in progress. Please wait.")
            return

        # Check if the serial port is open
        if self.serial_port is None or not self.serial_port.is_open:
            self.get_logger().warn("Cannot send command: Port is not open.")
            return

        self.command_in_progress = True
        try:

            # Send the new command to the STM32
            command = f"c:{linear_x},{linear_y},{angular_z}\n"
            self.serial_port.write(command.encode())
            self.get_logger().info(f"Sending command: {command.strip()}")

        except ValueError:
            self.get_logger().error("Invalid input. Please enter numeric values.")
        finally:
            self.command_in_progress = False  # Reset the state variable

def main(args=None):
    rclpy.init(args=args)
    cml_vel_node = CmlVel()
    rclpy.spin(cml_vel_node)
    cml_vel_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()