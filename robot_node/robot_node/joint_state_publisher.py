import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import threading
from collections import deque
import math 
class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        self.joint_state_msg = JointState()
        self.joint_state_msg.name = ['left_wheel_joint_1', 'right_wheel_joint_1', 'left_wheel_joint_2', 'right_wheel_joint_2']
        self.joint_state_msg.position = [0.0, 0.0, 0.0, 0.0]

        self.timer = self.create_timer(0.05, self.publish_joint_state)

        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        self.data_buffer = deque(maxlen=10)
        self.lock = threading.Lock()

        self.reading_thread = threading.Thread(target=self.read_uart, daemon=True)
        self.reading_thread.start()

    def publish_joint_state(self):
        with self.lock:
            if self.data_buffer:
                data = self.data_buffer.popleft()
                self.process_uart_data(data)

        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state_msg)

    def read_uart(self):
        while rclpy.ok():
            if self.serial_port.in_waiting > 0:
                try:
                    data = self.serial_port.readline().decode('utf-8').strip()
                    self.get_logger().info(f"Received data: {data}")
                    self.get_logger().info(f"{self.joint_state_msg.position}")
                    with self.lock:
                        self.data_buffer.append(data)
                except UnicodeDecodeError as e:
                    self.get_logger().warn(f"Unicode Decode Error: {e}")
                except Exception as e:
                    self.get_logger().warn(f"UART Read Error: {e}")

    def process_uart_data(self, data):
        try:
            parts = data.split()
            joint_positions_degrees = []
            for part in parts:
                if part.startswith('a'):
                    joint_positions_degrees.append(float(part.split(':')[1]))
                elif part.startswith('b'):
                    joint_positions_degrees.append(float(part.split(':')[1]))
                elif part.startswith('c'):
                    joint_positions_degrees.append(float(part.split(':')[1]))
                elif part.startswith('d'):
                    joint_positions_degrees.append(float(part.split(':')[1]))

            if len(joint_positions_degrees) == 4:
                joint_positions_radians = [(deg * math.pi / 180.0) for deg in joint_positions_degrees]
                self.joint_state_msg.position = joint_positions_radians
            else:
                self.get_logger().warn(f"Incorrect joint data format: {data}")

        except ValueError as e:
            self.get_logger().error(f"Error parsing data: {e}")

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()