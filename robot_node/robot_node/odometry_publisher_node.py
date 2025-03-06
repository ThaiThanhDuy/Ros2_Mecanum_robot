import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import math
from collections import deque
import threading
from sensor_msgs.msg import JointState
class OdometryPublisherNode(Node):
    def __init__(self):
        super().__init__('odometry_publisher_node')
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
       
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        self.joint_state_msg = JointState()
        self.joint_state_msg.name = ['left_wheel_joint_1', 'right_wheel_joint_1', 'left_wheel_joint_2', 'right_wheel_joint_2']
        self.joint_state_msg.position = [0.0, 0.0, 0.0, 0.0]

        self.x = 0.0
        self.y = 0.0
        self.last_time = self.get_clock().now()

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.orientation = 0.0

        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")
            exit(1)

        self.data_buffer = deque(maxlen=10)
        self.lock = threading.Lock()

        self.reading_thread = threading.Thread(target=self.read_uart, daemon=True)
        self.reading_thread.start()

        self.timer = self.create_timer(0.02, self.publish_odom) # 50hz

    def publish_odom(self):
        with self.lock:
            if self.data_buffer:
                data = self.data_buffer.popleft()
                self.process_uart_data(data)

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        self.x += (self.linear_x * math.cos(self.orientation) - self.linear_y * math.sin(self.orientation)) * dt
        self.y += (self.linear_x * math.sin(self.orientation) + self.linear_y * math.cos(self.orientation)) * dt

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        q = self.euler_to_quaternion(0, 0, self.orientation)
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = self.linear_x
        odom.twist.twist.linear.y = self.linear_y
        odom.twist.twist.angular.z = self.angular_z

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q

        self.tf_broadcaster.sendTransform(t)
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state_msg)

      

    def read_uart(self):
        while rclpy.ok():
            if self.serial_port.in_waiting > 0:
                try:
                    data = self.serial_port.readline().decode('utf-8').strip()
                    self.get_logger().info(f"Received data: {data}")

                    with self.lock:
                        self.data_buffer.append(data)
                except UnicodeDecodeError as e:
                    self.get_logger().warn(f"Unicode Decode Error: {e}")
                except Exception as e:
                    self.get_logger().warn(f"UART Read Error: {e}")

    def process_uart_data(self, data):
        try:
            parts = data.split()
            yaw = 0.0

            for part in parts:
                if part.startswith('x'):
                    self.linear_x = float(part.split(':')[1])
                elif part.startswith('y'):
                    self.linear_y = float(part.split(':')[1])
                elif part.startswith('z'):
                    self.angular_z = float(part.split(':')[1])
                elif part.startswith('o'):
                    yaw = float(part.split(':')[1])

            yaw_radians = (yaw % 360) * (math.pi / 180)
            self.orientation = yaw_radians
            
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

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    odometry_publisher_node = OdometryPublisherNode()
    rclpy.spin(odometry_publisher_node)
    odometry_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()