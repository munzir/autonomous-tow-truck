import rclpy  # ROS 2 client library for Python
from std_msgs.msg import Float64MultiArray  # For publishing the pulses and timestamps
from geometry_msgs.msg import PoseStamped  # For publishing the pose
from std_msgs.msg import Float32MultiArray  # For publishing distance and wheel speed
import serial  # To use Serial (in Arduino IDE) for communication
from serial.tools import list_ports
from tf_transformations import quaternion_from_euler  # To convert theta to quaternion

def main():
    rclpy.init()  # Initialize rclpy
    node = rclpy.create_node('ros_arduino_communication')  # Create a ROS 2 node
    pub_pose = node.create_publisher(PoseStamped, 'robot_pose', 10)  # Publisher for pose data
    pub_distance_speed = node.create_publisher(Float32MultiArray, 'distance_speed_data', 10)  # Publisher for distance and wheel speed

    # Establish a serial connection to the Arduino
    try:
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
        	if port.device.startswith("/dev/ttyACM"):
        		break
        arduino = serial.Serial(port.device, 115200)  # Adjust the port and baud rate as necessary
        node.get_logger().info('Connected to serial port')
    except serial.SerialException as e:
        node.get_logger().error(f'Failed to connect to serial port: {e}')
        return

    try:
        while rclpy.ok():
            if arduino.in_waiting > 0:  # Check if data is available from Arduino
                line = arduino.readline().decode('utf-8').rstrip()  # Read the line, decode, and strip whitespace
                try:
                    # Assuming the Arduino is sending x, y, theta, distance, and speed separated by commas
                    x, y, theta, distance, speed = map(float, line.split(','))  # Split the line and convert to float

                    # Create the PoseStamped message
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = node.get_clock().now().to_msg()  # Set the timestamp
                    pose_msg.header.frame_id = 'map'  # Frame ID, adjust as needed

                    pose_msg.pose.position.x = x
                    pose_msg.pose.position.y = y
                    pose_msg.pose.position.z = 0.0  # Assuming a 2D plane

                    # Convert theta (yaw) to a quaternion
                    q = quaternion_from_euler(0, 0, theta)
                    pose_msg.pose.orientation.x = q[0]
                    pose_msg.pose.orientation.y = q[1]
                    pose_msg.pose.orientation.z = q[2]
                    pose_msg.pose.orientation.w = q[3]

                    # Publish the pose
                    pub_pose.publish(pose_msg)
                    node.get_logger().info(f'Published Pose: x={x}, y={y}, theta={theta}')

                    # Publish the distance and wheel speed
                    distance_speed_msg = Float32MultiArray()
                    distance_speed_msg.data = [distance, speed]
                    pub_distance_speed.publish(distance_speed_msg)
                    node.get_logger().info(f'Published Distance: {distance}, Speed: {speed}')

                except ValueError:
                    node.get_logger().warn(f'Received invalid data: {line}')

    except KeyboardInterrupt:
        pass

    finally:
        if arduino.is_open:
            arduino.close()  # Ensure the serial connection is closed
        node.destroy_node()  # Destroy the node
        rclpy.shutdown()  # Shutdown rclpy

if __name__ == '__main__':
    main()

