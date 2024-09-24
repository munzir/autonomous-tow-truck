import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
from serial.tools import list_ports
import time

class GearSwitchNode(Node):
    def __init__(self):
        super().__init__('gear_switch_node')
        
        # Establish a serial connection to the Arduino
        try:
            ports = list(serial.tools.list_ports.comports())
            for port in ports:
                if port.device.startswith("/dev/ttyACM"):
                    break
            self.arduino = serial.Serial(port.device, 9600)  # Adjust the port and baud rate as necessary
            self.get_logger().info('Connected to serial port')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')
            raise RuntimeError('Could not establish serial connection to Arduino')

        # Subscribe to the /joy topic to get controller input
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

    def send_command(self, command):
        """Send the gear command to the Arduino."""
        self.arduino.write(command.encode())  # Send the command to the Arduino
        self.get_logger().info(f'Sent command: {command}')

    def joy_callback(self, msg):
        """Callback function to handle input from the PS4 controller."""
        # Example: Assume button X (index 0) is for Forward, Circle (index 1) for Reverse, and Square (index 2) for Neutral
        if msg.buttons[0] == 1:  # X button
            self.send_command('f')
        elif msg.buttons[1] == 1:  # Circle button
            self.send_command('r')
        elif msg.buttons[2] == 1:  # Square button
            self.send_command('n')
        elif msg.buttons[9] == 1:  # Options button for quitting
            self.get_logger().info('Quit command received')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    gear_switch_node = GearSwitchNode()
    rclpy.spin(gear_switch_node)

if __name__ == '__main__':
    main()

