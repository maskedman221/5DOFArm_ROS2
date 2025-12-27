#!/usr/bin/env python3
import json
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial
import time
from sensor_msgs.msg import JointState
class SerialNode(Node):
    def __init__(self):
        super().__init__("serial_node")
        self.ip = "192.168.1.5"
        self.TCP_port = 3232
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.ip, self.TCP_port))
        self.sock.setblocking(False)
        # self.ser = serial.Serial("/dev/ttyACM0" , 115200)
        self.get_logger().info(f"Creating subscription to /joint_commands")
        # self.static_timer = self.create_timer(3.0, self.send_static_json)
        self.publisher = self.create_publisher(Bool , '/serial_status' , 10)
        self.subscription = self.create_subscription(JointState , '/joint_commands' , self.send_json , 10 )
        self.read_timer = self.create_timer(1.0, self.read_TCP)
        # self.write_timer = self.create_timer(1.0, self.send_message)
        self.get_logger().info(f"Subscription created: {self.subscription}")
        self.publisher.publish(Bool(data=True))

    def read_TCP(self):
        try:
            data = self.sock.recv(1024).decode().strip()
            if data:
                self.get_logger().info(f"Arduino Says: {data}")
                if data == "finished movement":
                    self.publisher.publish(Bool(data=True))
        except BlockingIOError:
            # No data available right now
            pass
        except socket.timeout:
            # Timed out waiting for data
            pass
        except socket.error as e:
            self.get_logger().error(f"Socket error: {e}")

        # if self.ser.in_waiting :
        #     data = self.ser.readline().decode().strip() 
        #     self.get_logger().info(f"Arduino Says: {data}")
    
    def write_TCP(self , msg):
        try:
            self.sock.sendall((msg + "\n").encode())
            self.get_logger().error("sending data")
        except socket.error:
            self.get_logger().error("TCP disconnected")

    def send_message(self , msg):
        positions = msg.position
        position_str = ','.join(f'{pos: .4f} 'for pos in positions[:5])
        self.write_TCP(position_str)
        self.get_logger().info(f'Sent: {position_str.strip()}', throttle_duration_sec=1.0)

    def send_json(self , msg):
        jointjson = self.format_json(msg)
        json_str = json.dumps(jointjson, separators=(',', ':'))
        json_str += '\n'
        self.sock.sendall(json_str.encode('utf-8'))
        self.get_logger().info(f'Sent: {json_str.strip()}')
    
    def format_json(self , msg):
        positions = msg.position
        jointjson = {
            'type': 'joints',  # Message type identifier
            'timestamp': time.time(),
            'joints': {
                'j1': float(positions[0]),
                'j2': float(positions[1]),
                'j3': float(positions[2]),
                'j4': float(positions[3]),
                'j5': float(positions[4])
            },
        }
        return jointjson
    

    def send_static_json(self):
        """Send a fixed static JSON position to Arduino"""
        try:
            # Fixed test position (in radians)
            fixed_angles = [0.0, 45, 0.0, 0.0, 0.0]  # Joint2 at 45°
            
            json_data = {
                'type': 'static_command',
                'timestamp': time.time(),
                'joints': {
                    'j1': float(fixed_angles[0]),
                    'j2': float(fixed_angles[1]),
                    'j3': float(fixed_angles[2]),
                    'j4': float(fixed_angles[3]),
                    'j5': float(fixed_angles[4])
                }
            }
            
            json_str = json.dumps(json_data) + '\n'
            self.sock.sendall(json_str.encode('utf-8'))
            self.get_logger().info(f"Sent static position: {fixed_angles[1]:.1f}° at joint2")
            
        except Exception as e:
            self.get_logger().error(f"Error sending static JSON: {e}")
def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
