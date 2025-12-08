#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
import serial
import time
from sensor_msgs.msg import JointState
class SerialNode(Node):
    def __init__(self):
        super().__init__("serial_node")
        self.ser = serial.Serial("/dev/ttyACM0" , 115200)
        self.subscription = self.create_subscription(JointState , '/joint_commands' , self.send_json , 10 )
        self.read_timer = self.create_timer(1.0, self.read_serial)
        # self.write_timer = self.create_timer(1.0, self.send_message)

    def read_serial(self):
        if self.ser.in_waiting :
            data = self.ser.readline().decode().strip() 
            self.get_logger().info(f"Arduino Says: {data}")
    
    def write_serial(self , msg):
        self.ser.write((msg + "\n").encode())

    def send_message(self , msg):
        positions = msg.position
        position_str = ','.join(f'{pos: .4f} 'for pos in positions[:5])
        self.write_serial(position_str)
        self.get_logger().info(f'Sent: {position_str.strip()}', throttle_duration_sec=1.0)

    def send_json(self , msg):
        jointjson = self.format_json(msg)
        json_str = json.dumps(jointjson, separators=(',', ':'))
        json_str += '\n'
        self.ser.write(json_str.encode('utf-8'))
        self.get_logger().debug(f'Sent: {json_str.strip()}')

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
def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
