
from my_py_pkg.camera_vision_classes.CameraProcessor import CameraProcessor
from my_py_pkg.camera_vision_classes.ColorDetector import ColorDetector
from my_py_pkg.camera_vision_classes.ObjectFinder import ObjectFinder
from std_msgs.msg import Bool
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class CameraVisionNode(Node):
    def __init__(self):
        super().__init__("vision")
        self.detector = ColorDetector()
        self.finder = ObjectFinder()
        self.processor = CameraProcessor(self.detector,self.finder)
        self.publisher = self.create_publisher(Point , '/target_position' , 10)
        self.subscription = self.create_subscription(Bool , '/serial_status' , self.init_Image  ,10)
        self.alreadyVisioned = False
        self.detections_list = []
        self.detector = 0
    #     self.timer = self.create_timer(20 , self.publish_point)
    
    # def publish_point(self):
    #     point = Point()
    #     point.x=0.10
    #     point.y=0.05
    #     point.z=0.02
    #     self.publisher.publish(point)
        
    def init_Image(self , msg):
        self.get_logger().debug(msg.data)
        if self.alreadyVisioned:
            self.publishPoint()
        elif msg.data:
            self.get_logger().debug("init the Image processing")
            self.alreadyVisioned = True
            self.run()

    def run(self):
        print("Starting Color Detection App...")
        self.detections_list = self.processor.run()
    
    def publishPoint(self):
        x, y = self.detections_list[self.detector]['real_arm_cm']
        point = Point()
        point.x = x/100
        point.y = y/100
        point.z = 0.12
        self.detector += 1
        self.publisher.publish(point)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = CameraVisionNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
