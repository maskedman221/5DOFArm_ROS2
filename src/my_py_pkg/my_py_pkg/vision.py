import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class visionNode(Node):
    def __init__(self):
        super().__init__("vision")
        self.publisher = self.create_publisher(Point , '/target_position' , 10)
        self.timer = self.create_timer(15 , self.publish_point)
        self.get_logger().info("inite the Point")
    
    def publish_point(self):
        point = Point()
        point.x=0.12
        point.y=0.0
        point.z=0.1
        self.publisher.publish(point)
    
def main(args=None):
    rclpy.init(args=args)
    node = visionNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
