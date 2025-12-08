import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

class SimpleIKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
        
        self.subscription = self.create_subscription(
            Point,
            '/target_position',
            self.ik_callback,
            10
        )

        self.publisher = self.create_publisher(
            JointState,
            '/joint_commands', 
            10
        )
        
        # Your arm dimensions (CHANGE THESE!)
        self.link_lengths = [0.05, 0.12, 0.09, 0.035]  # L1, L2, L3, L4 in meters
        
        # Joint limits for safety (radians)
        self.joint_limits = [
            (-3.14, 3.14),   # Joint1: ±180°
            (-1.57, 1.57),   # Joint2: ±90°
            (0.0, 2.09),     # Joint3: 0-120° (typical)
            (-1.57, 1.57),   # Joint4: ±90°
            (-3.14, 3.14),   # Joint5: ±180°
        ]
        
        # Current joint angles (for velocity calculation)
        self.current_angles = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.get_logger().info('Simple IK Node Ready!')
    
    def clamp_angle(self, angle, min_val, max_val):
        return max(min(angle, max_val), min_val)
    
    def ik_callback(self, msg):
        x, y, z = msg.x, msg.y, msg.z
        
        # FIX 1: Check if target is above base
        min_z = self.link_lengths[0] + 0.02  # At least 2cm above base
        if z <= min_z:
            self.get_logger().error(
                f"Target too low! z={z:.3f}m must be > {min_z:.3f}m"
            )
            return
        
        # FIX 2: Check if target is too close to base
        r = np.sqrt(x**2 + y**2)
        if r < 0.03:  # At least 3cm from base center
            self.get_logger().error(f"Target too close! r={r:.3f}m must be > 0.03m")
            return
        
        # STEP 1: Calculate base rotation (joint1)
        theta1 = np.arctan2(y, x)
        
        # STEP 2: Calculate vertical plane angles (joint2, joint3, joint4)
        # Project to vertical plane
        r_proj = np.sqrt(x**2 + y**2) - self.link_lengths[3]  # Remove wrist offset
        
        # Height relative to base
        height = z - self.link_lengths[0]  # Subtract base height
        
        # Simple 2-link planar IK (for joints 2-3)
        L2, L3 = self.link_lengths[1], self.link_lengths[2]
        
        # Distance to target in vertical plane
        D = np.sqrt(r_proj**2 + height**2)
        
        # FIX 3: Better reachability check with debugging
        if D > (L2 + L3):
            self.get_logger().warn(
                f"Target too far! D={D:.3f}m > L2+L3={L2+L3:.3f}m"
            )
            return
        if D < abs(L2 - L3):
            self.get_logger().warn(
                f"Target too close! D={D:.3f}m < |L2-L3|={abs(L2-L3):.3f}m"
            )
            return
        
        # FIX 4: Use np.clip to prevent math domain errors
        cos_theta3 = (D**2 - L2**2 - L3**2) / (2 * L2 * L3)
        cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)  # CRITICAL FIX!
        
        # Calculate joint angles using law of cosines
        theta3 = np.arccos(cos_theta3)
        
        alpha = np.arctan2(height, r_proj)
        
        # FIX 5: Also clip here
        cos_beta = (L2**2 + D**2 - L3**2) / (2 * L2 * D)
        cos_beta = np.clip(cos_beta, -1.0, 1.0)
        beta = np.arccos(cos_beta)
        
        # FIX 6: Try both elbow configurations
        theta2_up = alpha - beta    # Elbow up
        theta2_down = alpha + beta  # Elbow down
        
        # Choose configuration with smaller angles
        # Compare max absolute angle of each configuration
        max_up = max(abs(theta2_up), abs(theta3))
        max_down = max(abs(theta2_down), abs(theta3))
        
        if max_up < max_down:
            theta2 = theta2_up
            config = "elbow_up"
        else:
            theta2 = theta2_down
            config = "elbow_down"
        
        # Joint4 keeps end effector vertical
        theta4 = -theta2 - theta3
        
        # Joint5 is gripper rotation (set to 0 for now)
        theta5 = 0.0
        
        # FIX 7: Clamp all angles to safe ranges
        theta1 = self.clamp_angle(theta1, *self.joint_limits[0])
        theta2 = self.clamp_angle(theta2, *self.joint_limits[1])
        theta3 = self.clamp_angle(theta3, *self.joint_limits[2])
        theta4 = self.clamp_angle(theta4, *self.joint_limits[3])
        theta5 = self.clamp_angle(theta5, *self.joint_limits[4])
        
        # FIX 8: Calculate safe velocities
        target_angles = [theta1, theta2, theta3, theta4, theta5]
        velocities = self.calculate_velocities(target_angles)
        angles_deg = [a * 180/np.pi for a in target_angles]
        
        # Create joint message
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        joint_msg.position = [float(theta) for theta in angles_deg]
        joint_msg.velocity = velocities
        
        # Publish all 5 joints
        self.publisher.publish(joint_msg)
        
        # FIX 9: Log in degrees for better understanding
        self.get_logger().info(
            f"IK Solution ({config}): "
            f"[{angles_deg[0]:.1f}°, {angles_deg[1]:.1f}°, "
            f"{angles_deg[2]:.1f}°, {angles_deg[3]:.1f}°, "
            f"{angles_deg[4]:.1f}°]"
        )
        
        # FIX 10: Debug info
        self.get_logger().debug(
            f"Debug: r_proj={r_proj:.3f}m, height={height:.3f}m, D={D:.3f}m"
        )
        
        # Update current angles
        self.current_angles = target_angles
    
    def calculate_velocities(self, target_angles, max_time=2.0):
        velocities = []
        for i in range(5):
            # Angle difference from current position
            angle_diff = abs(target_angles[i] - self.current_angles[i])
            
            # Calculate required velocity (angle/time)
            required_vel = angle_diff / max_time
            
            # Maximum safe velocities (rad/sec)
            max_velocities = [1.0, 0.8, 0.8, 1.0, 1.0]  # Adjust for your servos
            
            # Use whichever is smaller: required or max
            velocity = min(required_vel, max_velocities[i])
            
            # Minimum velocity to ensure movement
            velocity = max(velocity, 0.1)
            
            velocities.append(velocity)
        
        return velocities

def main(args=None):
    rclpy.init(args=args)
    node = SimpleIKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()