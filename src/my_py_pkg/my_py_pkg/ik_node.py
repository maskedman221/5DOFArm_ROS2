import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import math

PI = 3.14159265359
HALF_PI = 1.5707963268
DOUBLE_PI = 6.28318530718
DEGREE_STEP = 0.01745329251
FREE_ANGLE = 999.9

class Link:
    def __init__(self):
        self._length = 0.0
        self._angleLow = 0.0
        self._angleHigh = 0.0
        self._angle = 0.0

    def init(self, length, angle_low_limit, angle_high_limit):
        self._length = length
        self._angleLow = angle_low_limit
        self._angleHigh = angle_high_limit

    def inRange(self, angle):
        return self._angleLow <= angle <= self._angleHigh

    def getLength(self):
        return self._length

    def getAngle(self):
        return self._angle

    def setAngle(self, angle):
        self._angle = angle


class Inverse:
    def __init__(self):
        self._L0 = Link()
        self._L1 = Link()
        self._L2 = Link()
        self._L3 = Link()
        self._currentPhi = -DOUBLE_PI

    def attach(self, shoulder, upperarm, forearm, hand):
        self._L0 = shoulder
        self._L1 = upperarm
        self._L2 = forearm
        self._L3 = hand

    def _cosrule(self, opposite, adjacent1, adjacent2):
        delta = 2 * adjacent1 * adjacent2
        if delta == 0:
            return None
        cos_val = (adjacent1**2 + adjacent2**2 - opposite**2) / delta
        if cos_val > 1 or cos_val < -1:
            return None
        return math.acos(cos_val)

    def _solve_fixed_phi(self, x, y, phi):
        # wrist coordinates
        xw = x + self._L3.getLength() * math.cos(phi)
        yw = y + self._L3.getLength() * math.sin(phi)

        R = math.sqrt(xw**2 + yw**2)

        beta = self._cosrule(self._L2.getLength(), R, self._L1.getLength())
        if beta is None:
            return None
        gamma = self._cosrule(R, self._L1.getLength(), self._L2.getLength())
        if gamma is None:
            return None

        alpha = math.atan2(yw, xw)
        shoulder = alpha + beta
        elbow = gamma
        wrist = PI - shoulder - elbow + phi

        return shoulder, elbow, wrist

    def _solve_free_phi(self, x, y):
        for phi in frange(-DOUBLE_PI, DOUBLE_PI, DEGREE_STEP):
            result = self._solve_fixed_phi(x, y, phi)
            if result is not None:
                self._currentPhi = phi
                return result
        return None

    def solve(self, x, y, z, phi=FREE_ANGLE):
        _r = math.sqrt(x**2 + y**2)
        _base = math.atan2(y, x)

        if not self._L0.inRange(_base):
            _base += PI if _base < 0 else -PI
            _r *= -1
            if phi != FREE_ANGLE:
                phi = PI - phi

        if phi == FREE_ANGLE:
            result = self._solve_free_phi(_r, z - self._L0.getLength())
        else:
            result = self._solve_fixed_phi(_r, z - self._L0.getLength(), phi)

        if result is None:
            return None

        shoulder, elbow, wrist = result
        return _base, shoulder, elbow, wrist


def frange(start, stop, step):
    while start < stop:
        yield start
        start += step


# Conversion helpers
def b2a(b):
    return b / 180.0 * PI

def a2b(a):
    return a * 180.0 / PI

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
        self.link_lengths = [0.125, 0.12, 0.12, 0.035]  # L1, L2, L3, L4 in meters
        
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
        x, y, z = msg.x * 1000, msg.y * 1000, msg.z * 1000
        base = Link()
        upperarm = Link()
        forearm = Link()
        hand = Link()

        base.init(130, b2a(0.0), b2a(180.0))
        upperarm.init(130, b2a(-45.0), b2a(165.0))
        forearm.init(130, b2a(45.0), b2a(300.0))
        hand.init(100, b2a(90.0), b2a(270.0))

        inv = Inverse()
        inv.attach(base, upperarm, forearm, hand)

        result = inv.solve(x, y, z, b2a(90.0))
        if result:
            a0, a1, a2, a3 = result
            print(a2b(a0), a2b(a1), a2b(a2), a2b(a3))
        else:
            print("No solution found!")
        target_angles = [0,80, 85, 0 , 180]
        # # FIX 1: Check if target is above base
        # min_z = self.link_lengths[0] + 0.02  # At least 2cm above base
        # if z <= 0:
        #     self.get_logger().error(
        #         f"Target too low! z={z:.3f}m must be > {min_z:.3f}m"
        #     )
        #     return
        
        # # FIX 2: Check if target is too close to base
        # r = np.sqrt(x**2 + y**2)
        # if r < 0.03:  # At least 3cm from base center
        #     self.get_logger().error(f"Target too close! r={r:.3f}m must be > 0.03m")
        #     return
        
        # # STEP 1: Calculate base rotation (joint1)
        # theta1 = np.arctan2(y, x)
        
        # # STEP 2: Calculate vertical plane angles (joint2, joint3, joint4)
        # # Project to vertical plane
        # r_proj = np.sqrt(x**2 + y**2) - self.link_lengths[3]  # Remove wrist offset
        
        # # Height relative to base
        # height = z - self.link_lengths[0]  # Subtract base height
        
        # # Simple 2-link planar IK (for joints 2-3)
        # L2, L3 = self.link_lengths[1], self.link_lengths[2]
        
        # # Distance to target in vertical plane
        # D = np.sqrt(r_proj**2 + height**2)
        
        # # FIX 3: Better reachability check with debugging
        # if D > (L2 + L3):
        #     self.get_logger().warn(
        #         f"Target too far! D={D:.3f}m > L2+L3={L2+L3:.3f}m"
        #     )
        #     return
        # if D < abs(L2 - L3):
        #     self.get_logger().warn(
        #         f"Target too close! D={D:.3f}m < |L2-L3|={abs(L2-L3):.3f}m"
        #     )
        #     return
        
        # # FIX 4: Use np.clip to prevent math domain errors
        # cos_theta3 = (D**2 - L2**2 - L3**2) / (2 * L2 * L3)
        # cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)  # CRITICAL FIX!
        
        # # Calculate joint angles using law of cosines
        # theta3 = np.arccos(cos_theta3)
        
        # alpha = np.arctan2(height, r_proj)
        
        # # FIX 5: Also clip here
        # cos_beta = (L2**2 + D**2 - L3**2) / (2 * L2 * D)
        # cos_beta = np.clip(cos_beta, -1.0, 1.0)
        # beta = np.arccos(cos_beta)
        
        # # FIX 6: Try both elbow configurations
        # theta2_up = alpha - beta    # Elbow up
        # theta2_down = alpha + beta  # Elbow down
        
        # # Choose configuration with smaller angles
        # # Compare max absolute angle of each configuration
        # max_up = max(abs(theta2_up), abs(theta3))
        # max_down = max(abs(theta2_down), abs(theta3))
        
        # if max_up < max_down:
        #     theta2 = theta2_up
        #     config = "elbow_up"
        # else:
        #     theta2 = theta2_down
        #     config = "elbow_down"
        
        # # Joint4 keeps end effector vertical
        # theta4 = -theta2 - theta3
        
        # # Joint5 is gripper rotation (set to 0 for now)
        # theta5 = 0.0
        
        # # FIX 7: Clamp all angles to safe ranges
        # theta1 = self.clamp_angle(theta1, *self.joint_limits[0])
        # theta2 = self.clamp_angle(theta2, *self.joint_limits[1])
        # theta3 = self.clamp_angle(theta3, *self.joint_limits[2])
        # theta4 = self.clamp_angle(theta4, *self.joint_limits[3])
        # theta5 = self.clamp_angle(theta5, *self.joint_limits[4])
        
        # # FIX 8: Calculate safe velocities
        # target_angles = [theta1, theta2, theta3, theta4, theta5]
        velocities = self.calculate_velocities(target_angles)
        # angles_deg = [a * 180/np.pi for a in target_angles]
        
        # Create joint message
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        joint_msg.position = [float(theta) for theta in target_angles]
        joint_msg.velocity = velocities
        
        # Publish all 5 joints
        self.publisher.publish(joint_msg)
        
        # FIX 9: Log in degrees for better understanding
        self.get_logger().info(
            f"IK Solution ({"config"}): "
            f"[{a2b(a0):.1f}°, {a2b(a1):.1f}°, "
            f"{a2b(a2):.1f}°, {a2b(a3):.1f}°, "
            # f"{angles_deg[4]:.1f}°]"
        )
        
        # FIX 10: Debug info
        # self.get_logger().debug(
        #     f"Debug: r_proj={r_proj:.3f}m, height={height:.3f}m, D={D:.3f}m"
        # )
        
        # Update current angles
        self.current_angles = target_angles
    
    def calculate_velocities(self, target_angles, max_time=2.0):
        velocities = []
        for i in range(4):
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
    