import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.node import Node
# Correct imports for the Unitree G1:
from unitree_hg.msg import LowState, LowCmd

class G1StateMonitor(Node):
    def __init__(self):
        super().__init__('g1_state_monitor')
        
        # Create a QoS profile that matches the RELIABLE publisher
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth=10
        )

        # Subscriber for core state
        self.lowstate_sub = self.create_subscription(
            LowState,
            '/lf/lowstate',
            self.lowstate_callback,
            qos_profile
        )
        
        self.get_logger().info("G1 State Monitor node started. Waiting for data...")
    
    def lowstate_callback(self, msg):
        # Example: Check first joint position
        self.get_logger().info(f'Joint 0 position: {msg.joint_position[0]:.3f} rad')
        # Example: Check IMU orientation (quaternion)
        self.get_logger().info(f'IMU Quat: w={msg.imu_state.quaternion[0]:.3f}')
    
    # The following methods are not used but kept for structure
    def bms_callback(self, msg):
        pass
 
    def mode_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = G1StateMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
