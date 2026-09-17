class G1JointController:
    def __init__(self, interface_name="en0", domain_id=0):
        """
        Initialize connection to G1 robot.
        Args:
            interface_name: Name of your network interface (e.g., "en0" for Mac, "eth0" for Linux)
            domain_id: DDS domain ID (0 for real robot, often 1 for simulation)[citation:8]
        """
        # Initialize the DDS communication channel
        ChannelFactory.Instance().Init(domain_id, interface_name)
        
        # Create a publisher for low-level motor commands
        self.lowcmd_pub = ChannelFactory.Instance().CreatePublisher(LowCmd, "rt/lowcmd")
        
        # Create a command message
        self.cmd = LowCmd()
        print(f"G1 controller initialized on interface {interface_name}")

    def set_joint_angles(self, angles_list, duration=1.0):
        """
        Set all joint angles.
        Args:
            angles_list: List of 29 target angles (radians) for all joints[citation:8].
            duration: Time to reach target position (seconds).
        """
        # 1. Set the control mode to servo (position control)
        for i in range(len(angles_list)):
            # "set_servo_angle()" is the common method for position control
            # The exact method name may vary; check your SDK's LowCmd class
            self.cmd.set_servo_angle(i, angles_list[i])  # Sets target position for joint i
            self.cmd.set_servo_torque(i, 0.0)            # Sets feedforward torque (often 0)
            self.cmd.set_servo_kp(i, 100.0)              # Position gain (stiffness)
            self.cmd.set_servo_kd(i, 5.0)                # Damping gain

        # 2. Send the command to the robot
        self.lowcmd_pub.Write(self.cmd)
        
        # 3. Wait for the movement
        print(f"Moving joints...")
        time.sleep(duration)

# Example usage
if __name__ == "__main__":
    # Initialize controller (use "lo" for simulation[citation:8])
    controller = G1JointController(interface_name="en0")
    
    # Create a target pose: all joints at 0 radians
    target_angles = [0.0] * 29  # G1 has 29 joints[citation:8]
    
    # Modify some joints as an example
    target_angles[3] = 0.5   # Left knee
    target_angles[9] = 0.5   # Right knee
    
    # Send the command
    controller.set_joint_angles(target_angles, duration=2.0)
