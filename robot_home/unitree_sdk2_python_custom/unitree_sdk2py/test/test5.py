#!/usr/bin/env python3
"""
Unitree G1 Joint Angle Controller
Based on unitree_sdk2_python SDK
Date: 2026-01-31
"""

import time
import numpy as np

# Main import from unitree_sdk2_python
try:
    from unitree_sdk2py.core.robot import Robot
    from unitree_sdk2py.core.joint_command import JointCmd
    from unitree_sdk2py.core.robot_state import RobotState
    print("unitree_sdk2_python imported successfully")
except ImportError as e:
    print(f"Error importing unitree_sdk2_python: {e}")
    print("Please install with: pip install unitree_sdk2_python")
    print("If not available via pip, clone from the official repository")
    exit(1)

class G1JointController:
    """Controller for Unitree G1 humanoid robot joints using unitree_sdk2_python"""
    
    def __init__(self, robot_ip="192.168.123.161", port=50051):
        """
        Initialize connection to G1 robot.
        
        Args:
            robot_ip: IP address of G1 robot (default: 192.168.123.161 for development PC1)[citation:7]
            port: Port number for connection
        """
        print(f"Connecting to G1 robot at {robot_ip}:{port}...")
        
        # Initialize robot connection
        self.robot = Robot()
        
        try:
            # Connect to robot
            self.robot.connect(robot_ip, port)
            print("Connected successfully to G1 robot")
            
            # Get robot state interface
            self.state = self.robot.get_state()
            
            # Initialize joint command
            self.joint_cmd = JointCmd()
            self.joint_cmd.name = "joint_cmd"
            self.joint_cmd.topic = "joint_cmd"
            
            # Joint configuration for G1
            # G1 EDU Plus (U2) has 29 DOF with enhanced waist and 7 DOF arms[citation:9]
            self.num_joints = 29
            
            # Control parameters
            self.default_kp = 100.0  # Position gain
            self.default_kd = 5.0    # Damping gain
            self.default_tau = 0.0   # Torque
            
            # Joint limits in radians (approximate values - adjust based on your G1 model)
            self.joint_limits_min = np.array([
                # Left leg (0-5)
                -2.53, -0.52, -2.76, -0.09, -0.87, -0.26,
                # Right leg (6-11)
                -2.53, -2.97, -2.76, -0.09, -0.87, -0.26,
                # Waist (12-14)
                -2.62, -0.52, -0.52,
                # Left arm (15-21)
                -3.09, -1.59, -2.62, -1.05, -1.97, -1.61, -1.61,
                # Right arm (22-28)
                -3.09, -2.25, -2.62, -1.05, -1.97, -1.61, -1.61
            ])
            
            self.joint_limits_max = np.array([
                # Left leg (0-5)
                2.88, 2.97, 2.76, 2.88, 0.52, 0.26,
                # Right leg (6-11)
                2.88, 0.52, 2.76, 2.88, 0.52, 0.26,
                # Waist (12-14)
                2.62, 0.52, 0.52,
                # Left arm (15-21)
                2.67, 2.25, 2.62, 2.09, 1.97, 1.61, 1.61,
                # Right arm (22-28)
                2.67, 1.59, 2.62, 2.09, 1.97, 1.61, 1.61
            ])
            
            # Emergency stop flag
            self.emergency_stop = False
            
        except Exception as e:
            print(f"Failed to connect to robot: {e}")
            raise
    
    def validate_joint_angles(self, angles):
        """
        Validate joint angles against limits.
        
        Args:
            angles: List/numpy array of joint angles (radians)
            
        Returns:
            tuple: (is_valid, error_message)
        """
        if len(angles) != self.num_joints:
            return False, f"Expected {self.num_joints} joints, got {len(angles)}"
        
        for i, angle in enumerate(angles):
            if angle < self.joint_limits_min[i] or angle > self.joint_limits_max[i]:
                return False, f"Joint {i} angle {angle:.3f} outside limits [{self.joint_limits_min[i]:.3f}, {self.joint_limits_max[i]:.3f}]"
        
        return True, ""
    
    def set_joint_angles(self, angles, duration=2.0, kp=None, kd=None):
        """
        Set all joint angles using position control.
        
        Args:
            angles: List of 29 joint angles in radians
            duration: Time to reach target position (seconds)
            kp: Position gain (optional)
            kd: Damping gain (optional)
            
        Returns:
            bool: True if successful, False otherwise
        """
        if self.emergency_stop:
            print("Emergency stop active! Call reset_emergency_stop() first.")
            return False
        
        # Validate angles
        is_valid, error_msg = self.validate_joint_angles(angles)
        if not is_valid:
            print(f"Invalid joint angles: {error_msg}")
            return False
        
        try:
            # Convert to numpy array if needed
            if not isinstance(angles, np.ndarray):
                angles = np.array(angles, dtype=np.float32)
            
            # Set control parameters
            kp_val = kp if kp is not None else self.default_kp
            kd_val = kd if kd is not None else self.default_kd
            
            print(f"Setting {len(angles)} joints to target angles...")
            print(f"Control params: kp={kp_val}, kd={kd_val}")
            
            # Send joint commands
            for joint_idx in range(len(angles)):
                # Create joint command
                cmd_data = {
                    'mode': 1,  # Position control mode
                    'q': float(angles[joint_idx]),
                    'dq': 0.0,  # Target velocity
                    'tau': self.default_tau,
                    'kp': kp_val,
                    'kd': kd_val,
                    'reserve': [0.0, 0.0, 0.0]
                }
                
                # Send command for this joint
                self.robot.set_joint_cmd(joint_idx, cmd_data)
            
            # Wait for movement to complete
            print(f"Moving to target position (duration: {duration}s)...")
            time.sleep(duration)
            
            print("Joint angles set successfully")
            return True
            
        except Exception as e:
            print(f"Error setting joint angles: {e}")
            return False
    
    def set_single_joint(self, joint_index, angle, kp=None, kd=None):
        """
        Set angle for a single joint.
        
        Args:
            joint_index: Index of joint (0-28)
            angle: Target angle in radians
            kp: Position gain (optional)
            kd: Damping gain (optional)
            
        Returns:
            bool: True if successful, False otherwise
        """
        if joint_index < 0 or joint_index >= self.num_joints:
            print(f"Invalid joint index: {joint_index}. Must be 0-{self.num_joints-1}")
            return False
        
        # Check joint limits
        if angle < self.joint_limits_min[joint_index] or angle > self.joint_limits_max[joint_index]:
            print(f"Joint {joint_index} angle {angle:.3f} outside limits")
            return False
        
        try:
            kp_val = kp if kp is not None else self.default_kp
            kd_val = kd if kd is not None else self.default_kd
            
            cmd_data = {
                'mode': 1,
                'q': float(angle),
                'dq': 0.0,
                'tau': self.default_tau,
                'kp': kp_val,
                'kd': kd_val,
                'reserve': [0.0, 0.0, 0.0]
            }
            
            self.robot.set_joint_cmd(joint_index, cmd_data)
            print(f"Set joint {joint_index} to {angle:.3f} rad")
            return True
            
        except Exception as e:
            print(f"Error setting joint {joint_index}: {e}")
            return False
    
    def get_current_joint_angles(self):
        """
        Get current joint angles from robot state.
        
        Returns:
            numpy array: Current joint angles in radians, or None if failed
        """
        try:
            # Get robot state
            state_data = self.robot.get_state_data()
            
            # Extract joint positions (q)
            # The exact structure may vary - adjust based on your SDK version
            if hasattr(state_data, 'motor_state'):
                # If using motor_state structure
                angles = np.zeros(self.num_joints)
                for i in range(self.num_joints):
                    if i < len(state_data.motor_state):
                        angles[i] = state_data.motor_state[i].q
                return angles
            else:
                print("Warning: Could not extract joint angles from state")
                return None
                
        except Exception as e:
            print(f"Error getting joint angles: {e}")
            return None
    
    def set_standing_pose(self):
        """
        Set G1 to a basic standing pose.
        All joints at neutral positions.
        """
        print("Setting standing pose...")
        
        # Create standing pose (all joints at 0, slight bend in knees)
        standing_angles = np.zeros(self.num_joints)
        
        # Slight bend in knees for stability
        standing_angles[3] = 0.1   # Left knee
        standing_angles[9] = 0.1   # Right knee
        
        # Arms slightly forward
        standing_angles[15] = 0.2   # Left shoulder pitch
        standing_angles[22] = 0.2   # Right shoulder pitch
        
        # Elbows slightly bent
        standing_angles[18] = -0.3  # Left elbow
        standing_angles[25] = -0.3  # Right elbow
        
        return self.set_joint_angles(standing_angles, duration=3.0)
    
    def set_sitting_pose(self):
        """
        Set G1 to a sitting-like pose.
        """
        print("Setting sitting pose...")
        
        sitting_angles = np.zeros(self.num_joints)
        
        # Bend knees
        sitting_angles[3] = 1.5   # Left knee
        sitting_angles[9] = 1.5   # Right knee
        
        # Hip pitch
        sitting_angles[0] = -0.5  # Left hip pitch
        sitting_angles[6] = -0.5  # Right hip pitch
        
        # Keep torso upright
        sitting_angles[14] = 0.1  # Waist pitch slightly forward
        
        return self.set_joint_angles(sitting_angles, duration=3.0)
    
    def emergency_stop(self):
        """
        Emergency stop - set all joints to zero torque.
        """
        print("EMERGENCY STOP ACTIVATED!")
        self.emergency_stop = True
        
        try:
            for joint_idx in range(self.num_joints):
                cmd_data = {
                    'mode': 5,  # Torque control mode
                    'q': 0.0,
                    'dq': 0.0,
                    'tau': 0.0,  # Zero torque
                    'kp': 0.0,
                    'kd': 0.0,
                    'reserve': [0.0, 0.0, 0.0]
                }
                self.robot.set_joint_cmd(joint_idx, cmd_data)
            
            print("All joints set to zero torque")
            
        except Exception as e:
            print(f"Error in emergency stop: {e}")
    
    def reset_emergency_stop(self):
        """
        Reset emergency stop flag.
        """
        self.emergency_stop = False
        print("Emergency stop reset")
    
    def disconnect(self):
        """
        Safely disconnect from robot.
        """
        print("Disconnecting from robot...")
        try:
            self.robot.disconnect()
            print("Disconnected successfully")
        except Exception as e:
            print(f"Error disconnecting: {e}")

# Example usage function
def example_usage():
    """
    Demonstrate how to use the G1 joint controller.
    """
    print("=== Unitree G1 Joint Controller Example ===")
    
    # Initialize controller
    # Note: The G1 typically uses IP 192.168.123.161 for PC1[citation:7]
    controller = G1JointController(robot_ip="192.168.123.161", port=50051)
    
    try:
        # Example 1: Get current joint angles
        print("\n1. Getting current joint angles...")
        current_angles = controller.get_current_joint_angles()
        if current_angles is not None:
            print(f"Current angles (first 5): {current_angles[:5]}")
        
        # Example 2: Set standing pose
        print("\n2. Setting standing pose...")
        controller.set_standing_pose()
        time.sleep(2)
        
        # Example 3: Set specific joint angles
        print("\n3. Setting specific joint angles...")
        
        # Create a custom pose (example: raise arms)
        custom_angles = np.zeros(29)
        
        # Copy current angles if available
        if current_angles is not None:
            custom_angles = current_angles.copy()
        
        # Modify some joints
        custom_angles[15] = 0.5   # Left shoulder pitch (raise arm forward)
        custom_angles[22] = 0.5   # Right shoulder pitch
        
        # Set the custom pose
        success = controller.set_joint_angles(custom_angles, duration=2.0)
        
        if success:
            print("Custom pose set successfully")
            time.sleep(2)
        
        # Example 4: Single joint control
        print("\n4. Controlling single joint...")
        controller.set_single_joint(18, -0.5)  # Left elbow bend
        time.sleep(1)
        
        # Example 5: Return to standing
        print("\n5. Returning to standing pose...")
        controller.set_standing_pose()
        
        print("\nExample completed successfully!")
        
    except KeyboardInterrupt:
        print("\n\nProgram interrupted by user")
    except Exception as e:
        print(f"\nError during example: {e}")
    finally:
        # Always disconnect properly
        print("\nDisconnecting...")
        controller.disconnect()

# Main entry point
if __name__ == "__main__":
    # Test import first
    try:
        import unitree_sdk2_python
        print("unitree_sdk2_python is available")
    except ImportError:
        print("unitree_sdk2_python not found. Installation options:")
        print("1. Clone from repository: git clone https://github.com/unitreerobotics/unitree_sdk2_python")
        print("2. Check Weston Robot documentation for installation[citation:7]")
        print("3. Note: Official releases might not be on PyPI yet[citation:1]")
        exit(1)
    
    # Run example
    example_usage()
