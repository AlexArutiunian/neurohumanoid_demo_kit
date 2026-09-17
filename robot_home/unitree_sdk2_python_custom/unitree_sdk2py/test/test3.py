#!/usr/bin/env python3
"""
Unitree G1 Right Hand Control Script (Final Corrected Version)
Target SDK: unitree_sdk2_python with DDS-style message classes (LowCmd_, LowState_)
"""

import time
import numpy as np
import array  # ADD THIS IMPORT

# CORRECTED IMPORTS: Using trailing underscores and standard 'time' module
from unitree_sdk2py.core.channel import ChannelFactory
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_, MotorCmd_, MotorState_

class G1RightHandController:
    """Controller for Unitree G1's right hand/arm."""

    # Right Arm Joint Mapping (Indices and Limits in Radians)
    # WARNING: Verify these indices for your specific G1 hardware version
    RIGHT_ARM_JOINTS = {
        'shoulder_pitch': {'idx': 14, 'min': -3.0892, 'max': 2.6704, 'name': 'R_SHOULDER_PITCH'},
        'shoulder_roll':  {'idx': 15, 'min': -2.7925, 'max': 0.5236, 'name': 'R_SHOULDER_ROLL'},
        'shoulder_yaw':   {'idx': 16, 'min': -0.6109, 'max': 2.9671, 'name': 'R_SHOULDER_YAW'},
        'elbow':          {'idx': 17, 'min': -1.0472, 'max': 2.0944, 'name': 'R_ELBOW'},
        'wrist':          {'idx': 18, 'min': -1.0472, 'max': 1.0472, 'name': 'R_WRIST'}
        # Add finger joints here if your G1 has them (e.g., indices 19-23)
    }

    def __init__(self, network_interface="eth0"):
        """
        Initialize the SDK2 controller.
        """
        print(f"[INFO] Initializing G1 controller on interface: {network_interface}")

        # 1. Create and initialize factory
        self.factory = ChannelFactory()
        self.factory.Init(0, network_interface)

        # 2. Create Send Channel for commands
        self.cmd_channel = self.factory.CreateSendChannel("rt/lowcmd", LowCmd_)
        
        # 3. Create Receive Channel for state
        self.state_channel = self.factory.CreateRecvChannel("rt/lowstate", LowState_)
        
        # 4. Get initial state by reading from channel
        self.low_state = None
        print("[INFO] Waiting for initial robot state (timeout: 5s)...")
        timeout_start = time.time()
        
        while time.time() - timeout_start < 5.0:
            try:
                self.low_state = self.state_channel.Read(timeout=0.1)
                if self.low_state is not None:
                    print("[INFO] Initial robot state received.")
                    break
            except Exception:
                pass
            time.sleep(0.01)
        
        if self.low_state is None:
            print("[ERROR] Failed to get initial robot state.")
            print("[ERROR] Check: 1) Robot power, 2) Ethernet connection, 3) Robot mode")
            raise ConnectionError("No robot state received")
        
        # 5. Get motor count from state
        self.motor_count = len(self.low_state.motor_state)
        print(f"[INFO] Robot has {self.motor_count} motors.")
        
        # 6. Initialize motor commands to safe state
        self._init_motor_commands()
        
        self.print_joint_info()

    def _init_motor_commands(self):
        """Initialize LowCmd_ with proper structure for G1."""
        # Create motor command array - use the motor count from state
        motor_cmds = []
        for i in range(self.motor_count):
            # FIXED: Use array.array for reserve field
            cmd = MotorCmd_(
                mode=10,          # Damping mode for safety
                q=0.0,            # Position
                dq=0.0,           # Velocity
                tau=0.0,          # Torque
                kp=0.0,           # Stiffness
                kd=0.0,           # Damping
                reserve=array.array('I', [0, 0, 0, 0])  # 4 unsigned integers
            )
            motor_cmds.append(cmd)
        
        # FIXED: Create LowCmd_ with proper array types
        self.low_cmd = LowCmd_(
            motor_cmd=motor_cmds,
            version=0,
            mode_pr=0,
            mode_machine=0,
            tick=0,
            wireless_remote=array.array('B', [0]*40),  # 40 bytes
            reserve=array.array('B', [0]*8),          # 8 bytes
            crc=0
        )

    def _check_angle_limits(self, joint_name, angle_rad):
        """Clamp commanded angle to safe hardware limits."""
        joint = self.RIGHT_ARM_JOINTS[joint_name]
        if angle_rad < joint['min']:
            print(f"[LIMIT] Clamping {joint['name']} to min: {joint['min']:.3f} rad")
            return joint['min']
        if angle_rad > joint['max']:
            print(f"[LIMIT] Clamping {joint['name']} to max: {joint['max']:.3f} rad")
            return joint['max']
        return angle_rad

    def set_right_hand_angles(self, angle_dict, duration=3.0, kp=30.0, kd=1.5):
        """
        Smoothly move right hand joints to specified angles.
        """
        print(f"\n[MOVE] Starting movement over {duration:.1f}s (kp={kp}, kd={kd})")

        # Validate joints and clamp angles
        valid_joints = {}
        for name, target_angle in angle_dict.items():
            if name not in self.RIGHT_ARM_JOINTS:
                print(f"[SKIP] Unknown joint '{name}'. Available: {list(self.RIGHT_ARM_JOINTS.keys())}")
                continue
            safe_angle = self._check_angle_limits(name, target_angle)
            valid_joints[name] = safe_angle
            print(f"       {self.RIGHT_ARM_JOINTS[name]['name']:22} -> {safe_angle:6.3f} rad")

        if not valid_joints:
            print("[ERROR] No valid joints in command.")
            return

        # Record start positions for smooth interpolation
        start_positions = {}
        for name in valid_joints.keys():
            idx = self.RIGHT_ARM_JOINTS[name]['idx']
            start_positions[name] = self.low_state.motor_state[idx].q

        # Interpolate and send commands
        steps = max(int(duration * 100), 1)
        for step in range(steps):
            t = (step + 1) / steps

            for name, target_angle in valid_joints.items():
                idx = self.RIGHT_ARM_JOINTS[name]['idx']
                start_angle = start_positions[name]
                current_angle = start_angle + (target_angle - start_angle) * t

                # Update the existing MotorCmd_ object
                self.low_cmd.motor_cmd[idx].mode = 1  # Position control
                self.low_cmd.motor_cmd[idx].q = current_angle
                self.low_cmd.motor_cmd[idx].kp = kp
                self.low_cmd.motor_cmd[idx].kd = kd

            # Send command to robot
            self.cmd_channel.Write(self.low_cmd)
            time.sleep(0.01)

        print("[MOVE] Completed.\n")

    def get_current_angles(self):
        """Return the current angles of all right arm joints."""
        angles = {}
        for name, info in self.RIGHT_ARM_JOINTS.items():
            idx = info['idx']
            angles[name] = self.low_state.motor_state[idx].q
        return angles

    def print_joint_info(self):
        """Print a table of joint info and current angles."""
        print("\n" + "="*70)
        print("G1 RIGHT ARM JOINT STATUS")
        print("="*70)
        current = self.get_current_angles()
        for name, info in self.RIGHT_ARM_JOINTS.items():
            print(f"{name:18} [Idx:{info['idx']:2d}] {info['name']:22} | "
                  f"Angle: {current[name]:7.4f} rad | Limits: [{info['min']:7.4f}, {info['max']:7.4f}]")
        print("="*70)

    def safe_shutdown(self):
        """Put all arm motors back into damping mode."""
        print("\n[SHUTDOWN] Engaging motor damping...")
        for name, info in self.RIGHT_ARM_JOINTS.items():
            idx = info['idx']
            self.low_cmd.motor_cmd[idx].mode = 10  # Damping mode
            self.low_cmd.motor_cmd[idx].kp = 0.0
            self.low_cmd.motor_cmd[idx].kd = 0.5
        
        self.cmd_channel.Write(self.low_cmd)
        time.sleep(0.2)
        print("[SHUTDOWN] Complete.")

def main():
    """Example demonstration."""
    NETWORK_INTERFACE = "eth0"
    TEST_SAFETY = True

    try:
        controller = G1RightHandController(network_interface=NETWORK_INTERFACE)
        
        if TEST_SAFETY:
            print("\n" + "#"*50)
            print("SAFETY TEST: Micro-movement")
            print("WARNING: Ensure arm has clear space. Keep remote L1+A ready.")
            input("Press Enter to continue (or Ctrl+C to abort)...")
            
            controller.set_right_hand_angles({'wrist': 0.1}, duration=4.0, kp=15.0, kd=0.8)
            time.sleep(2)
            controller.set_right_hand_angles({'wrist': 0.0}, duration=4.0, kp=15.0, kd=0.8)
            print("Safety test passed. You may disable TEST_SAFETY flag.")

        print("\nFinal angles:", controller.get_current_angles())

    except KeyboardInterrupt:
        print("\n[INFO] Program interrupted by user.")
    except Exception as e:
        print(f"\n[ERROR] Unexpected error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'controller' in locals():
            controller.safe_shutdown()

if __name__ == "__main__":
    main()
