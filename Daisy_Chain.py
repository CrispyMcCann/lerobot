#!/usr/bin/env python3
"""
ODrive CAN Bus Motor Test Script
Tests each motor sequentially by moving forward and backward at low speed
"""

import can
import struct
import time

# ODrive CAN Protocol Constants
CMD_SET_AXIS_STATE = 0x07
CMD_SET_INPUT_VEL = 0x0D
CMD_GET_ENCODER_ESTIMATES = 0x09

# Axis States
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

# Configuration
CAN_INTERFACE = 'can1'
BITRATE = 250000
NODE_IDS = [1, 2, 3, 4, 5, 6]  # Your 6 motors
TEST_VELOCITY = 2.0  # turns/sec (slow speed for testing)
TEST_DURATION = 2.0  # seconds for each direction
TORQUE_LIMIT = 0.5  # Nm (low torque limit for safety)

class ODriveCAN:
    def __init__(self, channel='can1'):
        """Initialize CAN bus connection"""
        self.bus = can.interface.Bus(channel=channel, bustype='socketcan', bitrate=BITRATE)
        print(f"✓ Connected to {channel} at {BITRATE} bps")
    
    def send_command(self, node_id, cmd_id, data=b''):
        """Send a CAN command to an ODrive"""
        arbitration_id = (node_id << 5) | cmd_id
        msg = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=False
        )
        self.bus.send(msg)
    
    def set_axis_state(self, node_id, state):
        """Set the axis state (IDLE, CLOSED_LOOP_CONTROL, etc.)"""
        data = struct.pack('<I', state)
        self.send_command(node_id, CMD_SET_AXIS_STATE, data)
        print(f"  Node {node_id}: Setting axis state to {state}")
    
    def set_velocity(self, node_id, velocity, torque_ff=0.0):
        """Set velocity setpoint in turns/sec"""
        data = struct.pack('<ff', velocity, torque_ff)
        self.send_command(node_id, CMD_SET_INPUT_VEL, data)
    
    def test_motor(self, node_id):
        """Test a single motor by moving it forward and backward"""
        print(f"\n{'='*50}")
        print(f"Testing Motor on Node {node_id}")
        print(f"{'='*50}")
        
        try:
            # Step 1: Set to IDLE first (safety)
            print(f"  Step 1: Setting to IDLE state...")
            self.set_axis_state(node_id, AXIS_STATE_IDLE)
            time.sleep(0.5)
            
            # Step 2: Enable closed-loop control
            print(f"  Step 2: Enabling CLOSED_LOOP_CONTROL...")
            self.set_axis_state(node_id, AXIS_STATE_CLOSED_LOOP_CONTROL)
            time.sleep(1.0)  # Wait for control to engage
            
            # Step 3: Move forward
            print(f"  Step 3: Moving FORWARD at {TEST_VELOCITY} turns/sec for {TEST_DURATION}s...")
            self.set_velocity(node_id, TEST_VELOCITY, 0.0)
            time.sleep(TEST_DURATION)
            
            # Step 4: Stop
            print(f"  Step 4: Stopping...")
            self.set_velocity(node_id, 0.0, 0.0)
            time.sleep(0.5)
            
            # Step 5: Move backward
            print(f"  Step 5: Moving BACKWARD at {-TEST_VELOCITY} turns/sec for {TEST_DURATION}s...")
            self.set_velocity(node_id, -TEST_VELOCITY, 0.0)
            time.sleep(TEST_DURATION)
            
            # Step 6: Stop and return to IDLE
            print(f"  Step 6: Stopping and returning to IDLE...")
            self.set_velocity(node_id, 0.0, 0.0)
            time.sleep(0.5)
            self.set_axis_state(node_id, AXIS_STATE_IDLE)
            time.sleep(0.5)
            
            print(f"✓ Node {node_id}: Test completed successfully!")
            
        except Exception as e:
            print(f"✗ Node {node_id}: Error during test - {e}")
            # Try to return to IDLE on error
            try:
                self.set_velocity(node_id, 0.0, 0.0)
                self.set_axis_state(node_id, AXIS_STATE_IDLE)
            except:
                pass
    
    def test_all_motors(self):
        """Test all motors sequentially"""
        print("\n" + "="*50)
        print("ODrive CAN Bus Motor Test")
        print("="*50)
        print(f"Interface: {CAN_INTERFACE}")
        print(f"Node IDs: {NODE_IDS}")
        print(f"Test Velocity: ±{TEST_VELOCITY} turns/sec")
        print(f"Test Duration: {TEST_DURATION}s each direction")
        print("="*50)
        
        for node_id in NODE_IDS:
            self.test_motor(node_id)
            time.sleep(1.0)  # Pause between motors
        
        print("\n" + "="*50)
        print("All motor tests completed!")
        print("="*50 + "\n")
    
    def close(self):
        """Close the CAN bus connection"""
        self.bus.shutdown()
        print("✓ CAN bus connection closed")


def main():
    """Main function"""
    print("\n" + "="*50)
    print("ODRIVE CAN MOTOR TEST")
    print("="*50)
    print("\n⚠️  WARNING: Motors will move!")
    print("Make sure:")
    print("  1. Motors are properly mounted and secured")
    print("  2. There are no obstructions")
    print("  3. You can quickly disconnect power if needed")
    print("\nPress Ctrl+C to abort at any time\n")
    
    try:
        input("Press ENTER to start the test...")
    except KeyboardInterrupt:
        print("\n✗ Test aborted by user")
        return
    
    try:
        # Initialize ODrive CAN interface
        odrive = ODriveCAN(channel=CAN_INTERFACE)
        
        # Run the test sequence
        odrive.test_all_motors()
        
        # Clean up
        odrive.close()
        
    except KeyboardInterrupt:
        print("\n\n✗ Test interrupted by user (Ctrl+C)")
        try:
            # Emergency stop - try to idle all motors
            print("Attempting emergency stop...")
            for node_id in NODE_IDS:
                odrive.set_velocity(node_id, 0.0, 0.0)
                odrive.set_axis_state(node_id, AXIS_STATE_IDLE)
            odrive.close()
        except:
            pass
    
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
