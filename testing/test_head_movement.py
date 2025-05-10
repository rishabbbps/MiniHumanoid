#!/usr/bin/env python3
"""
Head Movement Testing Tool

This script sets up the robot in standing position and then allows
interactive control of the head servo (channel 16) while logging positions.

Usage:
  python head_servo_test.py [--port PORT]
"""

import sys
import time
import os
import argparse
from datetime import datetime

# Add parent directory to path to import standing_pos
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from examples.standing_pos import init_serial, set_target, standing_position, channel_to_joint_map

def log_position(channel, position, log_file=None):
    """Log the current position of the servo."""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    joint_name = channel_to_joint_map.get(channel, f"Unknown Channel {channel}")
    log_message = f"{timestamp} - Channel {channel} ({joint_name}): Position = {position}"
    print(log_message)
    
    if log_file:
        with open(log_file, 'a') as f:
            f.write(log_message + "\n")

def main():
    parser = argparse.ArgumentParser(description='Test head movement (channel 16)')
    parser.add_argument('--port', dest='port', 
                        help='Serial port for the servo controller',
                        default='/dev/cu.usbmodem004763651')
    args = parser.parse_args()
    
    # Setup log file
    log_dir = os.path.dirname(os.path.abspath(__file__))
    log_file = os.path.join(log_dir, f"head_movement_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
    
    print(f"Head Movement Test - Channel 16")
    print(f"Log will be saved to: {log_file}")
    print("Initializing serial connection...")
    
    # Initialize serial connection
    ser = init_serial(port=args.port)
    if not ser:
        print("Failed to initialize serial connection. Exiting.")
        sys.exit(1)
    
    try:
        # Set robot to standing position
        print("Setting robot to standing position...")
        for channel, position in standing_position.items():
            set_target(ser, channel, position)
            time.sleep(0.1)  # Small delay between commands
        
        print("\nHead Movement Test Started")
        print("===========================")
        print(f"Current head position (channel 16): {standing_position[16]}")
        print("Commands:")
        print("  +N  : Increase position by N (e.g., +100)")
        print("  -N  : Decrease position by N (e.g., -100)")
        print("  =N  : Set exact position to N (e.g., =5500)")
        print("  r   : Reset to standing position")
        print("  q   : Quit")
        print("===========================")
        
        # Log initial position
        log_position(16, standing_position[16], log_file)
        
        current_position = standing_position[16]
        
        while True:
            command = input("\nEnter command: ")
            if command.lower() == 'q':
                print("Exiting...")
                break
            elif command.lower() == 'r':
                current_position = standing_position[16]
                set_target(ser, 16, current_position)
                log_position(16, current_position, log_file)
                print(f"Reset to standing position: {current_position}")
            elif command.startswith('+'):
                try:
                    increment = int(command[1:])
                    current_position = min(10000, current_position + increment)
                    set_target(ser, 16, current_position)
                    log_position(16, current_position, log_file)
                except ValueError:
                    print("Invalid increment. Use format: +100")
            elif command.startswith('-'):
                try:
                    decrement = int(command[1:])
                    current_position = max(0, current_position - decrement)
                    set_target(ser, 16, current_position)
                    log_position(16, current_position, log_file)
                except ValueError:
                    print("Invalid decrement. Use format: -100")
            elif command.startswith('='):
                try:
                    new_position = int(command[1:])
                    if 0 <= new_position <= 10000:
                        current_position = new_position
                        set_target(ser, 16, current_position)
                        log_position(16, current_position, log_file)
                    else:
                        print("Position must be between 0 and 10000")
                except ValueError:
                    print("Invalid position. Use format: =5500")
            else:
                print("Unknown command. Available commands: +N, -N, =N, r, q")
    
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    finally:
        print("Closing serial connection...")
        if ser and ser.is_open:
            ser.close()
        print(f"Position log saved to: {log_file}")

if __name__ == "__main__":
    main()
