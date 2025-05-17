# # To just release servos:
# ./release_servos.py --release

# # To restore defaults and reconfigure:
# ./release_servos.py --restore-defaults

# # To do both:
# ./release_servos.py --release --restore-defaults

# # To specify a different port:
# ./release_servos.py --release --port /dev/ttyACM1




#!/usr/bin/env python3
"""
release_servos.py - A standalone script to release servos using Maestro controller
This script provides functionality to release servos and restore Maestro defaults.
"""

import serial
import subprocess
import os
import time
import sys

# --- Configuration ---
PORT = '/dev/ttyACM0'  # Adjust if needed
BAUDRATE = 115200
CONNECT_RETRIES = 3
CONNECT_DELAY = 2.0  # Seconds

# --- UscCmd Configuration ---
# Adjust paths as needed for your system
USCCMD_DIR = "/home/rishab/Desktop/MiniHumanoid/libraries/pololu-usb-sdk/Maestro/UscCmd"
USCCMD_CMD = "UscCmd"  # No .exe needed

def init_serial(port=PORT, baudrate=BAUDRATE, retries=CONNECT_RETRIES, delay=CONNECT_DELAY):
    """Initialize serial connection to the Maestro controller."""
    for attempt in range(retries):
        try:
            print(f"Attempt {attempt+1}/{retries} to connect to {port}...")
            ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)
            if ser.is_open:
                print(f"Connected to {port}")
                return ser
        except serial.SerialException as e:
            print(f"Connection failed: {e}")
            if attempt < retries - 1:  # Don't sleep on the last attempt
                print(f"Waiting {delay} seconds before retrying...")
                time.sleep(delay)
    
    print(f"Failed to connect to {port} after {retries} attempts.")
    return None

def set_target(ser, channel, target):
    """
    Set target position for a servo channel.
    
    Args:
        ser: Serial connection
        channel: Channel number (0-based)
        target: Target position in quarter-microseconds (0-10000)
        
    Returns:
        True if successful, False otherwise
    """
    if not ser or not ser.is_open:
        print("[ERROR] set_target: Serial connection is not available.")
        return False
    
    try:
        # Command format: 0x84, channel number, target low bits, target high bits
        command = bytes([0x84, channel, target & 0x7F, (target >> 7) & 0x7F])
        ser.write(command)
        return True
    except Exception as e:
        print(f"[ERROR] Failed to set target for channel {channel}: {e}")
        return False

def set_speed(ser, channel, speed):
    """
    Set speed for a servo channel.
    
    Args:
        ser: Serial connection
        channel: Channel number (0-based)
        speed: Speed value (0-255)
        
    Returns:
        True if successful, False otherwise
    """
    if not ser or not ser.is_open:
        print("[ERROR] set_speed: Serial connection is not available.")
        return False
    
    try:
        # Command format: 0x87, channel number, speed low bits, speed high bits
        command = bytes([0x87, channel, speed & 0x7F, (speed >> 7) & 0x7F])
        ser.write(command)
        return True
    except Exception as e:
        print(f"[ERROR] Failed to set speed for channel {channel}: {e}")
        return False

def release_servos(ser):
    """
    Release all servos by setting them to "off" position.
    
    Args:
        ser: Serial connection
        
    Returns:
        True if successful, False otherwise
    """
    if not ser or not ser.is_open:
        print("[ERROR] release_servos: Serial connection is not available.")
        return False

    success = True
    try:
        print("Releasing all servos...")
        for channel in range(17):  # Assuming 17 servos (0-16)
            # The special command 0xFF (255) releases the servo
            if not set_target(ser, channel, 0):
                print(f"Warning: Failed to release channel {channel}")
                success = False
        
        if success:
            print("All servos released successfully.")
        return success
    except Exception as e:
        print(f"[ERROR] Failed to release servos: {e}")
        return False

def restore_maestro_defaults():
    """
    Executes UscCmd --restoredefaults, then --configure to reset the Maestro controller.
    
    Returns:
        True if successful, False otherwise
    """
    env = os.environ.copy()
    # For macOS users who might need this
    env["DYLD_LIBRARY_PATH"] = "/opt/homebrew/Cellar/libusb/1.0.28/lib"
    
    print("--- Starting Restore Defaults Sequence ---")
    print("Step 1: Restoring Maestro defaults via UscCmd...")
    
    try:
        # Restore defaults
        result_restore = subprocess.run(
            ["mono", USCCMD_CMD, "--restoredefaults"],
            cwd=USCCMD_DIR, check=True, capture_output=True, text=True, env=env, timeout=10
        )
        print(f"Step 1 SUCCESS: Restore defaults command successful.")
        if result_restore.stdout:
            print(f"STDOUT: {result_restore.stdout}")
        if result_restore.stderr:
            print(f"STDERR: {result_restore.stderr}")
            
        # Add a delay before configuration
        time.sleep(1.0)
        
        # Configure using the maestro_config.txt file
        print("Step 2: Configuring Maestro via UscCmd...")
        result_config = subprocess.run(
            ["mono", USCCMD_CMD, "--configure", "maestro_config.txt"],
            cwd=USCCMD_DIR, check=True, capture_output=True, text=True, env=env, timeout=15
        )
        print(f"Step 2 SUCCESS: Configure command successful.")
        if result_config.stdout:
            print(f"STDOUT: {result_config.stdout}")
        if result_config.stderr:
            print(f"STDERR: {result_config.stderr}")
            
        print("--- Restore Defaults Sequence Finished Successfully ---")
        return True
        
    except subprocess.CalledProcessError as e:
        print(f"ERROR: Command failed with return code {e.returncode}.")
        print(f"STDOUT: {e.stdout}")
        print(f"STDERR: {e.stderr}")
        return False
    except FileNotFoundError as e:
        print(f"ERROR: File or command not found: {e}")
        print("Make sure mono is installed and UscCmd path is correct.")
        return False
    except subprocess.TimeoutExpired:
        print("ERROR: Command timed out.")
        return False
    except Exception as e:
        print(f"ERROR: Unexpected error: {e}")
        return False

def main():
    """Main function to handle command-line arguments and execute commands."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Release servos and manage Maestro controller.')
    parser.add_argument('--release', action='store_true', help='Release all servos')
    parser.add_argument('--restore-defaults', action='store_true', help='Restore Maestro defaults and reconfigure')
    parser.add_argument('--port', default=PORT, help=f'Serial port (default: {PORT})')
    
    args = parser.parse_args()
    
    if not args.release and not args.restore_defaults:
        parser.print_help()
        print("\nERROR: No action specified. Please specify --release or --restore-defaults")
        sys.exit(1)
    
    # Update port if specified
    global PORT
    PORT = args.port
    
    if args.restore_defaults:
        print(f"Restoring Maestro defaults...")
        success = restore_maestro_defaults()
        if not success:
            print("Failed to restore Maestro defaults!")
            sys.exit(1)
    
    if args.release:
        print(f"Connecting to Maestro on {PORT}...")
        ser = init_serial(port=PORT)
        
        if not ser:
            print(f"Failed to connect to {PORT}. Cannot release servos.")
            sys.exit(1)
        
        try:
            print("Setting initial speeds...")
            for channel in range(17):  # Assuming 17 servos
                set_speed(ser, channel, 30)
            
            print("Releasing servos...")
            success = release_servos(ser)
            
            if success:
                print("Servos released successfully.")
            else:
                print("Failed to release some servos.")
                
        finally:
            print("Closing connection...")
            ser.close()
    
    print("Done.")

if __name__ == "__main__":
    main()
