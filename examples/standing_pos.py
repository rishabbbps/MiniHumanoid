import serial
import time
import sys

# Serial port configuration with proper timeouts and settings from pySerial docs
PORT = '/dev/cu.usbmodem004763651'  # Default port, can be overridden  
BAUDRATE = 115200  # Standard baud rate that works reliably
TIMEOUT = 1.0  # Seconds (None would block forever, 0 is non-blocking)
PARITY = serial.PARITY_NONE
STOPBITS = serial.STOPBITS_ONE
BYTESIZE = serial.EIGHTBITS
INTER_BYTE_TIMEOUT = 0.5  # Time between characters
WRITE_TIMEOUT = 1.0
# Flow control disabled since we're not using hardware flow control
RTSCTS = False  
XONXOFF = False

# No global serial object here; it will be passed into functions.
#   git config --global user.email "rishabbbps@gmail.com"
#   git config --global user.name "Rishab"
def init_serial(port=PORT, baudrate=BAUDRATE, retries=3, delay=2.0):
    """
    Initialize serial connection with configured parameters and retries.
    Returns the serial object on success, None on failure.
    """
    ser = None
    for attempt in range(retries):
        try:
            print(f"Attempting connection to {port} (try {attempt + 1}/{retries})...")
            ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                parity=PARITY,
                stopbits=STOPBITS,
                bytesize=BYTESIZE,
                timeout=TIMEOUT,
                inter_byte_timeout=INTER_BYTE_TIMEOUT,
                write_timeout=WRITE_TIMEOUT,
                exclusive=True  # Request exclusive access if supported
            )
            # Check if the port is actually open
            if ser.is_open:
                print(f"Successfully connected to serial port {ser.name}")
                # Clear any old data
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                return ser
            else:
                # This case should ideally not happen with successful Serial() call
                print(f"Connection attempt {attempt + 1} failed: Port not open.")
                if ser:
                    ser.close() # Ensure it's closed if partially opened
                ser = None # Reset ser object

        except serial.SerialException as e:
            print(f"Serial connection error (attempt {attempt + 1}/{retries}): {e}")
            # Ensure serial object is None if connection failed
            if ser:
                try:
                    ser.close()
                except: # Ignore errors during close on failure
                    pass
                ser = None
        except ValueError as e:
            print(f"Serial configuration error (attempt {attempt + 1}/{retries}): {e}")
            ser = None # Ensure serial object is None

        # Wait before retrying, unless it's the last attempt
        if attempt < retries - 1:
            print(f"Waiting {delay} seconds before retrying...")
            time.sleep(delay)

    print(f"Failed to connect to {port} after {retries} attempts.")
    return None # Return None if all retries fail

# Helper functions - Now accept 'ser' as the first argument
def set_speed(ser, channel, speed):
    """Set servo speed. Requires an active serial connection object."""
    if not ser or not ser.is_open:
        print("[ERROR] set_speed: Serial connection is not open.")
        return False
    try:
        # Using Pololu Protocol: 0xAA, device_id, 0x07, channel, speed_lsb, speed_msb
        # Assuming default device ID 12 (0x0C)
        speed = int(speed)
        lsb = speed & 0x7F
        msb = (speed >> 7) & 0x7F
        cmd = bytearray([0xAA, 0x0C, 0x07, channel, lsb, msb])
        bytes_written = ser.write(cmd)
        ser.flush() # Ensure data is sent
        if bytes_written == len(cmd):
            # print(f"[DEBUG] Set speed for channel {channel} to {speed}")
            return True
        else:
            print(f"[ERROR] set_speed: Wrote {bytes_written}/{len(cmd)} bytes for channel {channel}")
            return False
    except serial.SerialTimeoutException:
        print(f"[ERROR] set_speed: Write timeout for channel {channel}")
        return False
    except serial.SerialException as e:
        print(f"[ERROR] set_speed: SerialException for channel {channel}: {e}")
        return False # Indicate failure

def set_target(ser, channel, target):
    """Set servo target position. Requires an active serial connection object."""
    if not ser or not ser.is_open:
        print("[ERROR] set_target: Serial connection is not open.")
        return False # Indicate failure

    try:
        target = int(target) # Ensure target is an integer
        if not (0 <= target <= 10000): # Basic range check, adjust if needed
             print(f"[WARN] set_target: Target {target} for channel {channel} might be out of typical range (0-10000).")

        # Using Pololu Protocol: 0xAA, device_id, 0x04, channel, target_lsb, target_msb
        # Assuming default device ID 12 (0x0C)
        lsb = target & 0x7F
        msb = (target >> 7) & 0x7F
        cmd = bytearray([0xAA, 0x0C, 0x04, channel, lsb, msb])

        # Optional: Clear buffers before write if experiencing issues
        # ser.reset_output_buffer()

        bytes_written = ser.write(cmd)
        ser.flush() # Ensure data is sent immediately
        time.sleep(0.01) # Add small delay after sending command

        if bytes_written == len(cmd):
            # Keep the debug print to confirm sending
            print(f"[DEBUG] set_target: Sent Pololu command for channel {channel} to {target}")
            return True
        else:
            # This indicates a potential problem with the serial write
            print(f"[ERROR] set_target: Wrote {bytes_written}/{len(cmd)} bytes for channel {channel}")
            return False # Indicate failure

    except serial.SerialTimeoutException:
        # This exception occurs if write_timeout is set and reached
        print(f"[ERROR] set_target: Write timeout for channel {channel}")
        return False # Indicate failure
    except serial.SerialException as e:
        # This catches other serial communication errors (e.g., port disconnected)
        print(f"[ERROR] set_target: SerialException for channel {channel}: {e}")
        # Consider raising the exception instead of returning False
        # raise e
        return False # Indicate failure
    except ValueError as e:
        # Catch potential errors if target conversion fails
        print(f"[ERROR] set_target: Invalid target value '{target}' for channel {channel}: {e}")
        return False
    except Exception as e:
        # Catch any other unexpected errors
        print(f"[ERROR] set_target: Unexpected error for channel {channel}: {e}")
        return False


# Function to set multiple positions together - Accepts 'ser'
def set_positions(ser, positions, delay=0.5):
    """Sets multiple servo positions using the provided serial connection."""
    all_success = True
    for channel, position in positions.items():
        if not set_target(ser, channel, position):
            print(f"[WARN] Failed to set target for channel {channel}")
            all_success = False # Track if any command failed
    if delay > 0:
        time.sleep(delay)
    return all_success # Return overall success status

def release_servos(ser):
    """Sets the target of all servos to 0 to release them."""
    print("Releasing all servos...")
    all_success = True
    for channel in range(17): # Assuming 17 servos (0-16)
        if not set_target(ser, channel, 0):
            print(f"[WARN] Failed to release servo {channel}")
            all_success = False
    if all_success:
        print("All servos released (commands sent).")
    else:
        print("Attempted to release all servos, but some commands failed.")
    return all_success

# --- Define Movement Functions ---
# All movement functions now accept 'ser' as the first argument

def raise_hands(ser):
    """Moves the robot's arms to a raised position."""
    print("Raising hands...")
    raised_position = {
        11: 4000, # Right Shoulder (RS)
        10: 8000, # Left Shoulder (LS)
        13: 6000, # Right Elbow (RE)
        12: 6000  # Left Elbow (LE)
    }
    success = set_positions(ser, raised_position, delay=1.0) # Reduced delay slightly
    if success:
        print("Hands raised.")
    else:
        print("Failed to complete raise hands sequence.")
    return success

def make_cross(ser):
    """Moves the robot's arms to form a cross shape."""
    print("Making cross...")
    cross_position = {
        11: 6000, # Right Shoulder (RS)
        10: 6000, # Left Shoulder (LS)
        13: 6000, # Right Elbow (RE)
        12: 6000  # Left Elbow (LE)
    }
    success = set_positions(ser, cross_position, delay=1.0)
    if success:
        print("Cross made.")
    else:
        print("Failed to complete make cross sequence.")
    return success

def make_v(ser):
    """Moves the robot's arms to form a V shape."""
    print("Making V...")
    v_position = {
        11: 5000, # Right Shoulder (RS)
        10: 7000, # Left Shoulder (LS)
        13: 6000, # Right Elbow (RE)
        12: 6000  # Left Elbow (LE)
    }
    success = set_positions(ser, v_position, delay=1.0)
    if success:
        print("V made.")
    else:
        print("Failed to complete make V sequence.")
    return success

def walk_step(ser, leg, high_knee=False):
    """Performs a single walking step with the specified leg."""
    # print(f"Performing walk step with {leg} leg (high knee: {high_knee})...") # Verbose
    step_delay = 0.3 # Faster steps

    if leg == 'right':
        lift_pos = {3: 5500, 5: 4000, 7: 4500} if high_knee else {3: 6000, 5: 5000, 7: 5000}
        forward_pos = {3: 6300, 5: 5400, 7: 6000}
        down_pos = {3: 6300, 5: 5400, 7: 5600} # Back to standing hip
    elif leg == 'left':
        lift_pos = {2: 5800, 4: 5000, 6: 6400} if high_knee else {2: 5300, 4: 5400, 6: 5900}
        forward_pos = {2: 5000, 4: 6000, 6: 5300}
        down_pos = {2: 5000, 4: 6000, 6: 5300} # Back to standing hip
    else:
        print("[ERROR] Invalid leg specified for walk_step. Use 'left' or 'right'.")
        return False

    # Execute sequence
    if not set_positions(ser, lift_pos, delay=step_delay): return False
    if not set_positions(ser, forward_pos, delay=step_delay): return False
    if not set_positions(ser, down_pos, delay=step_delay): return False

    # print(f"Walk step with {leg} leg complete.") # Verbose
    return True

def walk(ser, steps):
    """Performs a walking sequence for a specified number of steps."""
    print(f"Starting walking sequence for {steps} steps...")
    all_success = True
    for i in range(steps):
        print(f"Step {i+1}/{steps}...")
        if not walk_step(ser, 'right'):
            all_success = False
            break
        if not walk_step(ser, 'left'):
            all_success = False
            break
    if all_success:
        print("Walking sequence finished.")
    else:
        print("Walking sequence interrupted due to command failure.")
    # Return to standing position after walking
    set_positions(ser, standing_position, delay=1.0)
    return all_success

def walk_high_knees(ser, steps):
    """Performs a high-knee walking sequence."""
    print(f"Starting high-knee walking sequence for {steps} steps...")
    all_success = True
    for i in range(steps):
        print(f"High-Knee Step {i+1}/{steps}...")
        if not walk_step(ser, 'right', high_knee=True):
            all_success = False
            break
        if not walk_step(ser, 'left', high_knee=True):
            all_success = False
            break
    if all_success:
        print("High-knee walking sequence finished.")
    else:
        print("High-knee walking sequence interrupted due to command failure.")
    # Return to standing position after walking
    set_positions(ser, standing_position, delay=1.0)
    return all_success

def bend_forward(ser):
    """Moves the robot to bend forward."""
    print("Bending forward...")
    bend_pose = { 6: 6500, 7: 4500, 4: 5600, 5: 6200 }
    success = set_positions(ser, bend_pose, delay=2.0)
    if success:
        print("Bend forward complete.")
    else:
        print("Failed to complete bend forward.")
    return success

def bend_backwards(ser):
    """Moves the robot to bend backwards."""
    print("Bending backwards...")
    bend_pose = { 6: 4500, 7: 6500, 4: 5200, 5: 5800 }
    success = set_positions(ser, bend_pose, delay=2.0)
    if success:
        print("Bend backwards complete.")
    else:
        print("Failed to complete bend backwards.")
    return success

def perform_boxing_move(ser, move_type='jab'):
    """Performs a basic boxing move."""
    print(f"Performing {move_type} boxing move...")
    success = True
    if move_type == 'jab':
        jab_pose = { 11: 5000, 13: 5000 } # Right jab
        neutral_arm_pose = { 11: 6000, 13: 6000 }
        if not set_positions(ser, jab_pose, delay=0.3): success = False
        if not set_positions(ser, neutral_arm_pose, delay=0.3): success = False
    else:
        print(f"Unknown boxing move type: {move_type}")
        success = False

    if success:
        print(f"{move_type} boxing move complete.")
    else:
        print(f"Failed to complete {move_type} boxing move.")
    return success

def do_split(ser):
    """Attempts to move the robot into a split position."""
    print("Attempting split...")
    split_pose = {
        9: 4000, 8: 7000, 7: 5600, 6: 5300, 5: 6000,
        4: 6000, 3: 6000, 2: 5000
    }
    success = set_positions(ser, split_pose, delay=3.0)
    if success:
        print("Split attempt complete.")
    else:
        print("Failed to complete split attempt.")
    return success

def sit_down(ser):
    """Moves the robot into a sitting position."""
    print("Attempting to sit down...")
    sit_pose = {
        5: 4000, 4: 8000, 7: 7000, 6: 4000, 10: 6000,
        3: 6000, 2: 5000
    }
    success = set_positions(ser, sit_pose, delay=3.0)
    if success:
        print("Sit down attempt complete.")
    else:
        print("Failed to complete sit down attempt.")
    return success

def get_up(ser):
    """Attempts to move the robot from sitting to standing."""
    print("Attempting to get up...")
    # Assumes starting near sit_pose
    lean_forward_pose = {
        5: 4000, 4: 8000, 7: 6500, 6: 4500, 10: 5500,
        3: 6000, 2: 5000
    }
    success = True
    if not set_positions(ser, lean_forward_pose, delay=1.5): success = False
    print("Extending legs...")
    if not set_positions(ser, standing_position, delay=3.0): success = False

    if success:
        print("Get up attempt complete.")
    else:
        print("Failed to complete get up attempt.")
    return success


# Standing position definition (remains the same)
standing_position = {
    0: 6000,  # Left Ankle Twist (LAT)
    1: 6400,  # Right Ankle Twist (RAT)
    2: 6300,  # Left Ankle (LA)
    3: 5000,  # Right Ankle (RA)
    4: 5400,  # Left Knee (LK)
    5: 6000,  # Right Knee (RK)
    6: 5600,  # Left Hip-Thigh (LHT)
    7: 5300,  # Right Hip-Thigh (RHT)
    8: 5600,  # Left Hip (LH)
    9: 5600,  # Right Hip (RH)
    10: 6000,  # Left Shoulder (LS)
    11: 6000,  # Right Shoulder (RS)
    12: 6000,  # Left Elbow (LE)
    13: 6000,  # Right Elbow (RE)
    14: 6000,  # Left Palm (LP)
    15: 5000,  # Right Palm (RP)
    16: 5200   # Head
}

channel_to_joint_map = {
    0: "Left Ankle Twist (LAT)",
    1: "Right Ankle Twist (RAT)",
    2: "Left Ankle (LA)",
    3: "Right Ankle (RA)",
    4: "Left Knee (LK)",
    5: "Right Knee (RK)",
    6: "Left Hip-Thigh (LHT)",
    7: "Right Hip-Thigh (RHT)",
    8: "Left Hip (LH)",
    9: "Right Hip (RH)",
    10: "Left Shoulder (LS)",
    11: "Right Shoulder (RS)",
    12: "Left Elbow (LE)",
    13: "Right Elbow (RE)",
    14: "Left Palm (LP)",
    15: "Right Palm (RP)",
    16: "Head"
}

# --- Removed execution code from module level ---
# Initialization, setting speeds, standing position, and cleanup
# should be handled by the main application script (robot_control_app.py)
