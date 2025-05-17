# standing_pos.py
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
#   git config --global user.name "Your Name"
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
    """Moves the robot's arms to a raised position (straight up)."""
    print("Raising hands straight up...")
    raised_position = {
        11: 6000, # Right Shoulder (RS) - Straight up
        10: 6000, # Left Shoulder (LS) - Straight up
        13: 6000, # Right Elbow (RE) - Straighter
        12: 6000  # Left Elbow (LE) - Straighter
    }
    # Also ensure palms are somewhat open/neutral
    raised_position.update({14: 6000, 15: 5000}) # LP, RP
    success = set_positions(ser, raised_position, delay=1.5)
    if success:
        print("Hands raised straight up.")
    else:
        print("Failed to complete raise hands straight up sequence.")
    return success

def make_cross(ser):
    """Moves the robot's arms to form a cross shape over the chest."""
    print("Making cross over chest...")
    cross_position = {
        11: 7500, # Right Shoulder (RS) - Across body
        10: 4500, # Left Shoulder (LS) - Across body
        13: 7000, # Right Elbow (RE) - Bent
        12: 5000, # Left Elbow (LE) - Bent
        14: 6000, # Left Palm
        15: 5000  # Right Palm
    }
    success = set_positions(ser, cross_position, delay=1.5)
    if success:
        print("Cross over chest made.")
    else:
        print("Failed to complete make cross over chest sequence.")
    return success

def make_v(ser):
    """Moves the robot's arms to form a V shape."""
    print("Making V...")
    v_position = {
        11: 5000, # Right Shoulder (RS)
        10: 7000, # Left Shoulder (LS)
        13: 5000, # Right Elbow (RE) - Slightly straighter
        12: 7000, # Left Elbow (LE) - Slightly straighter
        14: 6000, # Left Palm
        15: 5000  # Right Palm
    }
    success = set_positions(ser, v_position, delay=1.0)
    if success:
        print("V made.")
    else:
        print("Failed to complete make V sequence.")
    return success

def adjust_balance(ser, stepping_leg):
    """Adjusts the torso and supporting leg to maintain balance while stepping."""
    print(f"Adjusting balance for {stepping_leg} leg step...")
    balance_pose = {}
    
    if stepping_leg == 'right':
        # Lean slightly to the left to balance on the left leg
        balance_pose = {
            8: 6200,  # Left Hip (LH) - slight lean to the left
            9: 5000,  # Right Hip (RH) - slight lean to the left
            6: 5400,  # Left Hip-Thigh (LHT) - slight forward adjust
            7: 5700,  # Right Hip-Thigh (RHT) - slight forward adjust
            # Supporting leg (left) adjustment
            2: 6100,  # Left Ankle (LA) - slight tilt to stabilize
            4: 5600,  # Left Knee (LK) - slight bend for stability
            # Arms for counterbalance
            10: 6500,  # Left Shoulder (LS) - slight outward for balance
            11: 5500   # Right Shoulder (RS) - slight outward for balance
        }
    elif stepping_leg == 'left':
        # Lean slightly to the right to balance on the right leg
        balance_pose = {
            8: 5600,  # Left Hip (LH) - slight lean to the right
            9: 5600,  # Right Hip (RH) - slight lean to the right
            6: 5800,  # Left Hip-Thigh (LHT) - slight forward adjust
            7: 5300,  # Right Hip-Thigh (RHT) - slight forward adjust
            # Supporting leg (right) adjustment
            3: 4850,  # Right Ankle (RA) - slight tilt to stabilize
            5: 5800,  # Right Knee (RK) - slight bend for stability
            # Arms for counterbalance
            10: 5500,  # Left Shoulder (LS) - slight outward for balance
            11: 6500   # Right Shoulder (RS) - slight outward for balance
        }
    
    success = set_positions(ser, balance_pose, delay=0.3)
    if not success:
        print(f"Failed to adjust balance for {stepping_leg} leg.")
    return success

def walk_step(ser, leg, high_knee=False):
    """Performs a single walking step with the specified leg, with balance adjustments."""
    print(f"Performing walk step with {leg} leg (high knee: {high_knee})...")
    step_delay = 0.6  # Slower delay for better stability

    # Step 1: Adjust balance before lifting the leg
    if not adjust_balance(ser, leg):
        return False

    # Step 2: Define positions for lifting, moving forward, and setting down
    if leg == 'right':
        # Lift: Raise the right leg
        lift_pos = {3: 5250, 5: 4500, 7: 4800} if high_knee else {3: 5450, 5: 5200, 7: 5100}
        # Forward: Swing the leg forward
        forward_pos = {3: 6050, 5: 5000, 7: 5700}
        # Down: Place the leg down
        down_pos = {3: standing_position[3], 5: standing_position[5], 7: standing_position[7]}
    elif leg == 'left':
        # Lift: Raise the left leg
        lift_pos = {2: 6650, 4: 4600, 6: 6000} if high_knee else {2: 6450, 4: 5100, 6: 5800}
        # Forward: Swing the leg forward
        forward_pos = {2: 5650, 4: 5400, 6: 5200}
        # Down: Place the leg down
        down_pos = {2: standing_position[2], 4: standing_position[4], 6: standing_position[6]}
    else:
        print("[ERROR] Invalid leg specified for walk_step. Use 'left' or 'right'.")
        return False

    # Execute the walking sequence
    if not set_positions(ser, lift_pos, delay=step_delay):
        return False
    if not set_positions(ser, forward_pos, delay=step_delay):
        return False
    if not set_positions(ser, down_pos, delay=step_delay):
        return False

    print(f"Walk step with {leg} leg complete.")
    return True

def walk(ser, steps):
    """Performs a walking sequence for a specified number of steps."""
    print(f"Starting walking sequence for {steps} steps...")
    all_success = True
    for i in range(steps):
        print(f"Step {i+1}/{steps}...")
        # Step with the right leg
        if not walk_step(ser, 'right'):
            all_success = False
            break
        # Step with the left leg
        if not walk_step(ser, 'left'):
            all_success = False
            break
    
    if all_success:
        print("Walking sequence finished.")
    else:
        print("Walking sequence interrupted due to command failure.")
    
    # Return to standing position with a smooth transition
    print("Returning to standing position...")
    set_positions(ser, standing_position, delay=1.5)
    return all_success

def walk_high_knees(ser, steps):
    """Performs a high-knee walking sequence."""
    print(f"Starting high-knee walking sequence for {steps} steps...")
    all_success = True
    for i in range(steps):
        print(f"High-Knee Step {i+1}/{steps}...")
        # High-knee step with the right leg
        if not walk_step(ser, 'right', high_knee=True):
            all_success = False
            break
        # High-knee step with the left leg
        if not walk_step(ser, 'left', high_knee=True):
            all_success = False
            break
    
    if all_success:
        print("High-knee walking sequence finished.")
    else:
        print("High-knee walking sequence interrupted due to command failure.")
    
    # Return to standing position with a smooth transition
    print("Returning to standing position...")
    set_positions(ser, standing_position, delay=1.5)
    return all_success


def bend_forward(ser):
    """Moves the robot to bend forward."""
    print("Bending forward...")
    bend_pose = { 6: 6500, 7: 4500, 4: 5600, 5: 6200 } # LHT, RHT, LK, RK
    # Keep arms somewhat neutral or slightly forward
    bend_pose.update({10: 5500, 11: 6500, 12: 6000, 13: 6000}) # LS, RS, LE, RE
    success = set_positions(ser, bend_pose, delay=2.0)
    if success:
        print("Bend forward complete.")
    else:
        print("Failed to complete bend forward.")
    return success

def bend_backwards(ser):
    """Moves the robot to bend backwards."""
    print("Bending backwards...")
    bend_pose = { 6: 4500, 7: 6500, 4: 5200, 5: 5800 } # LHT, RHT, LK, RK
    # Keep arms somewhat neutral or slightly back
    bend_pose.update({10: 6500, 11: 5500, 12: 6000, 13: 6000}) # LS, RS, LE, RE
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
    neutral_arm_pose_right = { 11: 6000, 13: 6000 } # RS, RE
    neutral_arm_pose_left = { 10: 6000, 12: 6000 } # LS, LE

    if move_type == 'jab': # Right Jab
        jab_pose = { 11: 5000, 13: 5000 } # RS forward, RE extend
        if not set_positions(ser, jab_pose, delay=0.3): success = False
        if not set_positions(ser, neutral_arm_pose_right, delay=0.4): success = False
    elif move_type == 'left_jab':
        jab_pose = { 10: 7000, 12: 7000 } # LS forward, LE extend
        if not set_positions(ser, jab_pose, delay=0.3): success = False
        if not set_positions(ser, neutral_arm_pose_left, delay=0.4): success = False
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
        9: 4000, 8: 7000, # RH, LH (legs out)
        7: 5600, 6: 5300, # RHT, LHT (thighs straight)
        5: 6000, 4: 6000, # RK, LK (knees straight)
        3: 4650, 2: 6300, # RA, LA (ankles flat)
        # Arms for balance
        10: 7000, 11: 5000, # LS, RS (out to sides)
        12: 6000, 13: 6000  # LE, RE (elbows neutral)
    }
    success = set_positions(ser, split_pose, delay=3.5) # Increased delay
    if success:
        print("Split attempt complete.")
    else:
        print("Failed to complete split attempt.")
    return success

def sit_down(ser):
    """Moves the robot into a sitting position."""
    print("Attempting to sit down...")
    # Phase 1: Bend knees and hips, lean slightly forward with arms for balance
    pre_sit_pose = standing_position.copy()
    pre_sit_pose.update({
        4: 4000, 5: 8000,  # LK, RK (bend knees significantly)
        6: 7000, 7: 4000,  # LHT, RHT (bend hips significantly)
        8: 6200, 9: 5000,  # LH, RH (slight hip adjustment)
        # Arms forward for balance
        10: 5000, 11: 7000, # LS, RS
        12: 5000, 13: 7000  # LE, RE
    })
    if not set_positions(ser, pre_sit_pose, delay=2.0): return False

    # Phase 2: Final sit
    sit_pose = {
        # Legs
        2: 7500, 3: 3500,  # LA, RA (ankles for sitting)
        4: 3000, 5: 9000,  # LK, RK (knees fully bent)
        6: 8000, 7: 3000,  # LHT, RHT (hips fully bent)
        8: 6000, 9: 6000,  # LH, RH (hips neutral side-to-side)
        # Torso/Arms neutral or slightly relaxed
        10: 6000, 11: 6000, # LS, RS
        12: 6000, 13: 6000, # LE, RE
        14: 6000, 15: 5000  # Palms
    }
    success = set_positions(ser, sit_pose, delay=3.5) # Increased delay
    if success:
        print("Sit down attempt complete.")
    else:
        print("Failed to complete sit down attempt.")
    return success

def get_up(ser):
    """Attempts to move the robot from sitting to standing."""
    print("Attempting to get up...")
    # Assume starting near sit_pose or a crouched position
    # Phase 1: Lean forward, shift weight, prepare to push
    lean_forward_pose = {
        # Legs preparing to push
        2: 6300, 3: 4650,  # LA, RA (ankles to standing_ish)
        4: 4000, 5: 8000,  # LK, RK (knees still bent but ready)
        6: 7000, 7: 4000,  # LHT, RHT (hips forward)
        8: 5900, 9: 5300,  # LH, RH (hips to standing_ish)
        # Arms forward and down for push/balance
        10: 4500, 11: 7500, # LS, RS
        12: 4500, 13: 7500, # LE, RE
    }
    if not set_positions(ser, lean_forward_pose, delay=2.0): return False # Increased delay

    print("Pushing up and extending legs...")
    # Phase 2: Extend legs to standing position
    # Use standing_position as the target
    success = set_positions(ser, standing_position, delay=3.5) # Increased delay

    if success:
        print("Get up attempt complete.")
    else:
        print("Failed to complete get up attempt.")
    return success

# --- NEW ACTIONS ---
def wave_hello_right(ser):
    """Robot waves with its right hand."""
    print("Waving hello with right hand...")
    success = True
    # Initial position: right arm up
    initial_arm_pos = {11: 4500, 13: 5000, 15: 7000} # RS, RE, RP (palm open)
    if not set_positions(ser, initial_arm_pos, delay=0.7): return False

    # Wave 1
    wave_1_pos = {13: 4000} # RE (elbow bend)
    if not set_positions(ser, wave_1_pos, delay=0.5): success = False
    wave_2_pos = {13: 6000} # RE (elbow straighten)
    if not set_positions(ser, wave_2_pos, delay=0.5): success = False

    # Wave 2
    if not set_positions(ser, wave_1_pos, delay=0.5): success = False
    if not set_positions(ser, wave_2_pos, delay=0.5): success = False

    # Return arm to neutral
    neutral_arm_pos = {11: standing_position[11], 13: standing_position[13], 15: standing_position[15]}
    if not set_positions(ser, neutral_arm_pos, delay=0.7): success = False

    if success:
        print("Wave hello complete.")
    else:
        print("Failed to complete wave hello.")
    return success

def head_nod_yes(ser):
    """Robot nods its head for 'yes'."""
    print("Nodding head yes...")
    success = True
    head_channel = 16
    original_head_pos = standing_position.get(head_channel, 5200) # Default if not in standing_pos

    nod_down_pos = {head_channel: original_head_pos + 800} # Tilt head down
    nod_up_pos = {head_channel: original_head_pos - 500} # Tilt head slightly up
    neutral_head_pos = {head_channel: original_head_pos}

    if not set_positions(ser, nod_down_pos, delay=0.5): return False
    if not set_positions(ser, nod_up_pos, delay=0.5): return False
    if not set_positions(ser, nod_down_pos, delay=0.5): return False # Second nod
    if not set_positions(ser, neutral_head_pos, delay=0.5): success = False # Return to neutral

    if success:
        print("Head nod complete.")
    else:
        print("Failed to complete head nod.")
    return success

def look_up_down(ser):
    """Robot looks up and then down."""
    print("Looking up and down...")
    success = True
    head_channel = 16
    original_head_pos = standing_position.get(head_channel, 5200)

    look_up_pos = {head_channel: original_head_pos - 1000} # Look further up
    look_down_pos = {head_channel: original_head_pos + 1000} # Look further down
    neutral_head_pos = {head_channel: original_head_pos}

    if not set_positions(ser, look_up_pos, delay=0.8): return False
    if not set_positions(ser, neutral_head_pos, delay=0.6): return False
    if not set_positions(ser, look_down_pos, delay=0.8): return False
    if not set_positions(ser, neutral_head_pos, delay=0.6): success = False

    if success:
        print("Look up/down complete.")
    else:
        print("Failed to complete look up/down.")
    return success

def karate_chop_right(ser):
    """Robot performs a right hand karate chop."""
    print("Performing right karate chop...")
    success = True
    # Wind up
    wind_up_pos = {
        11: 4500, # RS - raise shoulder
        13: 4000, # RE - bend elbow back
        15: 6000  # RP - hand edge
    }
    if not set_positions(ser, wind_up_pos, delay=0.7): return False

    # Chop
    chop_pos = {
        11: 6500, # RS - swing shoulder forward/down
        13: 7000  # RE - extend elbow
    }
    if not set_positions(ser, chop_pos, delay=0.3): success = False

    # Return to neutral
    neutral_arm_pos = {
        11: standing_position[11],
        13: standing_position[13],
        15: standing_position[15]
    }
    if not set_positions(ser, neutral_arm_pos, delay=0.7): success = False

    if success:
        print("Karate chop complete.")
    else:
        print("Failed to complete karate chop.")
    return success

def bow_action(ser):
    """Robot performs a bow."""
    print("Performing bow...")
    # Current standing for arms
    arm_neutral_l = {10: 6000, 12: 6000}
    arm_neutral_r = {11: 6000, 13: 6000}

    # Arms slightly to sides/back for a formal bow
    bow_arms = {
        10: 6500, 12: 6000, # LS, LE
        11: 5500, 13: 6000  # RS, RE
    }
    if not set_positions(ser, bow_arms, delay=0.5): return False

    # Bend at hips and neck
    bow_body_pose = {
        6: 6800, 7: 4200,  # LHT, RHT (bend hips forward)
        16: 6000           # Head (tilt down)
    }
    # Combine with arm positions
    full_bow_pose = {**bow_arms, **bow_body_pose}
    if not set_positions(ser, full_bow_pose, delay=1.5): return False

    # Hold bow
    time.sleep(1.0)

    # Return to standing
    # First, straighten body while keeping arms
    straighten_body_pose = {
        6: standing_position[6], 7: standing_position[7],
        16: standing_position[16]
    }
    full_straighten_pose = {**bow_arms, **straighten_body_pose}
    if not set_positions(ser, full_straighten_pose, delay=1.5): return False
    # Then return arms to neutral standing
    if not set_positions(ser, standing_position, delay=1.0): return False # Full reset to standing

    print("Bow complete.")
    return True


def hug_prep(ser):
    """Robot moves arms to a 'ready to hug' position."""
    print("Preparing for a hug...")
    hug_pose = {
        10: 4500, # Left Shoulder (LS) - forward and slightly out
        11: 7500, # Right Shoulder (RS) - forward and slightly out
        12: 4500, # Left Elbow (LE) - bent, open
        13: 7500, # Right Elbow (RE) - bent, open
        14: 7000, # Left Palm (LP) - open
        15: 4000  # Right Palm (RP) - open
    }
    success = set_positions(ser, hug_pose, delay=1.5)
    if success:
        print("Ready for a hug!")
    else:
        print("Failed to prepare for hug.")
    return success

# Standing position definition (remains the same)
standing_position = {
    0: 6350,  # Left Ankle Twist (LAT)
    1: 6150,  # Right Ankle Twist (RAT)
    2: 6250,  # Left Ankle (LA)
    3: 4600,  # Right Ankle (RA)
    4: 5400,  # Left Knee (LK)
    5: 6000,  # Right Knee (RK)
    6: 5800,  # Left Hip-Thigh (LHT)
    7: 5400,  # Right Hip-Thigh (RHT)
    8: 5900,  # Left Hip (LH)
    9: 5100,  # Right Hip (RH)
    10: 5800,  # Left Shoulder (LS)
    11: 6100,  # Right Shoulder (RS)
    12: 8000,  # Left Elbow (LE)
    13: 3800,  # Right Elbow (RE)
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
