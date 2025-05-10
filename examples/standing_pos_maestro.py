# standing_post_maestro.py
import time
import serial  # Required for serial.SerialException, serial.SerialTimeoutException, and serial.Serial type check
from examples.maestro import Controller  # Assuming maestro.py is in examples/

# Default port, can be overridden in init_maestro_controller
PORT = '/dev/cu.usbmodem004763651'
DEVICE_ID = 0x0c  # Default Pololu device ID, matching implicit one in standing_pos.py

# Serial settings from standing_pos.py that we'll try to apply
SERIAL_TIMEOUT = 1.0  # Seconds for read timeout
SERIAL_WRITE_TIMEOUT = 1.0  # Seconds for write timeout

def init_maestro_controller(port=PORT, device_id=DEVICE_ID, retries=3, delay=2.0):
    """
    Initialize connection to the Maestro controller using the Controller class.
    Attempts to apply similar timeout settings as standing_pos.py's init_serial.
    Returns the Controller object on success, None on failure.
    """
    maestro_controller = None
    for attempt in range(retries):
        try:
            print(f"Attempting connection to Maestro on {port} (device ID {device_id}, try {attempt + 1}/{retries})...")
            maestro_controller = Controller(ttyStr=port, device=device_id)

            if hasattr(maestro_controller, 'usb') and isinstance(maestro_controller.usb, serial.Serial):
                maestro_controller.usb.timeout = SERIAL_TIMEOUT
                maestro_controller.usb.write_timeout = SERIAL_WRITE_TIMEOUT
                # Note: standing_pos.py also sets parity, stopbits, bytesize, inter_byte_timeout.
                # These are typically pyserial defaults (N, 1, 8) and inter_byte_timeout is less common.
                # Modifying them post-hoc on an open serial.Serial object is not standard.
                # We prioritize read/write timeouts for functional equivalence here.

                if maestro_controller.usb.is_open:
                    print(f"Successfully connected to Maestro on {maestro_controller.usb.name}")
                    # Clear buffers, similar to standing_pos.py's init_serial
                    maestro_controller.usb.reset_input_buffer()
                    maestro_controller.usb.reset_output_buffer()
                    return maestro_controller
                else:
                    print(f"Connection attempt {attempt + 1} failed: Port not open after Controller init.")
                    if maestro_controller:
                        try:
                            maestro_controller.close()
                        except Exception:
                            pass
                    maestro_controller = None
            else:
                print(f"Connection attempt {attempt + 1} failed: Controller.usb attribute not found or not a Serial object.")
                if maestro_controller:
                    try:
                        maestro_controller.close()
                    except Exception:
                        pass
                maestro_controller = None

        except serial.SerialException as e:
            print(f"Maestro connection error (attempt {attempt + 1}/{retries}): {e}")
            if maestro_controller:
                try:
                    maestro_controller.close()
                except Exception:
                    pass
            maestro_controller = None
        except Exception as e:  # Catch any other exceptions
            print(f"Unexpected error during Maestro Controller initialization (attempt {attempt + 1}/{retries}): {e}")
            if maestro_controller:
                try:
                    maestro_controller.close()
                except Exception:
                    pass
            maestro_controller = None

        if attempt < retries - 1:
            print(f"Waiting {delay} seconds before retrying...")
            time.sleep(delay)

    print(f"Failed to connect to Maestro on {port} after {retries} attempts.")
    return None

def set_speed(maestro_controller, channel, speed):
    """Set servo speed using Maestro Controller. Mimics standing_pos.py's interface."""
    if not maestro_controller:
        print("[ERROR] set_speed: Maestro controller object is None.")
        return False
    if not hasattr(maestro_controller, 'usb') or not maestro_controller.usb.is_open:
        print("[ERROR] set_speed: Serial connection in Maestro controller is not open or available.")
        return False

    try:
        speed = int(speed)
        maestro_controller.setSpeed(channel, speed)
        # print(f"[DEBUG] Set speed for channel {channel} to {speed} (command sent via Maestro Controller)")
        return True
    except serial.SerialTimeoutException:
        print(f"[ERROR] set_speed: Write timeout for channel {channel} via Maestro Controller")
        return False
    except serial.SerialException as e:
        print(f"[ERROR] set_speed: SerialException for channel {channel} via Maestro Controller: {e}")
        return False
    except ValueError as e:
        print(f"[ERROR] set_speed: Invalid speed value '{speed}' for channel {channel}: {e}")
        return False
    except Exception as e:
        print(f"[ERROR] set_speed: Unexpected error for channel {channel} via Maestro Controller: {e}")
        return False

def set_target(maestro_controller, channel, target):
    """Set servo target position using Maestro Controller. Mimics standing_pos.py's interface."""
    if not maestro_controller:
        print("[ERROR] set_target: Maestro controller object is None.")
        return False
    if not hasattr(maestro_controller, 'usb') or not maestro_controller.usb.is_open:
        print("[ERROR] set_target: Serial connection in Maestro controller is not open or available.")
        return False

    try:
        target = int(target)
        # standing_pos.py had a warning for target range. Maestro's setTarget has Min/Max logic
        # but it's only active if setRange is called. We don't call setRange here.
        if not (0 <= target <= 10000): # Basic range check from standing_pos.py
            print(f"[WARN] set_target: Target {target} for channel {channel} might be out of typical Maestro range (e.g., 3000-9000 for servos in quarter-us).")

        maestro_controller.setTarget(channel, target)
        print(f"[DEBUG] set_target: Sent command for channel {channel} to {target} via Maestro Controller")
        time.sleep(0.01)  # Delay from original standing_pos.py's set_target
        return True
    except serial.SerialTimeoutException:
        print(f"[ERROR] set_target: Write timeout for channel {channel} via Maestro Controller")
        return False
    except serial.SerialException as e:
        print(f"[ERROR] set_target: SerialException for channel {channel} via Maestro Controller: {e}")
        return False
    except ValueError as e:
        print(f"[ERROR] set_target: Invalid target value '{target}' for channel {channel}: {e}")
        return False
    except Exception as e:
        print(f"[ERROR] set_target: Unexpected error for channel {channel} via Maestro Controller: {e}")
        return False

def set_positions(maestro_controller, positions, delay=0.5):
    """Sets multiple servo positions using the provided Maestro controller."""
    all_success = True
    for channel, position in positions.items():
        if not set_target(maestro_controller, channel, position):
            print(f"[WARN] Failed to set target for channel {channel}")
            all_success = False
    if delay > 0:
        time.sleep(delay)
    return all_success

def release_servos(maestro_controller):
    """Sets the target of all servos to 0 to release them."""
    print("Releasing all servos...")
    all_success = True
    for channel in range(17):  # Assuming 17 servos (0-16) as in standing_pos.py
        if not set_target(maestro_controller, channel, 0):
            print(f"[WARN] Failed to release servo {channel}")
            all_success = False
    if all_success:
        print("All servos released (commands sent).")
    else:
        print("Attempted to release all servos, but some commands failed.")
    return all_success

# --- Define Movement Functions ---
# All movement functions now accept 'maestro_controller' as the first argument

standing_position = {
    0: 6000,  # Left Ankle Twist (LAT)
    1: 6400,  # Right Ankle Twist (RAT)
    2: 6250,  # Left Ankle (LA)
    3: 4600,  # Right Ankle (RA)
    4: 5400,  # Left Knee (LK)
    5: 6000,  # Right Knee (RK)
    6: 5600,  # Left Hip-Thigh (LHT)
    7: 5400,  # Right Hip-Thigh (RHT)
    8: 5900,  # Left Hip (LH)
    9: 5300,  # Right Hip (RH)
    10: 6000, # Left Shoulder (LS)
    11: 6000, # Right Shoulder (RS)
    12: 8000, # Left Elbow (LE)
    13: 4000, # Right Elbow (RE)
    14: 6000, # Left Palm (LP)
    15: 5000, # Right Palm (RP)
    16: 5200  # Head
}

def raise_hands(maestro_controller):
    """Moves the robot's arms to a raised position (straight up)."""
    print("Raising hands straight up...")
    raised_position = {
        11: 6000, # Right Shoulder (RS) - Straight up
        10: 6000, # Left Shoulder (LS) - Straight up
        13: 6000, # Right Elbow (RE) - Straighter
        12: 6000  # Left Elbow (LE) - Straighter
    }
    raised_position.update({14: 6000, 15: 5000}) # LP, RP
    success = set_positions(maestro_controller, raised_position, delay=1.5)
    if success:
        print("Hands raised straight up.")
    else:
        print("Failed to complete raise hands straight up sequence.")
    return success

def make_cross(maestro_controller):
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
    success = set_positions(maestro_controller, cross_position, delay=1.5)
    if success:
        print("Cross over chest made.")
    else:
        print("Failed to complete make cross over chest sequence.")
    return success

def make_v(maestro_controller):
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
    success = set_positions(maestro_controller, v_position, delay=1.0)
    if success:
        print("V made.")
    else:
        print("Failed to complete make V sequence.")
    return success

def adjust_balance(maestro_controller, stepping_leg):
    """Adjusts the torso and supporting leg to maintain balance while stepping."""
    print(f"Adjusting balance for {stepping_leg} leg step...")
    balance_pose = {}
    if stepping_leg == 'right':
        balance_pose = {
            8: 6200, 9: 5000, 6: 5400, 7: 5700,
            2: 6100, 4: 5600, 10: 6500, 11: 5500
        }
    elif stepping_leg == 'left':
        balance_pose = {
            8: 5600, 9: 5600, 6: 5800, 7: 5300,
            3: 4850, 5: 5800, 10: 5500, 11: 6500
        }
    success = set_positions(maestro_controller, balance_pose, delay=0.3)
    if not success:
        print(f"Failed to adjust balance for {stepping_leg} leg.")
    return success

def walk_step(maestro_controller, leg, high_knee=False):
    """Performs a single walking step with the specified leg, with balance adjustments."""
    print(f"Performing walk step with {leg} leg (high knee: {high_knee})...")
    step_delay = 0.6

    if not adjust_balance(maestro_controller, leg):
        return False

    if leg == 'right':
        lift_pos = {3: 5250, 5: 4500, 7: 4800} if high_knee else {3: 5450, 5: 5200, 7: 5100}
        forward_pos = {3: 6050, 5: 5000, 7: 5700}
        down_pos = {3: standing_position[3], 5: standing_position[5], 7: standing_position[7]}
    elif leg == 'left':
        lift_pos = {2: 6650, 4: 4600, 6: 6000} if high_knee else {2: 6450, 4: 5100, 6: 5800}
        forward_pos = {2: 5650, 4: 5400, 6: 5200}
        down_pos = {2: standing_position[2], 4: standing_position[4], 6: standing_position[6]}
    else:
        print("[ERROR] Invalid leg specified for walk_step. Use 'left' or 'right'.")
        return False

    if not set_positions(maestro_controller, lift_pos, delay=step_delay): return False
    if not set_positions(maestro_controller, forward_pos, delay=step_delay): return False
    if not set_positions(maestro_controller, down_pos, delay=step_delay): return False

    print(f"Walk step with {leg} leg complete.")
    return True

def walk(maestro_controller, steps):
    """Performs a walking sequence for a specified number of steps."""
    print(f"Starting walking sequence for {steps} steps...")
    all_success = True
    for i in range(steps):
        print(f"Step {i+1}/{steps}...")
        if not walk_step(maestro_controller, 'right'): all_success = False; break
        if not walk_step(maestro_controller, 'left'): all_success = False; break
    
    if all_success: print("Walking sequence finished.")
    else: print("Walking sequence interrupted due to command failure.")
    
    print("Returning to standing position...")
    set_positions(maestro_controller, standing_position, delay=1.5)
    return all_success

def walk_high_knees(maestro_controller, steps):
    """Performs a high-knee walking sequence."""
    print(f"Starting high-knee walking sequence for {steps} steps...")
    all_success = True
    for i in range(steps):
        print(f"High-Knee Step {i+1}/{steps}...")
        if not walk_step(maestro_controller, 'right', high_knee=True): all_success = False; break
        if not walk_step(maestro_controller, 'left', high_knee=True): all_success = False; break
        
    if all_success: print("High-knee walking sequence finished.")
    else: print("High-knee walking sequence interrupted due to command failure.")
        
    print("Returning to standing position...")
    set_positions(maestro_controller, standing_position, delay=1.5)
    return all_success

def bend_forward(maestro_controller):
    print("Bending forward...")
    bend_pose = {6: 6500, 7: 4500, 4: 5600, 5: 6200}
    bend_pose.update({10: 5500, 11: 6500, 12: 6000, 13: 6000})
    success = set_positions(maestro_controller, bend_pose, delay=2.0)
    if success: print("Bend forward complete.")
    else: print("Failed to complete bend forward.")
    return success

def bend_backwards(maestro_controller):
    print("Bending backwards...")
    bend_pose = {6: 4500, 7: 6500, 4: 5200, 5: 5800}
    bend_pose.update({10: 6500, 11: 5500, 12: 6000, 13: 6000})
    success = set_positions(maestro_controller, bend_pose, delay=2.0)
    if success: print("Bend backwards complete.")
    else: print("Failed to complete bend backwards.")
    return success

def perform_boxing_move(maestro_controller, move_type='jab'):
    print(f"Performing {move_type} boxing move...")
    success = True
    neutral_arm_pose_right = {11: 6000, 13: 6000}
    neutral_arm_pose_left = {10: 6000, 12: 6000}

    if move_type == 'jab':
        jab_pose = {11: 5000, 13: 5000}
        if not set_positions(maestro_controller, jab_pose, delay=0.3): success = False
        if success and not set_positions(maestro_controller, neutral_arm_pose_right, delay=0.4): success = False
    elif move_type == 'left_jab':
        jab_pose = {10: 7000, 12: 7000}
        if not set_positions(maestro_controller, jab_pose, delay=0.3): success = False
        if success and not set_positions(maestro_controller, neutral_arm_pose_left, delay=0.4): success = False
    else:
        print(f"Unknown boxing move type: {move_type}")
        success = False

    if success: print(f"{move_type} boxing move complete.")
    else: print(f"Failed to complete {move_type} boxing move.")
    return success

def do_split(maestro_controller):
    print("Attempting split...")
    split_pose = {
        9: 4000, 8: 7000, 7: 5600, 6: 5300, 5: 6000, 4: 6000,
        3: 4650, 2: 6300, 10: 7000, 11: 5000, 12: 6000, 13: 6000
    }
    success = set_positions(maestro_controller, split_pose, delay=3.5)
    if success: print("Split attempt complete.")
    else: print("Failed to complete split attempt.")
    return success

def sit_down(maestro_controller):
    print("Attempting to sit down...")
    pre_sit_pose = standing_position.copy()
    pre_sit_pose.update({
        4: 4000, 5: 8000, 6: 7000, 7: 4000, 8: 6200, 9: 5000,
        10: 5000, 11: 7000, 12: 5000, 13: 7000
    })
    if not set_positions(maestro_controller, pre_sit_pose, delay=2.0): return False

    sit_pose = {
        2: 7500, 3: 3500, 4: 3000, 5: 9000, 6: 8000, 7: 3000,
        8: 6000, 9: 6000, 10: 6000, 11: 6000, 12: 6000, 13: 6000,
        14: 6000, 15: 5000
    }
    success = set_positions(maestro_controller, sit_pose, delay=3.5)
    if success: print("Sit down attempt complete.")
    else: print("Failed to complete sit down attempt.")
    return success

def get_up(maestro_controller):
    print("Attempting to get up...")
    lean_forward_pose = {
        2: 6300, 3: 4650, 4: 4000, 5: 8000, 6: 7000, 7: 4000,
        8: 5900, 9: 5300, 10: 4500, 11: 7500, 12: 4500, 13: 7500
    }
    if not set_positions(maestro_controller, lean_forward_pose, delay=2.0): return False

    print("Pushing up and extending legs...")
    success = set_positions(maestro_controller, standing_position, delay=3.5)
    if success: print("Get up attempt complete.")
    else: print("Failed to complete get up attempt.")
    return success

def wave_hello_right(maestro_controller):
    print("Waving hello with right hand...")
    success = True
    initial_arm_pos = {11: 4500, 13: 5000, 15: 7000}
    if not set_positions(maestro_controller, initial_arm_pos, delay=0.7): return False

    wave_1_pos = {13: 4000}
    wave_2_pos = {13: 6000}
    for _ in range(2): # Wave twice
        if not set_positions(maestro_controller, wave_1_pos, delay=0.5): success = False; break
        if not set_positions(maestro_controller, wave_2_pos, delay=0.5): success = False; break
    if not success: return False # Exit if waving failed

    neutral_arm_pos = {
        11: standing_position[11], 
        13: standing_position[13], 
        15: standing_position[15]
    }
    if not set_positions(maestro_controller, neutral_arm_pos, delay=0.7): success = False

    if success: print("Wave hello complete.")
    else: print("Failed to complete wave hello.")
    return success

def head_nod_yes(maestro_controller):
    print("Nodding head yes...")
    success = True
    head_channel = 16
    original_head_pos = standing_position.get(head_channel, 5200)

    nod_down_pos = {head_channel: original_head_pos + 800}
    nod_up_pos = {head_channel: original_head_pos - 500} # standing_pos.py had this as original_head_pos - 500
    neutral_head_pos = {head_channel: original_head_pos}

    if not set_positions(maestro_controller, nod_down_pos, delay=0.5): return False
    if not set_positions(maestro_controller, nod_up_pos, delay=0.5): return False # Corrected to nod up
    if not set_positions(maestro_controller, nod_down_pos, delay=0.5): return False # Second nod
    if not set_positions(maestro_controller, neutral_head_pos, delay=0.5): success = False

    if success: print("Head nod complete.")
    else: print("Failed to complete head nod.")
    return success

def look_up_down(maestro_controller):
    print("Looking up and down...")
    success = True
    head_channel = 16
    original_head_pos = standing_position.get(head_channel, 5200)

    look_up_pos = {head_channel: original_head_pos - 1000}
    look_down_pos = {head_channel: original_head_pos + 1000}
    neutral_head_pos = {head_channel: original_head_pos}

    if not set_positions(maestro_controller, look_up_pos, delay=0.8): return False
    if not set_positions(maestro_controller, neutral_head_pos, delay=0.6): return False
    if not set_positions(maestro_controller, look_down_pos, delay=0.8): return False
    if not set_positions(maestro_controller, neutral_head_pos, delay=0.6): success = False

    if success: print("Look up/down complete.")
    else: print("Failed to complete look up/down.")
    return success

def karate_chop_right(maestro_controller):
    print("Performing right karate chop...")
    success = True
    wind_up_pos = {11: 4500, 13: 4000, 15: 6000}
    if not set_positions(maestro_controller, wind_up_pos, delay=0.7): return False

    chop_pos = {11: 6500, 13: 7000}
    if not set_positions(maestro_controller, chop_pos, delay=0.3): success = False
    if not success: return False # Exit if chop failed

    neutral_arm_pos = {
        11: standing_position[11], 
        13: standing_position[13], 
        15: standing_position[15]
    }
    if not set_positions(maestro_controller, neutral_arm_pos, delay=0.7): success = False

    if success: print("Karate chop complete.")
    else: print("Failed to complete karate chop.")
    return success

def bow_action(maestro_controller):
    print("Performing bow...")
    bow_arms = {10: 6500, 12: 6000, 11: 5500, 13: 6000}
    if not set_positions(maestro_controller, bow_arms, delay=0.5): return False

    bow_body_pose = {6: 6800, 7: 4200, 16: 6000}
    full_bow_pose = {**bow_arms, **bow_body_pose}
    if not set_positions(maestro_controller, full_bow_pose, delay=1.5): return False

    time.sleep(1.0) # Hold bow

    straighten_body_pose = {
        6: standing_position[6], 7: standing_position[7],
        16: standing_position[16]
    }
    full_straighten_pose = {**bow_arms, **straighten_body_pose}
    if not set_positions(maestro_controller, full_straighten_pose, delay=1.5): return False
    if not set_positions(maestro_controller, standing_position, delay=1.0): return False

    print("Bow complete.")
    return True

def hug_prep(maestro_controller):
    print("Preparing for a hug...")
    hug_pose = {
        10: 4500, 11: 7500, 12: 4500, 13: 7500, 14: 7000, 15: 4000
    }
    success = set_positions(maestro_controller, hug_pose, delay=1.5)
    if success: print("Ready for a hug!")
    else: print("Failed to prepare for hug.")
    return success

channel_to_joint_map = {
    0: "Left Ankle Twist (LAT)", 1: "Right Ankle Twist (RAT)",
    2: "Left Ankle (LA)", 3: "Right Ankle (RA)",
    4: "Left Knee (LK)", 5: "Right Knee (RK)",
    6: "Left Hip-Thigh (LHT)", 7: "Right Hip-Thigh (RHT)",
    8: "Left Hip (LH)", 9: "Right Hip (RH)",
    10: "Left Shoulder (LS)", 11: "Right Shoulder (RS)",
    12: "Left Elbow (LE)", 13: "Right Elbow (RE)",
    14: "Left Palm (LP)", 15: "Right Palm (RP)",
    16: "Head"
}

# No main execution block, this file is intended to be used as a module.
