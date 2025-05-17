## robot_control_app.py


import streamlit as st
import time
import serial
# import examples.standing_pos_maestro as standing_pos_maestro
import standing_pos as standing_pos_maestro
import subprocess  # Add this import
import os  # Also add this for path handling


# ls /dev/ttyACM*
# /dev/ttyACM0  /dev/ttyACM1

# Configure the page
st.set_page_config(
    page_title="Mini Humanoid Control Panel",
    page_icon="🤖",
    layout="wide",
    initial_sidebar_state="expanded"
)

# Custom CSS for styling
st.markdown("""
<style>
    .big-font {
        font-size:20px !important;
        font-weight: bold !important;
    }
    .title-font {
        font-size:32px !important;
        font-weight: bold !important;
        color: #FF4B4B;
        text-align: center;
        margin-bottom: 30px;
    }
    .stButton > button {
        width: 100%;
        height: 3em;
        margin: 0.5em 0;
        font-weight: bold;
        background-color: #4CAF50;
        color: white;
    }
    .stButton > button:disabled {
        background-color: #CCCCCC;
        color: #666666;
    }
    .emergency-button > button {
        background-color: #FF4B4B !important;
        color: white !important;
    }
    .servo-slider {
        margin-bottom: 1em;
    }
</style>
""", unsafe_allow_html=True)

# --- Configuration ---
PORT = '/dev/ttyACM1' # Adjust if needed
BAUDRATE = 115200
CONNECT_RETRIES = 3
CONNECT_DELAY = 2.0 # Seconds


# --- Initialize Session State ---
if 'ser' not in st.session_state:
    st.session_state.ser = None
if 'connected' not in st.session_state:
    st.session_state.connected = False
if 'last_error' not in st.session_state:
    st.session_state.last_error = ""

# --- Connection Management Functions ---
## MacOS
# USCCMD_DIR = "/Users/rishabverma/Desktop/Robotics/MiniHumanoid/libraries/pololu-usb-sdk/Maestro/UscCmd"
USCCMD_DIR = "/home/rishab/Desktop/MiniHumanoid/libraries/pololu-usb-sdk/Maestro/UscCmd"
USCCMD_CMD = "UscCmd"  # No .exe needed

def connect_robot():
    st.session_state.last_error = ""

    try:
        # --- Start Serial Connection ---
        """Attempts to establish serial connection."""
        st.session_state.last_error = ""
        print("Attempting direct serial connection (assuming Maestro is pre-configured)...")
        if st.session_state.ser and st.session_state.ser.is_open:
            print("Already connected.")
            st.session_state.connected = True
            return True

        print(f"Attempting to connect to {PORT}...")
        st.session_state.ser = standing_pos_maestro.init_serial(
            port=PORT,
            baudrate=BAUDRATE,
            retries=CONNECT_RETRIES,
            delay=CONNECT_DELAY
        )
        if st.session_state.ser and st.session_state.ser.is_open:
            st.session_state.connected = True
            print("Connection successful.")
            # Set initial speeds using helper function
            if _set_initial_speeds(st.session_state.ser):
                time.sleep(0.5) # Short delay after setting speeds
                return True
            else:
                # If setting speeds fails, consider it a connection failure
                print("Failed to set initial speeds after connection.")
                disconnect_robot() # Clean up
                st.session_state.last_error = "Failed to set initial servo speeds."
                return False
        else:
            st.session_state.connected = False
            st.session_state.last_error = f"Failed to connect to {PORT} after {CONNECT_RETRIES} attempts."
            print(st.session_state.last_error)
            # Ensure ser is None if connection failed
            if st.session_state.ser:
                try:
                    st.session_state.ser.close()
                except:
                    pass # Ignore errors on close during failure
            st.session_state.ser = None
            return False
    # Removed UscCmd specific exceptions as calls are removed
    # except subprocess.CalledProcessError as e: ...
    # except FileNotFoundError as e: ...
    # except subprocess.SubprocessError as e: ...
    except Exception as e: # Catch other potential errors during serial connection
        error_msg = f"An error occurred during serial connection: {e}"
        st.session_state.last_error = error_msg
        print(error_msg)
        # Ensure cleanup if serial object exists but failed
        if st.session_state.ser:
            try: st.session_state.ser.close()
            except: pass
        st.session_state.ser = None
        st.session_state.connected = False
        return False


def disconnect_robot():
    """Closes the serial connection."""
    if st.session_state.ser and st.session_state.ser.is_open:
        try:
            # Release servos before disconnecting
            print("Releasing servos before disconnect...")
            standing_pos_maestro.release_servos(st.session_state.ser)
            time.sleep(0.5)
            st.session_state.ser.close()
            print(f"Disconnected from {PORT}.")
        except Exception as e:
            print(f"Error during disconnect: {e}")
            st.session_state.last_error = f"Error during disconnect: {e}"
    else:
        print("Already disconnected.")
    st.session_state.ser = None
    st.session_state.connected = False

# --- Helper Function to Set Initial Speeds ---
def _set_initial_speeds(ser_instance):
    """Sets the initial speed for all servos."""
    if not ser_instance or not ser_instance.is_open:
        print("[ERROR] _set_initial_speeds: Serial connection is not available.")
        return False
    print("Setting initial servo speeds...")
    all_speeds_set = True
    for channel in range(17): # Assuming 17 servos
        if not standing_pos_maestro.set_speed(ser_instance, channel, 30):
            st.warning(f"Failed to set speed for channel {channel}")
            all_speeds_set = False
    if all_speeds_set:
        print("Initial speeds set.")
        return True
    else:
        print("Warning: Failed to set speed for one or more servos.")
        st.session_state.last_error = "Failed to set speed for one or more servos."
        return False

# --- Function to Restore Maestro Defaults and Reconfigure ---
def restore_maestro_defaults():
    """Executes UscCmd --restoredefaults, then --configure, then sets speeds."""
    st.session_state.last_error = ""
    env = os.environ.copy()
    env["DYLD_LIBRARY_PATH"] = "/opt/homebrew/Cellar/libusb/1.0.28/lib"
    restore_ok = False
    configure_ok = False
    speeds_set_ok = False

    # 1. Restore Defaults
    print("--- Starting Restore Defaults Sequence ---")
    print("Step 1: Restoring Maestro defaults via UscCmd...")
    try:
        result_restore = subprocess.run(
            ["mono", USCCMD_CMD, "--restoredefaults"],
            cwd=USCCMD_DIR, check=True, capture_output=True, text=True, env=env, timeout=10
        )
        print(f"Step 1 SUCCESS: Restore defaults command successful.\nSTDOUT: {result_restore.stdout}\nSTDERR: {result_restore.stderr}")
        restore_ok = True
    except subprocess.CalledProcessError as e:
        error_msg = (f"Step 1 FAILED: Failed to restore defaults: {e}\nSTDOUT:\n{e.stdout}\nSTDERR:\n{e.stderr}")
        print(error_msg)
        st.error(f"Failed to restore defaults: {e.stderr or e}")
        st.session_state.last_error = error_msg
        # Return immediately if restore fails
        return False
    except FileNotFoundError as e:
        error_msg = f"Step 1 FAILED: Command not found during restore: {e}. Check mono/UscCmd path."
        print(error_msg); st.error(error_msg); st.session_state.last_error = error_msg
        return False
    except subprocess.TimeoutExpired:
        error_msg = "Step 1 FAILED: Timeout during restore defaults."
        print(error_msg); st.error(error_msg); st.session_state.last_error = error_msg
        return False
    except Exception as e:
        error_msg = f"Step 1 FAILED: Unexpected error during restore defaults: {e}"
        print(error_msg); st.error(error_msg); st.session_state.last_error = error_msg
        return False

    # 2. Configure Maestro (only if restore was ok)
    if restore_ok:
        print("Step 2: Configuring Maestro via UscCmd...")
        try:
            # Increased delay before configuring
            time.sleep(1.0)
            result_config = subprocess.run(
                ["mono", USCCMD_CMD, "--configure", "maestro_config.txt"],
                cwd=USCCMD_DIR, check=True, capture_output=True, text=True, env=env, timeout=15 # Increased timeout
            )
            print(f"Step 2 SUCCESS: Configure command successful.\nSTDOUT: {result_config.stdout}\nSTDERR: {result_config.stderr}")
            configure_ok = True
        except subprocess.CalledProcessError as e:
            error_msg = (f"Step 2 FAILED: Failed to configure Maestro: {e}\nSTDOUT:\n{e.stdout}\nSTDERR:\n{e.stderr}")
            print(error_msg); st.error(f"Failed to configure Maestro: {e.stderr or e}"); st.session_state.last_error = error_msg
            # Return False as the process failed
            return False
        except FileNotFoundError as e:
            error_msg = f"Step 2 FAILED: Command/Config not found during configure: {e}."
            print(error_msg); st.error(error_msg); st.session_state.last_error = error_msg
            return False
        except subprocess.TimeoutExpired:
            error_msg = "Step 2 FAILED: Timeout during configure."
            print(error_msg); st.error(error_msg); st.session_state.last_error = error_msg
            return False
        except Exception as e:
            error_msg = f"Step 2 FAILED: Unexpected error during configure: {e}"
            print(error_msg); st.error(error_msg); st.session_state.last_error = error_msg
            return False

    # 3. Set Initial Speeds (only if restore and configure were ok)
    if configure_ok:
        print("Step 3: Re-applying initial servo speeds via serial...")
        # Increased delay before setting speeds
        time.sleep(1.0)
        speeds_set_ok = _set_initial_speeds(st.session_state.ser)
        if speeds_set_ok:
            print("Step 3 SUCCESS: Initial speeds re-applied.")
            st.toast("Maestro defaults restored, reconfigured, and speeds set.", icon="✅")
        else:
            print("Step 3 FAILED: Failed to set initial speeds after restore/configure.")
            st.error("Failed to set initial speeds after restore/configure.")
            # Return False as the full sequence didn't complete successfully
            return False
    else:
        # This path shouldn't be reached if configure failed, but added for safety
        print("Skipping speed setting because configuration failed.")
        return False

    print("--- Restore Defaults Sequence Finished ---")
    # Return True only if all steps (restore, configure, speeds) were successful
    return restore_ok and configure_ok and speeds_set_ok


# --- UI Setup ---
st.markdown('<div class="title-font">Mini Humanoid Control Panel</div>', unsafe_allow_html=True)

# Connection Status and Control
col1, col2, col3 = st.columns([1, 2, 1])
with col1:
    if st.button("🔌 Connect", key="connect_btn", disabled=st.session_state.connected):
        with st.spinner("Connecting..."):
            connect_robot()
        st.rerun() # Rerun to update UI state

with col2:
    status_placeholder = st.empty()
    if st.session_state.connected:
        status_placeholder.success(f"✅ Connected to {PORT}")
    else:
        status_placeholder.error(f"❌ Disconnected. {st.session_state.last_error}")

with col3:
    if st.button("🚫 Disconnect", key="disconnect_btn", disabled=not st.session_state.connected):
        disconnect_robot()
        st.rerun() # Rerun to update UI state


# --- Robot Control Functions (require connection) ---
def execute_robot_action(action_func, *args, success_msg="Action successful.", failure_msg="Action failed.", **kwargs):
    """
    Wrapper to execute robot actions, checking connection and handling errors.
    Passes both positional (*args) and keyword (**kwargs) arguments to the action_func.
    """
    if not st.session_state.connected or not st.session_state.ser:
        st.error("Not connected to the robot.")
        st.session_state.last_error = "Action failed: Not connected."
        return False

    try:
        print(f"Executing action: {action_func.__name__} with args: {args} and kwargs: {kwargs}")
        # Pass the serial object as the first argument, followed by other args and kwargs
        success = action_func(st.session_state.ser, *args, **kwargs)
        if success:
            print(success_msg)
            st.toast(success_msg, icon="✅")
            st.session_state.last_error = ""
            return True
        else:
            st.error(f"{failure_msg}: Command execution returned False.")
            st.session_state.last_error = f"{failure_msg}: Command execution returned False."
            return False
    except serial.SerialException as e:
        st.error(f"Serial Error during {action_func.__name__}: {e}")
        st.session_state.last_error = f"Serial Error: {e}"
        # Assume disconnection on serial error
        disconnect_robot()
        st.rerun() # Rerun to update UI after disconnect
        return False
    except Exception as e:
        st.error(f"Unexpected Error during {action_func.__name__}: {e}")
        st.session_state.last_error = f"Unexpected Error: {e}"
        return False

# --- UI Layout ---
st.markdown("---") # Separator

main_cols = st.columns(2)

with main_cols[0]:
    st.markdown('<p class="big-font">Preset Actions</p>', unsafe_allow_html=True)

    if st.button("🧍 Stand Up / Reset", key="stand_up", disabled=not st.session_state.connected):
        execute_robot_action(standing_pos_maestro.set_positions, standing_pos_maestro.standing_position, delay=2.0,
                             success_msg="Robot standing.", failure_msg="Failed to stand.")

    if st.button("🙌 Raise Hands", key="raise_hands", disabled=not st.session_state.connected):
        execute_robot_action(standing_pos_maestro.raise_hands, success_msg="Hands raised.", failure_msg="Failed to raise hands.")

    if st.button("✝️ Make Cross", key="make_cross", disabled=not st.session_state.connected):
        execute_robot_action(standing_pos_maestro.make_cross, success_msg="Cross made.", failure_msg="Failed to make cross.")

    if st.button("✌️ Make V", key="make_v", disabled=not st.session_state.connected):
        execute_robot_action(standing_pos_maestro.make_v, success_msg="V made.", failure_msg="Failed to make V.")

    walk_steps = st.number_input("Number of Walk Steps", min_value=1, max_value=10, value=2, key="walk_steps", disabled=not st.session_state.connected)
    if st.button("🚶 Walk", key="walk", disabled=not st.session_state.connected):
        execute_robot_action(standing_pos_maestro.walk, walk_steps, success_msg=f"Walked {walk_steps} steps.", failure_msg="Walking failed.")

    if st.button("🏃 Walk High Knees", key="walk_high", disabled=not st.session_state.connected):
        execute_robot_action(standing_pos_maestro.walk_high_knees, walk_steps, success_msg=f"Walked {walk_steps} high-knee steps.", failure_msg="High-knee walking failed.")

    if st.button("🙇 Bend Forward", key="bend_fwd", disabled=not st.session_state.connected):
        execute_robot_action(standing_pos_maestro.bend_forward, success_msg="Bent forward.", failure_msg="Failed to bend forward.")

    if st.button("🤸 Bend Backwards", key="bend_bwd", disabled=not st.session_state.connected):
        execute_robot_action(standing_pos_maestro.bend_backwards, success_msg="Bent backwards.", failure_msg="Failed to bend backwards.")

    if st.button("🥊 Boxing Jab", key="box_jab", disabled=not st.session_state.connected):
        execute_robot_action(standing_pos_maestro.perform_boxing_move, 'jab', success_msg="Performed jab.", failure_msg="Failed to jab.")

    if st.button("🤸‍♂️ Do Split", key="do_split", disabled=not st.session_state.connected):
        execute_robot_action(standing_pos_maestro.do_split, success_msg="Attempted split.", failure_msg="Failed to attempt split.")

    if st.button("🧘 Sit Down", key="sit_down", disabled=not st.session_state.connected):
        execute_robot_action(standing_pos_maestro.sit_down, success_msg="Attempted sit down.", failure_msg="Failed to attempt sit down.")

    if st.button("🧍‍♂️ Get Up", key="get_up", disabled=not st.session_state.connected):
        execute_robot_action(standing_pos_maestro.get_up, success_msg="Attempted get up.", failure_msg="Failed to attempt get up.")

    st.markdown("---") # Separator for new actions
    st.markdown('<p class="big-font" style="margin-top:15px;">New Actions</p>', unsafe_allow_html=True)

    if st.button("👋 Wave Hello (R)", key="wave_hello", disabled=not st.session_state.connected):
        execute_robot_action(standing_pos_maestro.wave_hello_right, success_msg="Waved hello.", failure_msg="Failed to wave hello.")

    if st.button("👍 Head Nod (Yes)", key="head_nod", disabled=not st.session_state.connected):
        execute_robot_action(standing_pos_maestro.head_nod_yes, success_msg="Nodded head.", failure_msg="Failed to nod head.")

    if st.button("↕️ Look Up/Down", key="look_up_down", disabled=not st.session_state.connected):
        execute_robot_action(standing_pos_maestro.look_up_down, success_msg="Looked up/down.", failure_msg="Failed to look up/down.")

    if st.button("🥋 Karate Chop (R)", key="karate_chop", disabled=not st.session_state.connected):
        execute_robot_action(standing_pos_maestro.karate_chop_right, success_msg="Performed karate chop.", failure_msg="Failed to perform karate chop.")

    if st.button("🙇 Bow", key="bow_action", disabled=not st.session_state.connected):
        execute_robot_action(standing_pos_maestro.bow_action, success_msg="Performed bow.", failure_msg="Failed to perform bow.")

    if st.button("🤗 Prepare Hug", key="hug_prep", disabled=not st.session_state.connected):
        execute_robot_action(standing_pos_maestro.hug_prep, success_msg="Prepared for hug.", failure_msg="Failed to prepare for hug.")

    st.markdown("---") # Separator before emergency button
    st.markdown('<div class="emergency-button" style="margin-top:15px;">', unsafe_allow_html=True)
    # Changed button action to call restore_maestro_defaults directly
    if st.button("⚠️ RESTORE DEFAULTS (Release Servos)", key="release", disabled=not st.session_state.connected):
        with st.spinner("Restoring Maestro defaults..."):
            restore_maestro_defaults()
        # No rerun needed unless state changes significantly require it
    st.markdown('</div>', unsafe_allow_html=True)

    if st.button("Release All Servos", key="release_all", disabled=not st.session_state.connected):
        execute_robot_action(standing_pos_maestro.release_servos,
                            success_msg="All servos released.",
                            failure_msg="Failed to release servos.")


with main_cols[1]:
    st.markdown('<p class="big-font">Manual Servo Control</p>', unsafe_allow_html=True)

    # Store slider values in session state to maintain position
    if 'slider_values' not in st.session_state:
        st.session_state.slider_values = standing_pos_maestro.standing_position.copy() # Initialize with standing pos

    selected_channel = st.selectbox(
        "Select Servo Channel",
        options=list(standing_pos_maestro.channel_to_joint_map.keys()),
        format_func=lambda x: f"{x}: {standing_pos_maestro.channel_to_joint_map.get(x, 'Unknown')}",
        key="servo_select",
        disabled=not st.session_state.connected
    )

    if selected_channel is not None:
        joint_name = standing_pos_maestro.channel_to_joint_map.get(selected_channel, 'Unknown')

        # Use a unique key for the slider based on the channel
        slider_key = f"slider_{selected_channel}"

        # Get current value from session state or default
        current_value = st.session_state.slider_values.get(selected_channel, 6000) # Default to 6000 if not found

        new_value = st.slider(
            f"Adjust {joint_name} (Channel {selected_channel})",
            min_value=0, # Adjust min/max based on servo limits if known
            max_value=10000,
            value=current_value,
            step=50, # Smaller step for finer control
            key=slider_key,
            disabled=not st.session_state.connected,
            help="Move the slider to set the servo position."
        )

        # Update servo only if the value has changed
        if new_value != current_value:
            st.session_state.slider_values[selected_channel] = new_value
            print(f"Slider changed for {selected_channel}: {new_value}")
            # Call set_target directly, bypassing execute_robot_action for sliders
            if st.session_state.connected and st.session_state.ser:
                if not standing_pos_maestro.set_target(st.session_state.ser, selected_channel, new_value):
                    st.warning(f"Failed to set {joint_name} directly.")
                    # Optionally add error to session state?
                    # st.session_state.last_error = f"Failed to set {joint_name}."
                # else: # Optional: Add success toast/message if needed, but might be too noisy
                #    st.toast(f"Set {joint_name} to {new_value}.", icon="✅")
            else:
                st.error("Cannot set servo: Not connected.")
            # No rerun needed here, slider updates automatically

# --- Footer ---
st.markdown("---")
st.caption("Mini Humanoid Control Panel v2.0")

# Optional: Add a cleanup function (though Streamlit's lifecycle makes this tricky)
# Consider adding a note to manually disconnect before closing the app.
st.sidebar.info("Remember to disconnect before closing the app to release the serial port.")
