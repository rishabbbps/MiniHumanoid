# Mini Humanoid Robot Control

This project provides a control panel application for a mini humanoid robot, allowing users to connect to the robot, execute predefined movements and poses, and manually control individual servos.

The control panel is built using Streamlit and communicates with the robot's servo controller (assumed to be a Pololu Maestro or compatible) via a serial connection.

## Features

*   **Connect/Disconnect:** Establish and terminate the serial connection to the robot.
*   **Preset Actions:** Execute a variety of predefined movements and poses (e.g., Stand Up, Walk, Raise Hands, Sit Down, Wave Hello, Head Nod, etc.).
*   **Manual Servo Control:** Adjust the target position of individual servos using sliders.
*   **Restore Defaults:** Option to restore the Maestro controller to its default configuration and reapply initial settings.

## Prerequisites

*   A mini humanoid robot equipped with servos.
*   A compatible servo controller (e.g., Pololu Maestro) connected to your computer via USB.
*   Python 3.x installed.
*   The required Python libraries (see `requirements.txt`).
*   The Pololu USB SDK, specifically the `UscCmd` utility, configured and accessible if you need to use the restore defaults functionality that interacts with the Maestro's configuration. The `robot_control_app.py` assumes `UscCmd` is in `/Users/rishabverma/Desktop/Robotics/MiniHumanoid/libraries/pololu-usb-sdk/Maestro/UscCmd`.

## Setup

1.  **Clone the repository:**
    ```bash
    git clone <repository_url>
    cd MiniHumanoid
    ```
2.  **Install dependencies:**
    ```bash
    pip install -r requirements.txt
    ```
3.  **Connect your robot:** Connect your servo controller to your computer via USB. Note the serial port it is assigned (e.g., `/dev/cu.usbmodemXXXXXXX` on macOS/Linux, `COMx` on Windows). You may need to update the `PORT` variable in `examples/robot_control_app.py` if it's different from the default.
4.  **(Optional) Configure Maestro:** If you need to use the "Restore Defaults" button in the app, ensure the Pololu USB SDK is installed and `UscCmd` is correctly configured and accessible from the path specified in `examples/robot_control_app.py`. You may also need a `maestro_config.txt` file in the `UscCmd` directory with your desired default settings.

## Running the Control Panel

Navigate to the `examples` directory in your terminal:

```bash
cd examples
```

Run the Streamlit application:

```bash
streamlit run robot_control_app.py
```

This will open the control panel in your web browser.

## Project Structure

*   `examples/robot_control_app.py`: The main Streamlit application script for the control panel.
*   `examples/standing_pos.py`: Contains functions for serial communication, servo control, predefined robot poses, and movement sequences.
*   `requirements.txt`: Lists the Python dependencies required for the project.
*   `.gitignore`: Specifies intentionally untracked files that Git should ignore.
*   `.vscode/launch.json`: VS Code launch configurations.
*   `examples/maestro.py`: (Based on file listing) Likely contains additional Maestro-specific code or utilities, though not directly used in the main app based on the provided code.
*   `examples/neutralize_servos.ipynb`, `examples/neutralize_servos.py`: (Based on file listing) Likely scripts or notebooks for neutralizing or centering servos.
*   `todo.md`: A markdown file for tracking tasks or notes.

## Defined Poses and Movements (`standing_pos.py`)

The `standing_pos.py` file defines various positions and sequences, including:

*   `standing_position`: The default standing pose.
*   `init_serial`: Initializes the serial connection.
*   `set_speed`, `set_target`: Functions for controlling individual servo speed and position.
*   `set_positions`: Sets multiple servo positions simultaneously.
*   `release_servos`: Sets all servo targets to 0 to release them.
*   `raise_hands`: Moves arms to a raised position.
*   `make_cross`: Forms a cross shape with arms.
*   `make_v`: Forms a V shape with arms.
*   `adjust_balance`: Helper for balance during walking.
*   `walk_step`: Performs a single walking step.
*   `walk`: Executes a walking sequence.
*   `walk_high_knees`: Executes a high-knee walking sequence.
*   `bend_forward`, `bend_backwards`: Bends the robot at the waist.
*   `perform_boxing_move`: Performs a basic boxing move (jab).
*   `do_split`: Attempts a split pose.
*   `sit_down`: Moves to a sitting position.
*   `get_up`: Moves from sitting to standing.
*   `wave_hello_right`: Waves with the right hand.
*   `head_nod_yes`: Performs a head nod.
*   `look_up_down`: Looks up and down with the head.
*   `karate_chop_right`: Performs a right-hand karate chop.
*   `bow_action`: Performs a bow.
*   `hug_prep`: Moves arms to a hug-ready position.
*   `channel_to_joint_map`: A dictionary mapping servo channel numbers to joint names.

## Contributing

(Add information on how others can contribute if this is an open-source project)

## License

(Add license information)


