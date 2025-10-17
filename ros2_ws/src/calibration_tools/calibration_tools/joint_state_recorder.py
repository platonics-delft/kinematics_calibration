import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
import pynput
from sensor_msgs.msg import JointState

class JointStatesRecorder(Node):
    def __init__(self, joint_state_topic_name: str, folder_name: str, joint_names: list = None):
        super().__init__("joint_states_recorder")
        
        # Create QoS profile for subscription
        qos_profile = QoSProfile(depth=10)
        
        self._timer = self.create_timer(0.5, self.timer_callback)  # 2 Hz equivalent
        self._joint_states_sub = self.create_subscription(
            JointState, 
            joint_state_topic_name, 
            self.joint_states_callback,
            qos_profile
        )
        
        self._data = {"hole_0": [], "hole_1": []}
        self._joint_names = joint_names  # Store the desired joint names
        self._positions = None  # Will be initialized when we know the size
        self._initialized = False  # Track if we've received the first message
        self._active_hole = 0
        self._pykeyboard = pynput.keyboard.Listener(on_press=self.on_press)
        self._pykeyboard.start()
        self._folder_name = folder_name
        
        if os.path.exists(folder_name):
            self.log(f"Folder {self._folder_name} already exists.")
            if len(os.listdir(folder_name)) != 0:
                self.read_data()
            else:
                self._data = {"hole_0": [], "hole_1": [], 'hole_0_pose': [], 'hole_1_pose': []}
        else:
            self.log(f"Creating folder {self._folder_name}")
            os.makedirs(self._folder_name, exist_ok=True)
            self._data = {"hole_0": [], "hole_1": [], 'hole_0_pose': [], 'hole_1_pose': []}

        self.print_info()
        self.print_status()

    def timer_callback(self):
        # This replaces the rospy.Rate.sleep() functionality
        pass

    def print_info(self):
        self.log("Press 'a' to add a data point")
        self.log("Press 'd' to delete the last data point")
        self.log("Press 's' to switch between holes")
        self.log("Press 'q' to save the data and quit.")

    def log(self, msg: str):
        self.get_logger().info(msg)
        print(msg)

    def hole_name(self):
        return f"hole_{self._active_hole}"

    def on_press(self, key):
        try:
            # Check if we have initialized positions
            if not self._initialized or self._positions is None:
                self.log("Waiting for joint state data... Please ensure the robot is publishing joint states.")
                return
                
            # Add the current position to the data if the space key is pressed
            if key.char == 'a':
                self._data[self.hole_name()].append(self._positions.tolist())
                self.log(f"Added data point for {self.hole_name()} with value {self._positions}")
            elif key.char == 'd':
                if len(self._data[self.hole_name()]) > 0:
                    self._data[self.hole_name()] = self._data[self.hole_name()][0:-1]
                    self.log(f"Deleted data point for {self.hole_name()}")
            elif key.char == 'q':
                self.log("Saving data to file and exiting")
                self.save_data()
                rclpy.shutdown()
            elif key.char == 's':
                self._active_hole = (self._active_hole + 1) % 2
                self.log(f"Switched to hole {self._active_hole}")
            elif key.char == 'i':
                self.print_info()
            self.print_status()
        except AttributeError:
            # Special keys (like arrows, ctrl, etc.) don't have char attribute
            pass

    def print_status(self):
        self.log(f"Saved joint for hole 0:")
        self.log_joint_length(len(self._data['hole_0']))
        self.log(f"Saved joint for hole 1:")
        self.log_joint_length(len(self._data['hole_1']))
        
        # Display joint names if available
        if hasattr(self, '_joint_names') and self._joint_names is not None:
            self.log(f"Joint names: {', '.join(self._joint_names)}")
        
        self.log(f"Active hole is green:")
        self.print_circle_with_number(self.hole_name()[-1])

    def log_joint_length(self, length):
        # Create an ASCII block representation of the length
        block = '█' * length  # Each '█' represents one unit in the length
        # Log the message with the ASCII block and the value at the end
        self.log(f"{block} ({length})")

    def print_circle_with_number(self, number):
        number = int(number)
        if number == 0: 
            color1 = 32
            color2 = 0
        if number == 1:
            color1 = 0
            color2 = 32
        square = f"""
            \033[{color1}m████████████   \033[{color2}m████████████
            \033[{color1}m█          █   \033[{color2}m█          █
            \033[{color1}m█     {0}    █   \033[{color2}m█     {1}    █
            \033[{color1}m█          █   \033[{color2}m█          █
            \033[{color1}m████████████   \033[{color2}m████████████ 
            \033[0m
        """
        self.log(square)

    def save_data(self):
        for hole_name, data in self._data.items():
            if len(data) > 0:  # Only save if there's data
                file_name = os.path.join(self._folder_name, f"{hole_name}.csv")
                
                # Create a structured array with joint names as header
                if hasattr(self, '_joint_names') and self._joint_names is not None:
                    # Save with header containing joint names
                    header = ','.join(self._joint_names)
                    np.savetxt(file_name, np.array(data), delimiter=",", header=header, comments='')
                else:
                    # Fallback to original behavior if joint names not available
                    np.savetxt(file_name, np.array(data), delimiter=",")
                
                self.log(f"Saved {len(data)} points for {hole_name}")

    def read_data(self):
        for file_name in os.listdir(self._folder_name):
            if not file_name.endswith('.csv'):
                continue
                
            hole_name = file_name.split(".")[0]
            file_path = os.path.join(self._folder_name, file_name)
            
            try:
                # Try to read with header
                with open(file_path, 'r') as f:
                    first_line = f.readline().strip()
                    if first_line.startswith('#'):
                        # File has header with joint names
                        joint_names = first_line[1:].split(',')  # Remove # and split
                        data = np.loadtxt(file_path, delimiter=",", skiprows=1)
                        
                        # Store joint names if this is the first time reading
                        if not hasattr(self, '_joint_names') or self._joint_names is None:
                            self._joint_names = joint_names
                    else:
                        # File has no header, read normally
                        data = np.loadtxt(file_path, delimiter=",")
                
                # Handle single row case
                if data.ndim == 1:
                    data = data.reshape(1, -1)
                    
                self._data[hole_name] = data.tolist()
                self.log(f"Loaded {len(data)} points for {hole_name}")
                
            except Exception as e:
                self.log(f"Error reading {file_path}: {e}")
                # Fallback to original method
                data = np.loadtxt(file_path, delimiter=",")
                if data.ndim == 1:
                    data = data.reshape(1, -1)
                self._data[hole_name] = data.tolist()
            
    def joint_states_callback(self, msg: JointState):
        # Initialize on first message if needed
        if not self._initialized:
            if self._joint_names is None:
                # Use all joints from the topic
                self._joint_names = list(msg.name)
                self.log(f"Auto-detected {len(self._joint_names)} joints: {self._joint_names}")

            self._positions = np.zeros(len(self._joint_names))
            self._initialized = True
        
        # Filter joints based on the joint names
        positions = []
        for joint_name in self._joint_names:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                positions.append(msg.position[idx])
            else:
                self.get_logger().warn(f"Joint {joint_name} not found in message")
                positions.append(0.0)  # Default value if joint not found
        self._positions = np.array(positions)

    def run(self):
        rclpy.spin(self)