
import os
import rospy
import numpy as np
import pynput
from sensor_msgs.msg import JointState

class JointStatesRecorder():
    def __init__(self, joint_state_topic_name: str, folder_name: str, dof: int):
        rospy.init_node("joint_states_recorder")
        self._rate = rospy.Rate(2)
        self._joint_states_sub = rospy.Subscriber(joint_state_topic_name, JointState, self.joint_states_callback)
        self._data = {"hole_0": [], "hole_1": []}
        self._dof = dof
        self._positions = np.zeros(self._dof)
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

    def read_data(self):
        for file_name in os.listdir(self._folder_name):
            hole_name = file_name.split(".")[0]
            file_path = os.path.join(self._folder_name, file_name)
            data = np.loadtxt(file_path, delimiter=",")
            self._data[hole_name] = data.tolist()

    def print_info(self):
        self.log("Press 'a' to add a data point")
        self.log("Press 'd' to delete the last data point")
        self.log("Press 's' to switch between holes")
        self.log("Press 'q' to save the data and quit.")

    def log(self, msg: str):
        #rospy.loginfo(msg)
        print(msg)

    def hole_name(self):
        return f"hole_{self._active_hole}"

    def on_press(self, key):
        # Add the current position to the data if the space key is pressed
        if key.char == 'a':
            self._data[self.hole_name()].append(self._positions)
            self.log(f"Addded data point for {self.hole_name()} with value {self._positions}")
        elif key.char == 'd':
            self._data[self.hole_name()] = self._data[self.hole_name()][0:-1]
            self.log(f"Deleted data point for {self.hole_name()}")
        elif key.char == 'q':
            self.log("Saving data to file and exiting")
            self.save_data()
            rospy.signal_shutdown("Finished recording data")
        elif key.char == 's':
            self._active_hole = (self._active_hole + 1) % 2
            self.log(f"Switched to hole {self._active_hole}")
        elif key.char == 'i':
            self.print_info()
        self.print_status()

    def print_status(self):
        self.log(f"Saved joint for hole 0:")
        self.log_joint_length(len(self._data['hole_0']))
        self.log(f"Saved joint for 1: {len(self._data['hole_1'])}")
        self.log_joint_length(len(self._data['hole_1']))
        self.log(f"Active hole:")
        self.print_circle_with_number(self.hole_name()[-1])

    def log_joint_length(self, length):
        # Calculate the length of the array
        # length = len(self._data['hole_0'])

        # Create an ASCII block representation of the length
        block = '█' * length  # Each '█' represents one unit in the length

        # Log the message with the ASCII block and the value at the end
        self.log(f"{block} ({length})")
    def print_circle_with_number(self, number):
        # ASCII art for a square
        square = f"""
        ███████████
        █         █
        █    {number}    █
        █         █
        ███████████
        """
        self.log(square)
    def save_data(self):
        for hole_name, data in self._data.items():
            file_name = os.path.join(self._folder_name, f"{hole_name}.csv")
            np.savetxt(file_name, np.array(data), delimiter=",")
            self.log(f"Saved {len(self._data['hole_0'])} points for {hole_name}")
            
    def joint_states_callback(self, msg: JointState):
        self._positions = np.array(msg.position)[0:self._dof]

    def run(self):
        while not rospy.is_shutdown():
            self._rate.sleep()
        rospy.signal_shutdown("Finished recording data")
