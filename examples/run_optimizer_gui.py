import os
import tkinter as tk
from tkinter import filedialog
from calibrate_fk.parameter_optimizer import ParameterOptimizer


class OptimizerApp:
    _end_link = None
    _root_link = None
    def __init__(self, root):
        self.root = root
        self.root.title("Parameter Optimizer GUI")

        # Initialize the optimizer instance
        self.optimizer = ParameterOptimizer()

        # Variables to store selected file paths
        self.urdf_file = tk.StringVar(value="No file selected")
        self.data_file = tk.StringVar(value="No file selected")
        self.out_urdf_file = tk.StringVar(value="No file selected")




        # Add buttons and labels for each step
        self.urdf_text_field = self.create_file_selector("URDF Model:", self.urdf_file, self.select_urdf_file)

        # Add fields to specify the root link and end link

        selector_frame = tk.Frame(self.root)
        selector_frame.pack()

        root_link_frame = tk.Frame(selector_frame)
        root_link_frame.pack(side=tk.LEFT)
        self.root_link_selector = tk.Listbox(root_link_frame, selectmode=tk.SINGLE, height=10)
        tk.Label(root_link_frame, text="Root Link:").pack(side=tk.TOP)
        self.root_link_selector.pack(side=tk.TOP)
        button_root_link = tk.Button(root_link_frame, text="Confirm Root Link", command=self.confirm_root_link)
        button_root_link.pack(side=tk.TOP)

        end_link_frame = tk.Frame(selector_frame)
        end_link_frame.pack(side=tk.LEFT)
        self.end_link_selector = tk.Listbox(end_link_frame, selectmode=tk.SINGLE, height=10)
        tk.Label(end_link_frame, text="End Link:").pack(side=tk.TOP)
        self.end_link_selector.pack(side=tk.TOP)
        button_end_link = tk.Button(end_link_frame, text="Confirm End Link", command=self.confirm_end_link)
        button_end_link.pack(side=tk.TOP)
        # joint selector
        joint_selector_frame = tk.Frame(selector_frame)
        joint_selector_frame.pack(side=tk.LEFT)
        tk.Label(joint_selector_frame, text="Joint Selector:").pack(side=tk.TOP)
        self.joint_selector = tk.Listbox(joint_selector_frame, selectmode=tk.MULTIPLE, height=10)
        self.joint_selector.pack()
        button_joint_selector = tk.Button(joint_selector_frame, text="Confirm Selection", command=self.get_selected_items)
        button_joint_selector.pack(side=tk.TOP)

        self.data_text_field = self.create_file_selector("Data File:", self.data_file, self.select_data_file)

        eval_frame = tk.Frame(self.root)
        eval_frame.pack()
        button_eval = tk.Button(eval_frame, text="Evaluate Model", command=self.evaluate_fks_verbose)
        button_optimize = tk.Button(eval_frame, text="Optimize", command=self.optimize)
        button_write = tk.Button(eval_frame, text="Modify URDF", command=self.modify_urdf_parameters)
        button_eval.pack(side=tk.LEFT)
        button_optimize.pack(side=tk.LEFT)
        button_write.pack(side=tk.LEFT)

        numbers_frame = tk.Frame(self.root)
        numbers_frame.pack()

        mean_1_frame = tk.Frame(numbers_frame)
        mean_1_frame.pack(side=tk.LEFT)
        tk.Label(mean_1_frame, text="Mean 1:").pack(side=tk.TOP)
        self._mean_1_field = tk.Text(mean_1_frame, height=1, width=20)
        self._mean_1_field.pack(side=tk.TOP)

        var_1_frame = tk.Frame(numbers_frame)
        var_1_frame.pack(side=tk.LEFT)
        tk.Label(var_1_frame, text="Var 1:").pack(side=tk.TOP)
        self._var_1_field = tk.Text(var_1_frame, height=1, width=20)
        self._var_1_field.pack(side=tk.TOP)

        mean_2_frame = tk.Frame(numbers_frame)
        mean_2_frame.pack(side=tk.LEFT)
        tk.Label(mean_2_frame, text="Mean_2:").pack(side=tk.TOP)
        self._mean_2_field = tk.Text(mean_2_frame, height=1, width=20)
        self._mean_2_field.pack(side=tk.TOP)

        var_2_frame = tk.Frame(numbers_frame)
        var_2_frame.pack(side=tk.LEFT)
        tk.Label(var_2_frame, text="Var 2:").pack(side=tk.TOP)
        self._var_2_field = tk.Text(var_2_frame, height=1, width=20)
        self._var_2_field.pack(side=tk.TOP)

        distance_frame = tk.Frame(numbers_frame)
        distance_frame.pack(side=tk.LEFT)
        tk.Label(distance_frame, text="Distance:").pack(side=tk.TOP)
        self._distance_field = tk.Text(distance_frame, height=1, width=10)
        self._distance_field.pack(side=tk.TOP)



        # Text widget for logs
        self.log_widget = tk.Text(self.root, height=10, width=60)
        self.log_widget.pack(pady=10)

    def get_selected_items(self):
        # Get selected indices
        selected_indices = self.joint_selector.curselection()
        # Get corresponding values
        selected_items = [self.joint_selector.get(i) for i in selected_indices]
        self.optimizer.create_parameters(selected_items)
        self.optimizer.create_fk_expression()

    def log(self, message):
        """Log messages to the text widget."""
        self.log_widget.insert(tk.END, message + "\n")
        self.log_widget.see(tk.END)  # Scroll to the bottom

    def create_button(self, label, command):
        """Helper function to create a button."""
        button = tk.Button(self.root, text=label, command=command)
        button.pack(pady=5)

    def create_file_selector(self, label, variable, command) -> tk.Text:
        """Create a file selector with a label, entry, and browse button."""
        frame = tk.Frame(self.root)
        frame.pack(pady=5, fill=tk.X)
        tk.Label(frame, text=label, width=15, anchor="w").pack(side=tk.LEFT)
        tk.Button(frame, text="Browse", command=command).pack(side=tk.LEFT)

        text_box = tk.Text(frame, height=1, width=100)
        text_box.pack(side=tk.LEFT)
        return text_box

    # File selection methods
    def select_urdf_file(self):
        file_path = filedialog.askopenfilename(
            title="Select a URDF Model",
            initialdir=os.path.join(os.path.dirname(os.path.abspath(__file__)), "../assets"),
            filetypes=[("URDF Files", "*.urdf"), ("All Files", "*.*")]
        )
        if file_path:
            self.urdf_file.set(file_path)
        self.optimizer.load_model(filename=self.urdf_file.get())
        self.log(f"Model loaded successfully from {self.urdf_file.get()}.")
        self.urdf_text_field.delete(1.0, tk.END)
        self.urdf_text_field.insert(tk.END, self.urdf_file.get())
        self.root_link_selector.delete(0, tk.END)
        self.end_link_selector.delete(0, tk.END)
        for link in self.optimizer.available_links:
            self.root_link_selector.insert(tk.END, link)
            self.end_link_selector.insert(tk.END, link)
        self.root_link_selector.pack()
        self.end_link_selector.pack()


    def select_data_file(self):
        default_data_directory = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../ros_ws")
        data_folder = filedialog.askdirectory(
            title="Select a data folder.",
            initialdir=default_data_directory,
        )
        self.optimizer.read_data(data_folder)
        self.log(f"Data read successfully from {data_folder}.")
        self.data_text_field.delete(1.0, tk.END)
        self.data_text_field.insert(tk.END, data_folder)

    def confirm_root_link(self):
        try:
            self._root_link = self.root_link_selector.get(self.root_link_selector.curselection()[0])
            self.log(f"Selected root link: {self._root_link}")
        except Exception as e:
            self.log(f"Error: {e}")
        if self._root_link is not None and self._end_link is not None:
            self.create_symbolic_fk()

    def confirm_end_link(self):
        try:
            self._end_link = self.end_link_selector.get(self.end_link_selector.curselection()[0])
            self.log(f"Selected end link: {self._end_link}")
        except Exception as e:
            self.log(f"Error: {e}")
        if self._root_link is not None and self._end_link is not None:
            self.create_symbolic_fk()


    def create_symbolic_fk(self):
        try:
            self.log(f"Selected root link: {self._root_link}")
            self.log(f"Selected end link: {self._end_link}")
            self.optimizer.create_symbolic_fk(self._root_link, self._end_link)
            self.log("Symbolic FK created.")
        except Exception as e:
            self.log(f"Error: {e}")
        self.joint_selector.delete(0, tk.END)
        for joint in self.optimizer.active_joints:
            self.joint_selector.insert(tk.END, joint)

    def evaluate_fks_verbose(self):
        try:
            kpis = self.optimizer.evaluate_fks(verbose=True)
            self._mean_1_field.delete(1.0, tk.END)
            self._mean_1_field.insert(tk.END, kpis["mean_1"])
            self._var_1_field.delete(1.0, tk.END)
            self._var_1_field.insert(tk.END, kpis["var_1"])
            self._mean_2_field.delete(1.0, tk.END)
            self._mean_2_field.insert(tk.END, kpis["mean_2"])
            self._var_2_field.delete(1.0, tk.END)
            self._var_2_field.insert(tk.END, kpis["var_2"])
            self._distance_field.delete(1.0, tk.END)
            self._distance_field.insert(tk.END, kpis["distance"])
            self.log("FKs evaluated (verbose).")
        except Exception as e:
            self.log(f"Error: {e}")

    def optimize(self):
        try:
            self.optimizer.optimize()
            self.log("Optimization complete.")
        except Exception as e:
            self.log(f"Error: {e}")

    def modify_urdf_parameters(self):
        try:
            self.optimizer.modify_urdf_parameters('test.urdf')
            self.log("URDF parameters modified.")
        except Exception as e:
            self.log(f"Error: {e}")


if __name__ == "__main__":
    root = tk.Tk()
    app = OptimizerApp(root)
    root.mainloop()

