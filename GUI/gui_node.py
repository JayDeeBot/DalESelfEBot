#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from std_msgs.msg import String as RosString
from cv_bridge import CvBridge, CvBridgeError
import cv2
import tkinter as tk
from tkinter import messagebox, Toplevel, DISABLED, NORMAL, FLAT
from tkinter import simpledialog # For Set Robot IP
from PIL import Image as PILImage, ImageTk
import threading
import time
import numpy as np
import sys
import subprocess
import os
import traceback
import yaml # For Set Robot IP
import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

## Setup Image Processor Action Client ##
# Try to import the action definition
try:
    from img_prc_interface.action import Img
    from action_msgs.msg import GoalStatus
except ImportError:
    print("Error: Could not import Img action or GoalStatus.")
    print("Ensure the img_prc_interface package is built and sourced,")
    print("and that action_msgs is available.")
    Img = None
    GoalStatus = None

# Define the ROS 2 Node for GUI interaction
class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.get_logger().info('GUI Node started')
        self.bridge = CvBridge()

        self.latest_webcam_frame = None
        self.webcam_frame_lock = threading.Lock()
        self.latest_edge_frame = None
        self.edge_frame_lock = threading.Lock()
        self.latest_processed_frame = None
        self.processed_frame_lock = threading.Lock()
        self.processing_active = False
        self.canvas_localisation_process = None
        self.robot_calibration_process = None
        self.dale_integration_process = None
        self.ur_system_process = None

        self.latest_error_message = None
        self.error_message_lock = threading.Lock()

        self._send_goal_future = None
        self._get_result_future = None

        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_main_folder = os.path.dirname(script_dir)
        self.params_file_path = os.path.join(project_main_folder, "/home/jarred/git/DalESelfEBot/GUI/params.yaml")
        self.get_logger().info(f"Parameters YAML file path set to: {self.params_file_path}")

        self.pixel_map_publisher = self.create_publisher(Image, '/pixel_map', 10)
        self.save_pos_publisher = self.create_publisher(Empty, '/save_position', 10)
        self.service_ee_publisher = self.create_publisher(Empty, '/service_ee', 10)
        self.get_logger().info("Publishers created for /pixel_map, /save_position, and /service_ee")

        self.webcam_subscription = self.create_subscription(
            Image, '/webcam_image', self.webcam_callback, 10)
        self.edge_subscription = self.create_subscription(
            Image, '/edge_image', self.edge_callback, 10)
        self.error_subscription = self.create_subscription(
            RosString, '/control_error', self.error_message_callback, 10)
        self.get_logger().info("Subscribed to /webcam_image, /edge_image, and /control_error")

        if Img is not None:
            self._action_client = ActionClient(self, Img, '/process_edge_image')
            self.get_logger().info("Action client created for /process_edge_image.")
        else:
            self._action_client = None
            self.get_logger().error("Img action type not available. Action client NOT created.")

    ## ------- Image Processor Section --------- ##
    def webcam_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.webcam_frame_lock:
                self.latest_webcam_frame = cv_image
        except CvBridgeError as e:
            self.get_logger().error(f'Webcam CV Bridge Error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error in webcam_callback: {e}\n{traceback.format_exc()}')

    def edge_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            with self.edge_frame_lock:
                self.latest_edge_frame = cv_image
        except CvBridgeError as e:
            self.get_logger().error(f'Edge CV Bridge Error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error in edge_callback: {e}\n{traceback.format_exc()}')

    def error_message_callback(self, msg):
        try:
            with self.error_message_lock:
                self.latest_error_message = msg.data
        except Exception as e:
            self.get_logger().error(f'Error in error_message_callback: {e}\n{traceback.format_exc()}')

    def get_latest_webcam_frame(self):
        with self.webcam_frame_lock:
            return self.latest_webcam_frame.copy() if self.latest_webcam_frame is not None else None

    def get_latest_edge_frame(self):
        with self.edge_frame_lock:
            return self.latest_edge_frame.copy() if self.latest_edge_frame is not None else None

    def get_latest_processed_frame(self):
        with self.processed_frame_lock:
            return self.latest_processed_frame.copy() if self.latest_processed_frame is not None else None

    def get_latest_error_message(self):
        with self.error_message_lock:
            return self.latest_error_message

    def send_process_image_goal(self):
        if self._action_client is None:
            self.get_logger().error("Action client not available.")
            return False
        if not self._action_client.server_is_ready():
            self.get_logger().warn("Action server '/process_edge_image' not available yet.")
            return False
        with self.webcam_frame_lock:
            frame_to_send = self.latest_webcam_frame
            if frame_to_send is None:
                self.get_logger().warn("No webcam frame available to send.")
                return False
            frame_copy = frame_to_send.copy()
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame_copy, encoding='bgr8')
        except CvBridgeError as e:
             self.get_logger().error(f"CV Bridge error converting frame for action: {e}")
             return False
        except Exception as e:
             self.get_logger().error(f"Unexpected error converting frame for action: {e}\n{traceback.format_exc()}")
             return False
        goal_msg = Img.Goal()
        goal_msg.image = image_msg
        self.get_logger().info('Sending goal request...')
        self.processing_active = True
        with self.processed_frame_lock:
            self.latest_processed_frame = None
        if not self._action_client.server_is_ready():
             self.get_logger().warn("Action server became unavailable before sending goal.")
             self.processing_active = False
             return False
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        goal_handle = None
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Exception while getting goal handle: {e}\n{traceback.format_exc()}')
            self.processing_active = False
            return
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().info('Goal rejected or failed :(')
            self.processing_active = False
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        pass

    def get_result_callback(self, future):
        processed_cv_image = None
        if GoalStatus is None:
             self.get_logger().error("GoalStatus message type not available. Cannot check action result status.")
             self.processing_active = False
             return
        try:
            result_response = future.result()
            if result_response is None:
                self.get_logger().error("Future for get_result_async returned None.")
                self.processing_active = False
                return
            result = result_response.result
            status = result_response.status
            if status == GoalStatus.STATUS_SUCCEEDED and result and result.processed_image:
                self.get_logger().info(f'Result received (status: {status}).')
                processed_cv_image = self.bridge.imgmsg_to_cv2(result.processed_image, desired_encoding='mono8')
            else:
                if status != GoalStatus.STATUS_SUCCEEDED:
                     self.get_logger().warn(f'Action did not succeed. Status: {status}')
                elif not result or not result.processed_image:
                     self.get_logger().warn(f'Action succeeded but returned no processed image (status: {status}).')
                else:
                     self.get_logger().warn(f'Action finished with unexpected state (status: {status}).')
        except CvBridgeError as e:
             self.get_logger().error(f'CV Bridge error processing action result image: {e}')
        except Exception as e:
             self.get_logger().error(f'Error processing action result: {e}\n{traceback.format_exc()}')
        finally:
            with self.processed_frame_lock:
                self.latest_processed_frame = processed_cv_image
            self.processing_active = False
            self.get_logger().info(f'Processing finished. Processed frame is {"available" if processed_cv_image is not None else "not available"}.')

    def publish_processed_image(self):
        frame_to_publish = None
        with self.processed_frame_lock:
            if self.latest_processed_frame is not None:
                frame_to_publish = self.latest_processed_frame.copy()
        if frame_to_publish is None:
            self.get_logger().warn("No processed image available to publish.")
            return False
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame_to_publish, encoding='mono8')
            self.pixel_map_publisher.publish(image_msg)
            self.get_logger().info(f"Published processed image to {self.pixel_map_publisher.topic_name}")
            return True
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error converting processed frame for publishing: {e}")
            return False
        except Exception as e:
            self.get_logger().error(f"Unexpected error publishing processed frame: {e}\n{traceback.format_exc()}")
            return False

    def publish_save_position(self):
        try:
            msg = Empty()
            self.save_pos_publisher.publish(msg)
            self.get_logger().info(f"Published Empty message to {self.save_pos_publisher.topic_name}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to publish to /save_position: {e}\n{traceback.format_exc()}")
            return False

    def publish_service_ee(self):
        try:
            msg = Empty()
            self.service_ee_publisher.publish(msg)
            self.get_logger().info(f"Published Empty message to {self.service_ee_publisher.topic_name}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to publish to /service_ee: {e}\n{traceback.format_exc()}")
            return False

    def launch_canvas_localisation_node(self):
        if self.canvas_localisation_process and self.canvas_localisation_process.poll() is None:
            self.get_logger().warn("Canvas localisation node might already be running.")
            return False, "Canvas localisation node might already be running."
        command = ['ros2', 'run', 'ur3_localisation', 'localisation_node']
        try:
            current_env = os.environ.copy()
            self.canvas_localisation_process = subprocess.Popen(command, env=current_env)
            self.get_logger().info(f"Launched command: {' '.join(command)}")
            return True, "Launched localisation_node."
        except FileNotFoundError:
            self.get_logger().error("Error: 'ros2 run' command not found. Is ROS 2 environment sourced correctly?")
            self.canvas_localisation_process = None
            return False, "Error: 'ros2 run' command not found."
        except Exception as e:
            self.get_logger().error(f"Failed to launch localisation_node: {e}\n{traceback.format_exc()}")
            self.canvas_localisation_process = None
            return False, f"Failed to launch node: {e}"

    def launch_robot_calibration_launchfile(self):
        if self.robot_calibration_process and self.robot_calibration_process.poll() is None:
            self.get_logger().warn("Robot calibration launch file might already be running.")
            return False, "Robot calibration might already be running."

        target_file = "calibration.yaml"
        if not os.path.exists(target_file):
            self.get_logger().error(f"Calibration target file not found: {target_file}")
            return False, f"Calibration target file not found: {target_file}"

        robot_ip, ur_type, msg = self.get_robot_info_from_yaml()
        if not robot_ip:
            robot_ip = "192.168.0.190"  # Fallback IP
            self.get_logger().warn(f"Robot IP for calibration not found in YAML, using fallback: {robot_ip}")

        command = [
            'ros2', 'launch', 'ur_calibration', 'calibration_correction.launch.py',
            f'robot_ip:={robot_ip}',
            f'target_filename:={target_file}'
        ]
        try:
            current_env = os.environ.copy()
            self.robot_calibration_process = subprocess.Popen(command, env=current_env)
            self.get_logger().info(f"Launched command: {' '.join(command)}")
            return True, "Launched robot calibration."
        except FileNotFoundError:
            self.get_logger().error("Error: 'ros2 launch' command not found. Is ROS 2 environment sourced correctly?")
            self.robot_calibration_process = None
            return False, "Error: 'ros2 launch' command not found."
        except Exception as e:
            self.get_logger().error(f"Failed to launch robot calibration: {e}\n{traceback.format_exc()}")
            self.robot_calibration_process = None
            return False, f"Failed to launch calibration: {e}"

    def get_robot_info_from_yaml(self):
        try:
            with open("params.yaml", "r") as file:  # Replace with actual path
                config = yaml.safe_load(file)
            robot_ip = config["robot"]["ip_address"]
            ur_type = config["robot"]["ur_type"]
            return robot_ip, ur_type, ""
        except Exception as e:
            return None, None, str(e)

    def launch_ur_system(self):
        if self.ur_system_process and self.ur_system_process.poll() is None:
            self.get_logger().warn("UR system (ur_control.launch.py) might already be running.")
            return False, "UR system might already be running."

        robot_ip, ur_type, msg = self.get_robot_info_from_yaml()
        if not robot_ip or not ur_type:
            self.get_logger().error(f"Cannot launch UR system: {msg}")
            return False, f"Failed to get robot info: {msg}"

        kinematics_config_path = "calibration.yaml"
        if not os.path.exists(kinematics_config_path):
            err_msg = f"Kinematics config file not found: {kinematics_config_path}. Please update the path in the script (GuiNode.launch_ur_system)."
            self.get_logger().error(err_msg)
            return False, err_msg

        command = [
            'ros2', 'launch', 'ur_robot_driver', 'ur_control.launch.py',
            f'ur_type:={ur_type}',
            f'robot_ip:={robot_ip}',
            'launch_rviz:=true',
            f'kinematics_config:={kinematics_config_path}'
        ]
        try:
            current_env = os.environ.copy()
            self.ur_system_process = subprocess.Popen(command, env=current_env)
            self.get_logger().info(f"Launched UR system command: {' '.join(command)}")
            return True, "Launched UR system (ur_control.launch.py)."
        except FileNotFoundError:
            err_msg = "Error: 'ros2 launch' command not found or 'ur_robot_driver' package not found. Is ROS 2 environment sourced and package installed?"
            self.get_logger().error(err_msg)
            self.ur_system_process = None
            return False, err_msg
        except Exception as e:
            self.get_logger().error(f"Failed to launch UR system: {e}\n{traceback.format_exc()}")
            self.ur_system_process = None
            return False, f"Failed to launch UR system: {e}"

    def launch_dale_integration(self):
        if self.dale_integration_process and self.dale_integration_process.poll() is None:
            self.get_logger().warn("Dale integration launch file might already be running.")
            return False, "Dale integration might already be running."
        command = ['ros2', 'launch', 'dale_integration', 'dale_launch.py']
        try:
            current_env = os.environ.copy()
            self.dale_integration_process = subprocess.Popen(command, env=current_env)
            self.get_logger().info(f"Launched command: {' '.join(command)}")
            return True, "Launched Dale Integration System."
        except FileNotFoundError:
            self.get_logger().error("Error: 'ros2 launch' or 'dale_integration' package/launchfile not found.")
            self.dale_integration_process = None
            return False, "Error: 'ros2 launch' or 'dale_integration' package/launchfile not found."
        except Exception as e:
            self.get_logger().error(f"Failed to launch Dale Integration System: {e}\n{traceback.format_exc()}")
            self.dale_integration_process = None
            return False, f"Failed to launch Dale Integration: {e}"

    def set_robot_ip_in_yaml(self, ip_address):
        try:
            data = {}
            if os.path.exists(self.params_file_path):
                with open(self.params_file_path, 'r') as f:
                    try:
                        loaded_data = yaml.safe_load(f)
                        if loaded_data is not None: data = loaded_data
                    except yaml.YAMLError as e:
                        self.get_logger().error(f"Error parsing YAML {self.params_file_path}: {e}")
                        return False, f"Error parsing existing YAML: {e}"

            if not isinstance(data, dict):
                data = {}

            if 'robot' not in data or not isinstance(data.get('robot'), dict): data['robot'] = {}
            data['robot']['ip_address'] = ip_address

            with open(self.params_file_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            self.get_logger().info(f"Robot IP set to {ip_address} in {self.params_file_path}")
            return True, f"Robot IP updated to {ip_address}."
        except PermissionError:
            self.get_logger().error(f"Permission denied for {self.params_file_path}.")
            return False, "Permission denied for parameters file."
        except Exception as e:
            self.get_logger().error(f"Failed to set IP in YAML: {e}\n{traceback.format_exc()}")
            return False, f"Unexpected error: {e}"

class LocalisationWindow(Toplevel):
    def __init__(self, master, ros_node):
        super().__init__(master)
        self.ros_node = ros_node
        self.master_app = master
        self.title("Localisation")
        self.resizable(False, False)
        self.config(bg="#555555")
        self.calibration_step = 0
        self.corner_order = ['BL', 'BR', 'TR', 'TL']
        main_frame = tk.Frame(self, padx=10, pady=10, bg="#555555")
        main_frame.pack(fill=tk.BOTH, expand=True)
        button_frame = tk.Frame(main_frame, bg="#555555")
        button_frame.pack(pady=5)
        self.buttons = {}
        button_config = {
            "width": 12, "height": 2, "bg": "#303030", "fg": "white",
            "activebackground": "#454545", "relief": FLAT,
            "borderwidth": 0, "highlightthickness": 0
        }
        disabled_button_bg = "#404040"
        self.buttons['TL'] = tk.Button(button_frame, text="Top Left", **button_config, command=lambda: self.on_corner_button_click('TL'))
        self.buttons['TL'].grid(row=0, column=0, padx=5, pady=5)
        self.buttons['TR'] = tk.Button(button_frame, text="Top Right", **button_config, command=lambda: self.on_corner_button_click('TR'))
        self.buttons['TR'].grid(row=0, column=1, padx=5, pady=5)
        self.buttons['BL'] = tk.Button(button_frame, text="Bottom Left", **button_config, command=lambda: self.on_corner_button_click('BL'))
        self.buttons['BL'].grid(row=1, column=0, padx=5, pady=5)
        self.buttons['BR'] = tk.Button(button_frame, text="Bottom Right", **button_config, command=lambda: self.on_corner_button_click('BR'))
        self.buttons['BR'].grid(row=1, column=1, padx=5, pady=5)
        orientation_label = tk.Label(main_frame, text="▲\nRobot Position", font=("Arial", 10), bg="#555555", fg="white")
        orientation_label.pack(pady=(10, 5))
        self.loc_status_label = tk.Label(main_frame, text="Click Bottom Left", justify=tk.CENTER, height=2, bg="#555555", fg="white")
        self.loc_status_label.pack(pady=5, fill=tk.X)
        close_button = tk.Button(main_frame, text="Close", command=self.close_window, **button_config)
        close_button.pack(pady=10, side=tk.BOTTOM)
        self.disabled_button_bg = disabled_button_bg
        self.active_button_bg = button_config["bg"]
        self.update_button_states()
        self.protocol("WM_DELETE_WINDOW", self.close_window)
        self.transient(master)
        self.grab_set()
        self.lift()
        self.update_idletasks()
        master_x, master_y = master.winfo_rootx(), master.winfo_rooty()
        master_width, master_height = master.winfo_width(), master.winfo_height()
        win_width, win_height = self.winfo_width(), self.winfo_height()
        x, y = master_x + (master_width - win_width) // 2, master_y + (master_height - win_height) // 2
        self.geometry(f'+{x}+{y}')

    def update_button_states(self):
        expected_corner = None
        if self.calibration_step >= len(self.corner_order):
            self.loc_status_label.config(text="Calibration Complete!")
        else:
            expected_corner = self.corner_order[self.calibration_step]
            corner_name = self.buttons[expected_corner]['text']
            self.loc_status_label.config(text=f"Move robot to {corner_name}\nand click the button.")
        for corner_id, button in self.buttons.items():
            if self.calibration_step >= len(self.corner_order):
                button.config(state=DISABLED, bg=self.disabled_button_bg)
            else:
                button.config(state=NORMAL if corner_id == expected_corner else DISABLED,
                              bg=self.active_button_bg if corner_id == expected_corner else self.disabled_button_bg)

    def on_corner_button_click(self, corner_id):
        if self.calibration_step >= len(self.corner_order):
            print("Calibration already complete.")
            return
        expected_corner = self.corner_order[self.calibration_step]
        if corner_id == expected_corner:
            print(f"Correct button clicked: {corner_id}")
            if self.ros_node.publish_save_position():
                self.calibration_step += 1
                self.update_button_states()
            else:
                messagebox.showerror("Publish Error", "Failed to publish to /save_position.", parent=self)
        else:
            expected_name = self.buttons[expected_corner]['text']
            messagebox.showwarning("Incorrect Button", f"Please click the '{expected_name}' button.", parent=self)

    def close_window(self):
        print("Localisation window close requested.")
        self.grab_release()
        if self.master_app: self.master_app.on_localisation_close()
        self.destroy()

class ActionViewerApp(tk.Tk):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.settings_window, self.localisation_window = None, None
        self.title("DalESelfEBot GUI"); self.geometry("1040x640"); self.minsize(800, 500)
        self.dark_grey, self.text_color, self.error_text_color = "#404040", "white", "red"
        self.config(bg=self.dark_grey)
        self.running, self.display_mode, self.last_processed_frame_available = True, "live", False
        self.top_frame = tk.Frame(self, bg=self.dark_grey)
        self.processed_display_frame = tk.Frame(self, bg=self.dark_grey)
        self.control_frame = tk.Frame(self, bg=self.dark_grey)

        for i, (txt, lbl_attr) in enumerate([("Webcam Feed", "webcam_label"), ("Edge Feed", "edge_label")]):
            container = tk.Frame(self.top_frame, bd=0, relief=FLAT, bg=self.dark_grey)
            container.pack(side=[tk.LEFT, tk.RIGHT][i], padx=5, pady=5, fill=tk.BOTH, expand=True)
            tk.Label(container, text=txt, bg=self.dark_grey, fg=self.text_color).pack()
            setattr(self, lbl_attr, tk.Label(container, bg=self.dark_grey))
            getattr(self, lbl_attr).pack(fill=tk.BOTH, expand=True)

        proc_cont = tk.Frame(self.processed_display_frame, bd=0, relief=FLAT, bg=self.dark_grey)
        proc_cont.pack(side=tk.TOP, padx=5, pady=5, fill=tk.BOTH, expand=True)
        tk.Label(proc_cont, text="Processed Image", bg=self.dark_grey, fg=self.text_color).pack()
        self.processed_label = tk.Label(proc_cont, bg=self.dark_grey)
        self.processed_label.pack(fill=tk.BOTH, expand=True)

        btn_sfr = tk.Frame(self.control_frame, bg=self.dark_grey)
        btn_sfr.pack(side=tk.LEFT, padx=5)
        btn_cfg = {"bg": "#303030", "fg": self.text_color, "activebackground":"#454545", "relief":FLAT, "borderwidth":0, "highlightthickness":0}
        self.capture_button = tk.Button(btn_sfr, text="Capture Image", width=15, command=self.on_capture_button_click, **btn_cfg)
        self.capture_button.pack(side=tk.LEFT, padx=(0,5))
        self.draw_button = tk.Button(btn_sfr, text="Draw", width=10, command=self.on_draw_button_click, state=DISABLED, **btn_cfg)
        self.draw_button.pack(side=tk.LEFT, padx=5)
        self.settings_button = tk.Button(self.control_frame, text="Settings", width=10, command=self.open_settings_window, **btn_cfg)
        self.settings_button.pack(side=tk.RIGHT, padx=5)
        msg_bars = tk.Frame(self.control_frame, bg=self.dark_grey)
        msg_bars.pack(side=tk.LEFT, padx=10, fill=tk.X, expand=True)
        self.status_label = tk.Label(msg_bars, text="Initializing...", anchor='w', bg=self.dark_grey, fg=self.text_color)
        self.status_label.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0,5))
        self.error_status_label = tk.Label(msg_bars, text="", anchor='w', bg=self.dark_grey, fg=self.error_text_color)
        self.error_status_label.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(5,0))

        self.control_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=5, padx=5)
        self.top_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.update_thread = threading.Thread(target=self.update_image_display, daemon=True)
        self.update_thread.start(); print("GUI update thread started.")
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

    def open_settings_window(self):
        if self.settings_window and self.settings_window.winfo_exists():
            self.settings_window.lift()
            return

        self.settings_window = Toplevel(self)
        self.settings_window.title("Robot Settings")
        self.settings_window.geometry("350x440")
        self.settings_window.resizable(False, False)
        settings_bg = "#555555"
        btn_cfg = {"width":20, "bg":"#303030", "fg":"white", "activebackground":"#454545", "relief":FLAT, "borderwidth":0, "highlightthickness":0}
        self.settings_window.config(bg=settings_bg)

        s_fr = tk.Frame(self.settings_window, padx=15, pady=15, bg=settings_bg)
        s_fr.pack(fill=tk.BOTH, expand=True)

        # New Button Order:
        # 1. Set Robot IP Address
        tk.Button(s_fr, text="Set Robot IP Address", **btn_cfg, command=self.on_set_robot_ip_click).pack(pady=8)

        # 2. Calibrate Robot
        tk.Button(s_fr, text="Calibrate Robot", **btn_cfg, command=self.on_calibrate_robot_click).pack(pady=8)

        # 3. Launch UR System ("launch driver")
        self.launch_ur_system_button = tk.Button(s_fr, text="Launch UR System", **btn_cfg, command=self.on_launch_ur_system_click)
        self.launch_ur_system_button.pack(pady=8)

        # 4. Localise Canvas
        tk.Button(s_fr, text="Localise Canvas", **btn_cfg, command=self.on_localise_canvas_click).pack(pady=8)

        # 5. Launch Dale System ("launch system")
        tk.Button(s_fr, text="Launch Dale System", **btn_cfg, command=self.on_launch_dale_system_click).pack(pady=8)

        # 6. Service End Effector
        self.service_ee_button = tk.Button(s_fr, text="Service End Effector", **btn_cfg, command=self.on_service_ee_click)
        self.service_ee_button.pack(pady=8)

        # Close button (always last at the bottom)
        tk.Button(s_fr, text="Close", command=self.on_settings_close, **btn_cfg).pack(pady=12, side=tk.BOTTOM)

        self.settings_window.protocol("WM_DELETE_WINDOW", self.on_settings_close)
        self.settings_window.transient(self)
        self.settings_window.grab_set()
        self.settings_window.lift()

        self.settings_window.update_idletasks()
        mx,my,mw,mh = self.winfo_rootx(),self.winfo_rooty(),self.winfo_width(),self.winfo_height()
        ww,wh = self.settings_window.winfo_width(),self.settings_window.winfo_height()
        self.settings_window.geometry(f'+{mx+(mw-ww)//2}+{my+(mh-wh)//2}')

    def on_service_ee_click(self):
        if not hasattr(self, 'service_ee_button') or not self.service_ee_button.winfo_exists():
            self.ros_node.get_logger().error("Service EE button referred to before creation or after destruction.")
            if self.settings_window and self.settings_window.winfo_exists():
                messagebox.showerror("Error", "Service EE button not found.", parent=self.settings_window)
            return

        current_text = self.service_ee_button.cget("text")
        self.ros_node.get_logger().info(f"Service End Effector button clicked. Current state: {current_text}")

        if self.ros_node.publish_service_ee():
            if current_text == "Service End Effector":
                self.service_ee_button.config(text="Service Complete")
                self.ros_node.get_logger().info("Service EE message sent. Button changed to 'Service Complete'.")
            elif current_text == "Service Complete":
                self.service_ee_button.config(text="Service End Effector")
                self.ros_node.get_logger().info("Service EE message sent. Button changed back to 'Service End Effector'.")
        else:
            messagebox.showerror("Publish Error", "Failed to publish to /service_ee.", parent=self.settings_window if self.settings_window and self.settings_window.winfo_exists() else self)
            self.ros_node.get_logger().error("Failed to publish to /service_ee.")

    def on_set_robot_ip_click(self):
        self.ros_node.get_logger().info("Set Robot IP Address button clicked.")
        parent_window = self.settings_window if self.settings_window and self.settings_window.winfo_exists() else self
        new_ip = simpledialog.askstring("Set Robot IP", "Enter new Robot IP Address:", parent=parent_window)
        if new_ip is not None:
            new_ip = new_ip.strip()
            if not new_ip: messagebox.showwarning("Input Error", "IP address cannot be empty.", parent=parent_window); return
            success, message = self.ros_node.set_robot_ip_in_yaml(new_ip)
            if success: messagebox.showinfo("Success", message, parent=parent_window)
            else: messagebox.showerror("Error", f"Failed to set IP: {message}", parent=parent_window)
        else: self.ros_node.get_logger().info("Set Robot IP cancelled.")

    def on_launch_dale_system_click(self): # Renamed from on_launch_system_click
        self.ros_node.get_logger().info("Launch Dale System button clicked.")
        parent_win = self.settings_window if self.settings_window and self.settings_window.winfo_exists() else self
        success, message = self.ros_node.launch_dale_integration()
        if success:
            messagebox.showinfo("Dale System Launch", message, parent=parent_win)
        else:
            messagebox.showerror("Dale System Launch Error", message, parent=parent_win)

    def on_launch_ur_system_click(self):
        self.ros_node.get_logger().info("'Launch UR System' button clicked.")
        parent_win = self.settings_window if self.settings_window and self.settings_window.winfo_exists() else self

        success, message = self.ros_node.launch_ur_system()

        if success:
            messagebox.showinfo("UR System Launch", message, parent=parent_win)
        else:
            messagebox.showerror("UR System Launch Error", message, parent=parent_win)

    def on_settings_close(self):
        if self.settings_window:
            self.ros_node.get_logger().info("Settings window closed.")
            self.settings_window.grab_release()
            self.settings_window.destroy()
            self.settings_window = None

    def open_localisation_window(self):
        if self.localisation_window and self.localisation_window.winfo_exists(): self.localisation_window.lift(); return
        self.localisation_window = LocalisationWindow(self, self.ros_node)

    def on_localisation_close(self):
        if self.localisation_window:
            self.ros_node.get_logger().info("Localisation window closed by child.")
            self.localisation_window = None

    def on_localise_canvas_click(self):
        self.ros_node.get_logger().info("Localise Canvas button clicked.")
        parent_win = self.settings_window if self.settings_window and self.settings_window.winfo_exists() else self
        success, msg = self.ros_node.launch_canvas_localisation_node()
        if not success:
            messagebox.showerror("Localisation Error", f"Failed to launch node: {msg}", parent=parent_win)
            return
        self.open_localisation_window()
        # MODIFICATION: Removed the success popup as requested
        # messagebox.showinfo("Localisation", f"{msg}\nLocalisation window opened.", parent=parent_win)
        self.ros_node.get_logger().info(f"{msg} Localisation window should be open.")


    def on_calibrate_robot_click(self):
        self.ros_node.get_logger().info("Calibrate Robot button clicked.")
        parent_win = self.settings_window if self.settings_window and self.settings_window.winfo_exists() else self
        success, msg = self.ros_node.launch_robot_calibration_launchfile()
        if success:
            messagebox.showinfo("Robot Calibration", msg, parent=parent_win)
        else:
             messagebox.showerror("Calibration Error", msg, parent=parent_win)

    def show_draw_confirmation_dialog(self):
        dialog = Toplevel(self)
        dialog.title("Confirm Drawing Operation")

        self.update_idletasks()
        parent_x = self.winfo_rootx()
        parent_y = self.winfo_rooty()
        parent_width = self.winfo_width()
        parent_height = self.winfo_height()

        dialog_width = 420
        dialog_height = 180

        x = parent_x + (parent_width - dialog_width) // 2
        y = parent_y + (parent_height - dialog_height) // 2
        dialog.geometry(f"{dialog_width}x{dialog_height}+{x}+{y}")

        dialog.resizable(False, False)
        dialog.transient(self)
        dialog.grab_set()

        dialog.config(bg=self.dark_grey)

        message = "Ensure people are clear of the robot and someone has their hand on the E-stop button of the UR3e."
        label = tk.Label(dialog, text=message, wraplength=dialog_width - 30, justify=tk.LEFT,
                         bg=self.dark_grey, fg=self.text_color, padx=15, pady=15, font=("Arial", 10))
        label.pack(pady=(10, 5), fill=tk.X, expand=True)

        button_frame = tk.Frame(dialog, bg=self.dark_grey)
        button_frame.pack(pady=(5, 15))

        result = {"confirmed": False}

        def on_start():
            result["confirmed"] = True
            dialog.destroy()

        def on_cancel():
            result["confirmed"] = False
            dialog.destroy()

        btn_cfg_dialog = {"bg": "#303030", "fg": self.text_color, "activebackground": "#454545",
                          "relief": FLAT, "borderwidth": 0, "highlightthickness": 0, "width": 15, "height":1, "font":("Arial",10)}

        start_button = tk.Button(button_frame, text="Start Drawing", command=on_start, **btn_cfg_dialog)
        start_button.pack(side=tk.LEFT, padx=10)

        cancel_button = tk.Button(button_frame, text="Cancel", command=on_cancel, **btn_cfg_dialog)
        cancel_button.pack(side=tk.LEFT, padx=10)

        dialog.protocol("WM_DELETE_WINDOW", on_cancel)
        dialog.wait_window()
        return result["confirmed"]

    def on_capture_button_click(self):
        if not self.running: return
        if self.display_mode == "live":
            self.ros_node.get_logger().info("Capture Image button clicked.")
            self.status_label.config(text="Sending processing request...")
            self.capture_button.config(state=DISABLED); self.draw_button.config(state=DISABLED)
            self.last_processed_frame_available = False
            if self.processed_label.winfo_exists(): self.processed_label.configure(image=None); hasattr(self.processed_label, 'image') and setattr(self.processed_label, 'image', None)
            if not self.ros_node.send_process_image_goal() and not self.ros_node.processing_active:
                self.status_label.config(text="Failed to send request."); messagebox.showerror("Action Error", "Could not send request.")
                self.capture_button.config(state=NORMAL)
        elif self.display_mode == "processed":
            self.ros_node.get_logger().info("Take Another button clicked.")
            self.display_mode = "live"
            self.capture_button.config(text="Capture Image", state=NORMAL); self.draw_button.config(state=DISABLED)
            self.last_processed_frame_available = False
            with self.ros_node.processed_frame_lock: self.ros_node.latest_processed_frame = None
            if self.processed_label.winfo_exists(): self.processed_label.configure(image=None); hasattr(self.processed_label, 'image') and setattr(self.processed_label, 'image', None)
            self.status_label.config(text="Ready for new capture.")

    def on_draw_button_click(self):
        if not self.running: return

        if not self.last_processed_frame_available:
            messagebox.showwarning("Publish Error", "No valid processed image available to publish.", parent=self)
            self.status_label.config(text="Draw failed: No image.")
            return

        if not self.show_draw_confirmation_dialog():
            self.status_label.config(text="Drawing cancelled by user.")
            self.ros_node.get_logger().info("Drawing operation cancelled by user.")
            return

        self.ros_node.get_logger().info("Draw button clicked and confirmed.")
        if not self.last_processed_frame_available:
             messagebox.showwarning("Publish Error", "No valid processed image to publish.", parent=self)
             self.status_label.config(text="Draw failed: No image.")
             return

        self.status_label.config(text="Publishing to /pixel_map...")
        self.draw_button.config(state=DISABLED)
        self.capture_button.config(state=DISABLED)
        if self.ros_node.publish_processed_image():
            self.status_label.config(text="Published to /pixel_map.")
            if self.display_mode == "processed": self.capture_button.config(state=NORMAL)
        else:
            self.status_label.config(text="Failed to publish."); messagebox.showerror("Publish Error", "Could not publish.", parent=self)
            if self.display_mode == "processed": self.capture_button.config(state=NORMAL)
            self.draw_button.config(state=NORMAL if self.last_processed_frame_available else DISABLED)


    def update_image_display(self):
        last_display_mode = None
        while self.running and rclpy.ok():
            if not self.winfo_exists(): break
            webcam, edge, processed = self.ros_node.get_latest_webcam_frame(), self.ros_node.get_latest_edge_frame(), self.ros_node.get_latest_processed_frame()
            is_proc, proc_avail = self.ros_node.processing_active, (processed is not None)

            if hasattr(self, 'error_status_label') and self.error_status_label.winfo_exists():
                err = self.ros_node.get_latest_error_message(); txt = f"ERR: {err}" if err else ""
                if self.error_status_label.cget("text") != txt: self.error_status_label.config(text=txt)

            if self.display_mode != last_display_mode:
                try:
                    self.ros_node.get_logger().debug(f"Switching display mode to {'Live' if self.display_mode=='live' else 'Processed'}")
                    frames_to_toggle=[self.processed_display_frame, self.top_frame]
                    visibility_map=[0,1] if self.display_mode=="live" else [1,0]

                    if frames_to_toggle[visibility_map[0]].winfo_exists() and frames_to_toggle[visibility_map[0]].winfo_ismapped():
                        frames_to_toggle[visibility_map[0]].pack_forget()
                    if frames_to_toggle[visibility_map[1]].winfo_exists() and not frames_to_toggle[visibility_map[1]].winfo_ismapped():
                        frames_to_toggle[visibility_map[1]].pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=5, pady=5)
                    last_display_mode = self.display_mode
                except tk.TclError as e:
                    self.ros_node.get_logger().warn(f"TclError during display mode switch: {e}"); last_display_mode=None
                except Exception as e_disp:
                    self.ros_node.get_logger().error(f"Error switching display mode: {e_disp}\n{traceback.format_exc()}")
                    last_display_mode = None


            try:
                if self.display_mode == "live":
                    for fr_data, lbl_widget in [(webcam,self.webcam_label),(edge,self.edge_label)]:
                        if lbl_widget.winfo_exists():
                            if fr_data is not None:
                                try:
                                    cvt_code=cv2.COLOR_BGR2RGB if fr_data.ndim==3 and fr_data.shape[2]==3 else cv2.COLOR_GRAY2RGB
                                    img_pil=PILImage.fromarray(cv2.cvtColor(fr_data,cvt_code))
                                    img_tk=ImageTk.PhotoImage(image=img_pil)
                                    lbl_widget.config(image=img_tk)
                                    lbl_widget.image=img_tk
                                except Exception as e_img:
                                    self.ros_node.get_logger().error(f"Error converting/displaying image for {lbl_widget}: {e_img}", throttle_duration_sec=5.0)

                elif self.processed_label.winfo_exists():
                    if proc_avail:
                        try:
                            img_pil=PILImage.fromarray(cv2.cvtColor(processed,cv2.COLOR_GRAY2RGB))
                            img_tk=ImageTk.PhotoImage(image=img_pil)
                            self.processed_label.config(image=img_tk)
                            self.processed_label.image=img_tk
                        except Exception as e_img_proc:
                             self.ros_node.get_logger().error(f"Error converting/displaying processed image: {e_img_proc}", throttle_duration_sec=5.0)
                    elif hasattr(self.processed_label,'image') and self.processed_label.image is not None:
                        self.processed_label.config(image=None)
                        self.processed_label.image = None

            except tk.TclError as e_img_update:
                self.ros_node.get_logger().warn(f"TclError during image update: {e_img_update}")
            except Exception as e_gen_img_update:
                 self.ros_node.get_logger().error(f"Generic error during image update: {e_gen_img_update}\n{traceback.format_exc()}")


            if hasattr(self, 'capture_button') and self.capture_button.winfo_exists() and \
               hasattr(self, 'draw_button') and self.draw_button.winfo_exists() and \
               hasattr(self, 'status_label') and self.status_label.winfo_exists():

                if not is_proc and self.ros_node._send_goal_future and self.display_mode=="live" and self.capture_button.cget('text')=="Capture Image":
                    if proc_avail:
                        self.ros_node.get_logger().info("Processing complete, image available. Switching to processed view.")
                        self.display_mode="processed"
                        self.last_processed_frame_available=True
                        self.capture_button.config(text="Take Another",state=NORMAL)
                        self.draw_button.config(state=NORMAL)
                        self.status_label.config(text="Processing complete. Image ready.")
                    elif self.ros_node._get_result_future and self.ros_node._get_result_future.done():
                        self.ros_node.get_logger().warn("Processing finished, but no valid image returned.")
                        self.status_label.config(text="Processing failed or no image returned.")
                        self.capture_button.config(state=NORMAL)
                    self.ros_node._send_goal_future, self.ros_node._get_result_future = None, None

                current_status_text = self.status_label.cget("text")
                if is_proc:
                    if not current_status_text.startswith("Processing request..."): self.status_label.config(text="Processing request...")
                    if self.capture_button.cget('state') != DISABLED: self.capture_button.config(state=DISABLED)
                    if self.draw_button.cget('state') != DISABLED: self.draw_button.config(state=DISABLED)
                else:
                    if self.display_mode == "live":
                        status_parts = []
                        if webcam is None: status_parts.append("Waiting for Webcam Feed")
                        if edge is None: status_parts.append("Waiting for Edge Feed")
                        live_status = ", ".join(status_parts) if status_parts else "Ready."

                        if current_status_text != live_status and not any(kw in current_status_text for kw in ["Published", "Failed", "complete", "Drawing"]):
                            self.status_label.config(text=live_status)

                        if self.capture_button.cget('state') == DISABLED and self.capture_button.cget('text') == "Capture Image":
                             self.capture_button.config(state=NORMAL)
                        if self.draw_button.cget('state') == NORMAL:
                             self.draw_button.config(state=DISABLED)

                    elif self.display_mode == "processed":
                        processed_status = "Image ready for drawing." if proc_avail else "No processed image available."
                        if not any(kw in current_status_text for kw in ["Published", "Drawing", "cancelled"]):
                             if current_status_text != processed_status and not current_status_text.startswith("Processing complete"):
                                self.status_label.config(text=processed_status)

                        if self.capture_button.cget('state') == DISABLED and self.capture_button.cget('text') == "Take Another":
                            self.capture_button.config(state=NORMAL)

                        new_draw_button_state = NORMAL if proc_avail else DISABLED
                        if self.draw_button.cget('state') != new_draw_button_state:
                            self.draw_button.config(state=new_draw_button_state)

            self.last_processed_frame_available=proc_avail
            time.sleep(0.05)
        self.ros_node.get_logger().info("GUI update loop finished.")


    def on_closing(self):
        if messagebox.askokcancel("Quit","Do you want to quit?"):
            self.running=False
            if hasattr(self,'update_thread') and self.update_thread.is_alive():
                self.ros_node.get_logger().info("Joining GUI update thread...")
                self.update_thread.join(timeout=1.0)
                if self.update_thread.is_alive():
                    self.ros_node.get_logger().warn("GUI update thread did not join in time.")

            for attr_n in ['localisation_window','settings_window']:
                win=getattr(self,attr_n,None)
                if win and win.winfo_exists():
                    try:
                        win.destroy()
                    except tk.TclError:
                        self.ros_node.get_logger().warn(f"TclError destroying {attr_n}. May already be gone.")
                    setattr(self,attr_n,None)

            self.ros_node.get_logger().info("Destroying main Tk window...")
            try:
                self.destroy()
                self.ros_node.get_logger().info("Main Tk window destroyed.")
            except tk.TclError as e:
                 self.ros_node.get_logger().warn(f"TclError destroying main window, may already be gone: {e}")

        else:
            self.ros_node.get_logger().info("Quit cancelled.")

def main(args=None):
    if Img is None or GoalStatus is None:
        print("CRITICAL ERROR: ROS 2 Action or Message types (Img, GoalStatus) not imported. Exiting.")
        sys.exit(1)

    print("Initializing rclpy...")
    rclpy.init(args=args)

    gui_node, app_gui, ros_spin_thread = None, None, None
    exit_code = 0

    try:
        print("Creating GuiNode instance...")
        gui_node = GuiNode()
        print("GuiNode instance created.")

        def spin_ros_node_thread(node_instance):
            node_name = node_instance.get_name() if node_instance else "ROS_Node"
            print(f"ROS spin thread for {node_name} started.")
            try:
                if node_instance:
                    rclpy.spin(node_instance)
                if rclpy.ok() and node_instance and node_instance.context.ok():
                    node_instance.get_logger().info(f"ROS spin for {node_name} completed cleanly.")
                else:
                    print(f"ROS spin for {node_name} finished; rclpy.ok: {rclpy.ok()}, node_instance.context.ok: {node_instance.context.ok() if node_instance else 'N/A'}")
            except Exception as e_spin:
                print(f"Exception in ROS spin thread for {node_name}: {e_spin}\n{traceback.format_exc()}")
            finally:
                print(f"ROS spin thread for {node_name} is exiting.")

        ros_spin_thread = threading.Thread(target=spin_ros_node_thread, args=(gui_node,), daemon=True)
        ros_spin_thread.start()
        print("ROS spin thread launched.")

        print("Creating ActionViewerApp instance...")
        app_gui = ActionViewerApp(gui_node)
        print("ActionViewerApp instance created. Starting Tk mainloop...")
        app_gui.mainloop()
        print("Tk mainloop finished.")

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received. Shutting down...")
        if app_gui: app_gui.running = False
    except Exception as e_main:
        print(f"Unhandled exception in main: {e_main}\n{traceback.format_exc()}")
        exit_code = 1
        if app_gui: app_gui.running = False
    finally:
        print("Starting cleanup sequence...")

        if app_gui and hasattr(app_gui, 'running'):
            app_gui.running = False

        if app_gui and app_gui.winfo_exists():
            print("Attempting to close Tkinter application window...")
            try:
                app_gui.destroy()
                print("Tkinter application window destroy called.")
            except tk.TclError:
                print("Tkinter window was already destroyed or unavailable.")

        if rclpy.ok():
            if gui_node:
                print("Terminating external processes managed by GuiNode...")
                processes_to_terminate = [
                    ("Canvas Localisation", "canvas_localisation_process"),
                    ("Robot Calibration", "robot_calibration_process"),
                    ("Dale Integration", "dale_integration_process"),
                    ("UR System", "ur_system_process")
                ]
                for name, attr_name in processes_to_terminate:
                    process_instance = getattr(gui_node, attr_name, None)
                    if process_instance and process_instance.poll() is None:
                        print(f"Terminating {name} process (PID: {process_instance.pid})...")
                        process_instance.terminate()
                        try:
                            process_instance.wait(timeout=1.5)
                            print(f"{name} process terminated.")
                        except subprocess.TimeoutExpired:
                            print(f"{name} process did not terminate gracefully, killing...")
                            process_instance.kill()
                            process_instance.wait()
                            print(f"{name} process killed.")
                        except Exception as e_proc_term:
                             print(f"Error during termination of {name}: {e_proc_term}")


                if gui_node.context.ok():
                    print("Destroying GuiNode...")
                    gui_node.destroy_node()
                    print("GuiNode destroyed.")
                else:
                    print("GuiNode context already invalid, node likely already destroyed.")

            print("Shutting down rclpy...")
            rclpy.shutdown()
            print("rclpy shutdown complete.")
        else:
            print("rclpy context was already invalid before final shutdown call.")

        if ros_spin_thread and ros_spin_thread.is_alive():
            print("Waiting for ROS spin thread to join...")
            ros_spin_thread.join(timeout=2.0)
            if ros_spin_thread.is_alive():
                print("ROS spin thread did not join in time.")
            else:
                print("ROS spin thread joined.")

        print(f"Cleanup sequence finished. Exiting with code {exit_code}.")
        sys.exit(exit_code)

if __name__ == '__main__':
    main()