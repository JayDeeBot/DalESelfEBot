import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import sys
import img_prc.image_processing as image_processing
import threading
import tkinter as tk
import subprocess
import re

def get_camera_property_ranges(device='/dev/video1'):
    """Return min/max/default for common controls from v4l2-ctl."""
    print(f"[DEBUG] Attempting to fetch camera properties from: {device}")
    try:
        result = subprocess.run(['v4l2-ctl', '-d', device, '--list-ctrls'],
                                capture_output=True, text=True, check=True)
        output = result.stdout
        print("[DEBUG] v4l2-ctl output successfully fetched.")

        pattern = re.compile(
        r'^\s*(\w+)\s+0x[0-9a-f]+\s+\(.*?\)\s*:\s*min=(-?\d+)\s+max=(-?\d+)\s+step=\d+\s+default=(-?\d+)\s+value=(-?\d+)',
        re.IGNORECASE)

        properties = {}
        for line in output.splitlines():
            match = pattern.match(line)
            if match:
                name, min_val, max_val, default_val, current_val = match.groups()
                properties[name] = {
                    'min': int(min_val),
                    'max': int(max_val),
                    'default': int(default_val),
                    'value': int(current_val)
                }
                print(f"[DEBUG] Found control: {name} | min={min_val}, max={max_val}, default={default_val}, current={current_val}")
        return properties
    except Exception as e:
        print(f"[ERROR] Failed to get camera property ranges: {e}")
        return {}


class WebcamPublisher(Node):
    def __init__(self, camera_index):
        self.camera_index = camera_index
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(Image, 'webcam_image', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv2.VideoCapture(camera_index)
        self.bridge = CvBridge()

        # Default camera settings
        self.brightness = self.cap.get(cv2.CAP_PROP_BRIGHTNESS)
        self.contrast = self.cap.get(cv2.CAP_PROP_CONTRAST)
        self.exposure = self.cap.get(cv2.CAP_PROP_EXPOSURE)
        self.gain = self.cap.get(cv2.CAP_PROP_GAIN)


                # Set exposure to manual mode (may vary per camera)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # Try 0.25 or 1 if this doesn't work
        self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure)
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.brightness)
        self.cap.set(cv2.CAP_PROP_CONTRAST, self.contrast)


    def timer_callback(self):
        # Apply current settings
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.brightness)
        self.cap.set(cv2.CAP_PROP_CONTRAST, self.contrast)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure)
        self.cap.set(cv2.CAP_PROP_GAIN, self.gain)


        ret, frame = self.cap.read()
        if ret:
            sizedFrame = cv2.resize(frame, (image_processing.window_width, image_processing.window_height))
            msg = self.bridge.cv2_to_imgmsg(sizedFrame, encoding='bgr8')
            self.publisher_.publish(msg)

    # Methods to update parameters from GUI
    def update_brightness(self, val):
        self.brightness = float(val)

    def update_contrast(self, val):
        self.contrast = float(val)

    def update_exposure(self, val):
        self.exposure = float(val)

    def update_gain(self, val):
        self.gain = float(val)


def start_gui(node):
    import tkinter as tk
    from tkinter import ttk

    # Try to detect the correct /dev/videoX path
    device_path = f'/dev/video{node.camera_index}'
    print(f"[DEBUG] Using video device: {device_path}")
    ranges = get_camera_property_ranges(device_path)
    print("[DEBUG] Available v4l2 controls:", ranges.keys())

    def make_slider(row, label_text, control_key, command, initial_value):
        # Fall back to sane defaults if not found
        props = ranges.get(control_key.lower(), {'min': 0, 'max': 100})
        min_val = props['min']
        max_val = props['max']

        label = tk.Label(root, text=label_text, font=("Helvetica", 14))
        label.grid(row=row, column=0, padx=20, pady=10, sticky="w")

        value_label = tk.Label(root, text=f"{initial_value:.2f}", font=("Helvetica", 14))
        value_label.grid(row=row, column=2, padx=10, sticky="w")

        slider = ttk.Scale(
            root, from_=min_val, to=max_val, orient="horizontal", length=600,
            command=lambda val: (command(val), value_label.config(text=f"{float(val):.2f}"))
        )
        slider.set(initial_value)
        slider.grid(row=row, column=1, pady=10, padx=20, sticky="w")

    root = tk.Tk()
    root.title("Camera Control Panel")
    root.geometry("900x600")
    root.configure(bg="#f0f0f0")

    make_slider(row=0, label_text="Brightness", control_key="brightness",
                command=node.update_brightness, initial_value=node.brightness)

    make_slider(row=1, label_text="Contrast", control_key="contrast",
                command=node.update_contrast, initial_value=node.contrast)

    make_slider(row=2, label_text="Exposure", control_key="exposure_time_absolute",
                command=node.update_exposure, initial_value=node.exposure)
    
    make_slider(row=3, label_text="Gain", control_key="gain",
                command=node.update_gain, initial_value=node.gain)
    
        
    make_slider(row=4, label_text="Gamma", control_key="gamma",
                command=node.update_gain, initial_value=node.gain)

    root.mainloop()




def main(args=None):
    rclpy.init(args=args)
    camera_index = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    node = WebcamPublisher(camera_index)

    # Start GUI in a separate thread
    gui_thread = threading.Thread(target=start_gui, args=(node,), daemon=True)
    gui_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    # make_slider(row=0, label_text="Brightness", from_=-60.0, to=60.0, resolution=1.0,
    #             command=node.update_brightness, initial_value=node.brightness)

    # make_slider(row=1, label_text="Contrast", from_=1, to=60, resolution=0.01,
    #             command=node.update_contrast, initial_value=node.contrast)

    # make_slider(row=2, label_text="Exposure", from_=1, to=1000, resolution=1,
    #             command=node.update_exposure, initial_value=node.exposure)