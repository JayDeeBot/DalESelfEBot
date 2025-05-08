import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import sys
import img_prc.image_processing as image_processing
import threading
import tkinter as tk

class WebcamPublisher(Node):
    def __init__(self, camera_index):
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

    def make_slider(row, label_text, from_, to, resolution, command, initial_value):
        label = tk.Label(root, text=label_text, font=("Helvetica", 14))
        label.grid(row=row, column=0, padx=20, pady=10, sticky="w")

        value_label = tk.Label(root, text=f"{initial_value:.2f}", font=("Helvetica", 14))
        value_label.grid(row=row, column=2, padx=10, sticky="w")

        slider = ttk.Scale(
            root, from_=from_, to=to, orient="horizontal", length=600,
            command=lambda val: (command(val), value_label.config(text=f"{float(val):.2f}"))
        )
        slider.set(initial_value)
        slider.grid(row=row, column=1, pady=10, padx=20, sticky="w")

    root = tk.Tk()
    root.title("Camera Control Panel")
    root.geometry("900x600")  # Adjusted for good visibility on 1080p
    root.configure(bg="#f0f0f0")

    make_slider(row=0, label_text="Brightness", from_=-60.0, to=60.0, resolution=0.5,
                command=node.update_brightness, initial_value=node.brightness)

    make_slider(row=1, label_text="Contrast", from_=0, to=60, resolution=0.5,
                command=node.update_contrast, initial_value=node.contrast)

    make_slider(row=2, label_text="Exposure", from_=1, to=1000, resolution=0.5,
                command=node.update_exposure, initial_value=node.exposure)
    
    make_slider(row=3, label_text="Gain", from_=0, to=100, resolution=0.5,
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