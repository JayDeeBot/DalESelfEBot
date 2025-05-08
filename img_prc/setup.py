from setuptools import find_packages, setup

import os # Keep os if needed for data_files path construction
from glob import glob

package_name = 'img_prc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Make sure the path construction is correct for your setup
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Add other data files if needed (e.g., launch files)
        #(os.path.join('share', package_name, 'launch'),
        # ),
    ],
    # setup_requires=['setuptools'], # Often not needed with modern pip/setuptools
    install_requires=[
        'setuptools', # Keep setuptools requirement
        'rclpy',      # ROS 2 Python library
        'sensor_msgs',# For Image messages
        'numpy<2.0',  # Keep numpy, specify compatible version if needed
        'Pillow',     # Image library used by rembg/cv2 interactions
        'onnxruntime',# Dependency for rembg
        'rembg'       # Background removal library (will be installed via pip in venv)
        # Add other direct Python dependencies of your nodes here
    ],
    # Removed cmdclass argument:
    # cmdclass={
    #     'install': PostInstallCommand,
    # },
    zip_safe=True,
    maintainer='dingus', # TODO: Update maintainer info
    maintainer_email='dingus@todo.todo', # TODO: Update email
    description='TODO: Package description', # TODO: Update description
    license='TODO: License declaration', # TODO: Update license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'image_processor = scripts.image_processor:main', #outdated, dont use this
            'image_sub = scripts.image_sub:main',
            'image_client = scripts.image_client:main',
            'webcam_pub = scripts.webcam_publisher:main',
           'edge_pub = scripts.edge_publisher:main',
            'bg_rmv_server = scripts.bg_rmv_server:main',],
    
    },
)