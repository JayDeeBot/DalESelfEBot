from setuptools import setup
import os
from glob import glob

package_name = 'tool_path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amrithdavid',
    maintainer_email='amrith.david123@gmail.com',
    description='Tool Path Planning for Selfie-Drawing Robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tool_path_planner_node = tool_path_planning.tool_path_planner_node:main',
        ],
    },
)
