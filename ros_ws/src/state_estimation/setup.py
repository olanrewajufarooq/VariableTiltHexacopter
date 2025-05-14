from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'state_estimation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'rclpy', 'numpy', 'opencv-python', 'open3d', 'gtsam', 'open3d_ros2_helper'],
    zip_safe=True,
    maintainer='Farooq Olanrewaju',
    maintainer_email='olanrewajufarooq@yahoo.com',
    description='Visual-Inertia State Estimation using GTSAM and Open3D',
    license='MIT',
    entry_points={
        'console_scripts': [
            'state_estimator_node = state_estimation.state_estimator_node:main',
            'vo_node = state_estimation.vo_node:main',
            'vio_node = state_estimation.vio_node:main',
        ],
    },
)
