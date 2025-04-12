from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'state_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='Farooq Olanrewaju',
    maintainer_email='olanrewajufarooq@yahoo.com',
    description='TODO: Package description',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ekf_node = state_estimation.ekf_node:main',
            'ikf_node = state_estimation.ikf_node:main',
        ],
    },
)
