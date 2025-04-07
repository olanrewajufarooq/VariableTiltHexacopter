from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'hexacopter_description'

setup(
    name=package_name,
    version='0.0.1',
    # packages=[package_name],
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.sdf')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share/ament_index/resource_index/packages'), ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Farooq Olanrewaju',
    maintainer_email='olanrewajufarooq@yahoo.com',
    description='Hexacopter variable tilt description and simulation launch',
    license='MIT',
    entry_points={
        'console_scripts': [
            'simple_control_node = hexacopter_description.simple_control_node:main',
        ],
    },
)
