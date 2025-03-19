from setuptools import find_packages, setup

package_name = 'hexacopter_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Farooq Olanrewaju',
    maintainer_email='olanrewajufarooq@yahoo.com',
    description="This package provides a comprehensive description of a hexacopter, including URDF models, meshes, and configuration files necessary for simulation and visualization. It supports integration with both RViz and Gazebo environments.",
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
        ],
    },
)
