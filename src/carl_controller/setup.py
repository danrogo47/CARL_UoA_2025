from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'carl_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danielrogers',
    maintainer_email='a1821615@adelaide.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["controller = carl_controller.ps4_controller_node:main",
                            "controller_state_monitor = carl_controller.controller_state_monitor_node:main",
                            "drive = carl_controller.wheg_node:main",
                            "joint = carl_controller.joint_node:main",
                            "test = carl_controller.test_control_node:main"
        ],
    },
)
