import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'create3_msg_transfer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "launch"), glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harsh V Surana',
    maintainer_email='harshsurana1507@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["battery_state_subscriber = create3_msg_transfer.battery_state_subscriber:main",
                            "dock_status_subscriber = create3_msg_transfer.dock_status_subscriber:main",
                            "cliff_intensity_subscriber = create3_msg_transfer.cliff_intensity_subscriber:main",
                            "ir_intensity_subscriber = create3_msg_transfer.ir_intensity_subscriber:main",
                            "kidnap_status_subscriber = create3_msg_transfer.kidnap_status_subscriber:main",
                            "ir_opcode_subscriber = create3_msg_transfer.ir_opcode_subscriber:main",
                            "imu_subscriber = create3_msg_transfer.imu_subscriber:main",
                            "mouse_subscriber = create3_msg_transfer.mouse_subscriber:main",
                            "odom_subscriber = create3_msg_transfer.odom_subscriber:main",
                            "slip_status_subscriber = create3_msg_transfer.slip_status_subscriber:main",
                            "stop_status_subscriber = create3_msg_transfer.stop_status_subscriber:main",
                            "tf_subscriber = create3_msg_transfer.tf_subscriber:main",
                            "wheel_status_subscriber = create3_msg_transfer.wheel_status_subscriber:main",
                            "wheel_ticks_subscriber = create3_msg_transfer.wheel_ticks_subscriber:main",
                            "wheel_vels_subscriber = create3_msg_transfer.wheel_vels_subscriber:main"









                            
        ],
    },
)
