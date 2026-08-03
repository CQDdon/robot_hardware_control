from setuptools import setup

package_name = 'vehicle_keyboard_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vehicle_keyboard_teleop.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ddon',
    maintainer_email='caoquangddon5@gmail.com',
    description='Keyboard teleoperation for vehicle_msgs.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vehicle_keyboard_teleop = vehicle_keyboard_teleop.keyboard_teleop:main',
        ],
    },
)
