from setuptools import find_packages, setup

package_name = 'ros_arduino_python'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Patrick Goebel',
    maintainer_email='patrick@pirobot.org',
    description='ROS Arduino Python.',
    license='BSD',
    url='http://ros.org/wiki/ros_arduino_python',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_arduino_python = ros_arduino_python.ros_arduino_python:main'
        ],
    },
)
