from setuptools import setup

package_name = 'joystick_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'joystick_driver.joystick_node',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/joystick.launch.py']),
    ],
    install_requires=['setuptools', 'inputs'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Joystick driver for ROS2 using inputs library',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_node = joystick_driver.joystick_node:main',
        ],
    },
)

