from setuptools import find_packages, setup

package_name = 'imu_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='capstoneb',
    maintainer_email='capstoneb@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = imu_pub.publisher_member_function:main',
                'listener = imu_pub.subscriber_member_function:main',
        ],
},
)
