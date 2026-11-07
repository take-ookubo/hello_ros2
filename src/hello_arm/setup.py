from glob import glob

from setuptools import setup

package_name = 'hello_arm'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='take-ookubo',
    maintainer_email='2122405+take-ookubo@users.noreply.github.com',
    description='Minimal Gazebo example: move a three-joint arm from ROS 2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'arm_mover = hello_arm.arm_mover:main',
        ],
    },
)
