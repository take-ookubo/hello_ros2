from setuptools import setup

package_name = 'agri_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='take-ookubo',
    maintainer_email='2122405+take-ookubo@users.noreply.github.com',
    description='Fruit detection from an RGB-D camera for the agri_arm harvesting demo',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fruit_detector = agri_perception.fruit_detector:main',
        ],
    },
)
