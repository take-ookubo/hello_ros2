from glob import glob

from setuptools import setup

package_name = 'agri_bench'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='take-ookubo',
    maintainer_email='2122405+take-ookubo@users.noreply.github.com',
    description='Load cell logger for the stem cutting and grip force bench tests',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'force_logger = agri_bench.force_logger:main',
            'trial = agri_bench.trial:main',
            'calibrate = agri_bench.calibrate:main',
            'stage = agri_bench.stage_cli:main',
        ],
    },
)
