from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'state_estimation'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join(
                'share',
                package_name,
                'config'
            ),
            glob('config/*.yaml')
        ),
    ],

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    maintainer='tommy',
    maintainer_email='todo@todo.com',
    description='State estimation package',

    license='TODO',

    entry_points={
        'console_scripts': [
            'imu_subscriber = state_estimation.imu_subscriber:main',
            'imu_covariance_node = state_estimation.imu_covariance_node:main',
        ],  
    },
)