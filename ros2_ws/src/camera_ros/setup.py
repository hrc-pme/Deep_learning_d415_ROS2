from setuptools import setup
from glob import glob
import os

package_name = 'camera_ros'

data_files = [
    ('share/ament_index/resource_index/packages',
     [os.path.join('resource', package_name)]),
    ('share/' + package_name + '/launch', glob('launch/*launch.py')),
    ('share/' + package_name + '/config', glob('config/*')),
    ('share/' + package_name, ['package.xml']),
]

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # 目前只有 __init__.py
    data_files=data_files,
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='hrc',
    maintainer_email='hrc@example.com',
    description='RealSense launch (ROS 2 Humble)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [

        ],
    },
)
