from setuptools import find_packages, setup

package_name = 'bag_recorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/bag_recorder']),
        ('share/bag_recorder', ['package.xml']),
        ('share/bag_recorder/launch', ['bag_recorder/launch/recorder_with_ui.launch.py']),
        ('share/bag_recorder/config', ['bag_recorder/config/bag_recorder.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hrc',
    maintainer_email='hrc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recorder_node = bag_recorder.recorder_node:main',
            'recorder_ui   = bag_recorder.recorder_ui:main',
        ],
    },
)
