from setuptools import setup

package_name = 'bag_recorder'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/record.launch.py', 'launch/record_with_ui.launch.py']),
        ('share/' + package_name + '/config', ['config/bag_recorder.yaml']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    author='Your Name',
    author_email='you@example.com',
    description='Record rosbag2 via YAML config (launch/CLI wrapper + simple UI).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'recorder = bag_recorder.recorder:main',
            'recorder_ui = bag_recorder.recorder_ui:main',   # ★ 新增
        ],
    },
)
