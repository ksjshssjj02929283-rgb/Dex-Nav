from setuptools import setup

package_name = 'robot_status_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/system_monitor.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Robot status monitoring node that checks the health of system nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'system_monitor = robot_status_monitor.system_monitor:main',
        ],
    },
)
