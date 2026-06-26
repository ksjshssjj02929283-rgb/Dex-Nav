from setuptools import find_packages, setup

package_name = 'general_joint_converter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dex',
    maintainer_email='dex@dexforce.com',
    description='General joint angle converter for dexforce robot system',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'general_joint_converter = general_joint_converter.general_joint_converter:main',
        ],
    },
)
