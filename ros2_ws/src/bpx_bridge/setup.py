from setuptools import find_packages, setup

package_name = 'bpx_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/bpx_bridge.yaml']),
        ('share/' + package_name + '/launch', ['launch/bpx_bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dean',
    maintainer_email='dean@localhost',
    description='ROS 2 bridge for the MirrorMe Black Panther X (BPX) robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bpx_bridge_node = bpx_bridge.bpx_bridge_node:main',
        ],
    },
)
