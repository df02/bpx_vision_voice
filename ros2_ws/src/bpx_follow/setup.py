from setuptools import find_packages, setup

package_name = 'bpx_follow'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/bpx_follow.yaml']),
        ('share/' + package_name + '/launch', ['launch/bpx_follow.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dean Ferraro',
    maintainer_email='df02@users.noreply.github.com',
    description='Person-following mode for the BPX quadruped.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bpx_follow_node = bpx_follow.bpx_follow_node:main',
        ],
    },
)
