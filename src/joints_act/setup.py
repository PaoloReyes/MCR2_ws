import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'joints_act'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Paolo Reyes',
    maintainer_email='paolo.alfonso.reyes@gmail.com',
    description='URDF examples week 1',
    license='MIT',
    entry_points={
        'console_scripts': [
            'joint_pub = joints_act.joint_pub:main'
        ],
    },
)
