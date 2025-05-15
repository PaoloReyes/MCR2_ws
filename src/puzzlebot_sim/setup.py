import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'puzzlebot_sim'

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
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.stl'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='paolo',
    maintainer_email='paolo.alfonso.reyes@gmail.com',
    description='Puzzlebot simulations package',
    license='MIT',
    entry_points={
        'console_scripts': ['puzzlebot_joint_publisher = puzzlebot_sim.puzzlebot_joint_publisher:main',
        ],
    },
)
