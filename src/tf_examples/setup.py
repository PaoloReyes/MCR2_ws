import os

from glob import glob
from setuptools import find_packages, setup

package_name = 'tf_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='paolo',
    maintainer_email='paolo.alfonso.reyes@gmail.com',
    description='TF examples Week 1',
    license='MIT',
    entry_points={
        'console_scripts': ['static_tf = tf_examples.static_tf:main',
                            'dynamic_tf = tf_examples.dynamic_tf:main',
                            'tf_listener = tf_examples.tf_listener:main',
        ],
    },
)
