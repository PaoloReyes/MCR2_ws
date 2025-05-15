from setuptools import find_packages, setup

package_name = 'puzzlebot_sim2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Paolo Reyes',
    maintainer_email='paolo.alfonso.reyes@gmail.com',
    description='TODO: Package description',
    license='MIT',
    entry_points={
        'console_scripts': [
            'puzzlebot_sim2 = puzzlebot_sim2.puzzlebot_sim2:main'
        ],
    },
)
