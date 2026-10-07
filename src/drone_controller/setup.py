from setuptools import setup
import os
from glob import glob


package_name = 'drone_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Step 2: minimal node skeleton, no control logic yet.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hello_node = drone_controller.hello_node:main',
            'controller = drone_controller.controller:main',
            'plain_sph = drone_controller.plain_sph:main',
            'swarm_sph = drone_controller.swarm_sph:main',
            'particle_sim = drone_controller.particle_sim:main',
        ],
    },
)