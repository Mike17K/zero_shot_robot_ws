import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'navigator_cli'

setup(
    name=package_name,
    version='3.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.json') + glob('config/*.txt')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaipis',
    maintainer_email='kaipismike1@gmail.com',
    description='Interactive pose-graph navigator for the group_a GP70L',
    license='BSD-2-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator_cli = navigator_cli.main:main',
            'visualize_pose_graph = navigator_cli.visualize_pose_graph:main',
            'navigator_server = navigator_cli.navigator_server:main',
        ],
    },
)
