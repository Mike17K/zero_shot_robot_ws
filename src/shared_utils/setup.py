from setuptools import find_packages, setup

package_name = 'shared_utils'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaipis',
    maintainer_email='kaipismike1@gmail.com',
    description='Reusable ROS 2 helpers and the cuMotion / MoveIt planning clients',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cumotion_cli = shared_utils.tools.cumotion_cli:main',
        ],
    },
)
