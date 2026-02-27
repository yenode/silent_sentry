from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sblp_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.xml') + glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aditya Pachauri',
    maintainer_email='aditya@example.com',
    description='Scenario-Based Local Planner for Thar Desert terrain',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sblp_node = sblp_planner.sblp_node:main',
        ],
    },
)
