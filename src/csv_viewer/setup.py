from setuptools import setup
import os
from glob import glob

package_name = 'csv_viewer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Visualize 3D points from CSV/TXT in RViz',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'csv_rviz_viewer = csv_viewer.csv_rviz_viewer:main',
        ],
    },
)
