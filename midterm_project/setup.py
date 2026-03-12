from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'midterm_project'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),
        ('share/' + package_name + '/config',
            glob('config/*.yaml')),
        ('share/' + package_name + '/worlds',
            glob('worlds/*.sdf')),
        ('share/' + package_name + '/models/planetary_terrain',
            glob('models/planetary_terrain/*.*')),
        ('share/' + package_name + '/models/planetary_terrain/meshes',
            glob('models/planetary_terrain/meshes/*.*')),
        ('share/' + package_name + '/models/planetary_terrain/materials/textures',
            glob('models/planetary_terrain/materials/textures/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jatin Satyam',
    maintainer_email='jsatyam@asu.edu',
    description='Uncertainty-Aware 3DGS Terrain Mapping and Precision Landing',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = midterm_project.control_node:main',
            'perception_node = midterm_project.perception_node:main',
            'mapping_node = midterm_project.mapping_node:main',
            'hazard_node = midterm_project.hazard_node:main',
            'planning_node = midterm_project.planning_node:main',
        ],
    },
    python_requires='>=3.8',
)
