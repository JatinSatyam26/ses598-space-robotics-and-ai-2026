from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'midterm_project'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jatin Satyam',
    maintainer_email='jsatyam@asu.edu',
    description='Aerial-Ground Cooperative Autonomy',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = midterm_project.control_node:main',
            'perception_node = midterm_project.perception_node:main',
            'mapping_node = midterm_project.mapping_node:main',
            'hazard_node = midterm_project.hazard_node:main',
            'planning_node = midterm_project.planning_node:main',
            'image_saver = midterm_project.image_saver:main',
        ],
    },
)
