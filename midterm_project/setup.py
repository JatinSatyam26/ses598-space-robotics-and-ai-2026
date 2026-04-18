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
        (os.path.join('share', package_name, 'models', 'mars_rover'),
         glob('models/mars_rover/*')),
        (os.path.join('share', package_name, 'models', 'planetary_terrain'),
         [f for f in glob('models/planetary_terrain/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'models', 'planetary_terrain', 'meshes'),
         glob('models/planetary_terrain/meshes/*')),
        (os.path.join('share', package_name, 'models', 'flying_drone'),
         glob('models/flying_drone/*')),
        (os.path.join('share', package_name, 'models', 'habitat_dome'),
         glob('models/habitat_dome/*')),
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
            'rover_navigator = midterm_project.rover_navigator:main',
            'demo_image_saver = midterm_project.demo_image_saver:main',
            'flight_control_node = midterm_project.flight_control_node:main',
            'rover_driver = midterm_project.rover_driver:main',
            'smart_flight_node = midterm_project.smart_flight_node:main',
            'smart_rover_node = midterm_project.smart_rover_node:main',
        ],
    },
)
