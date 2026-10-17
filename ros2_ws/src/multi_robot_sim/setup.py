from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'multi_robot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            os.path.join('share', package_name),
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config', 'semantic'),
            glob('config/semantic/*'),
        ),
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*'),
        ),
        (
            os.path.join('share', package_name, 'models', 'turtlebot3_burger'),
            glob('models/turtlebot3_burger/*'),
        ),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='Akshit Sharma',
    maintainer_email='akshitsharma20022@gmail.com',
    description='Multi-robot Gazebo simulation for the LTU Robotics and AI assignment',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'waypoint_planner = multi_robot_sim.waypoint_planner:main',
        'waypoint_navigator = multi_robot_sim.waypoint_navigator:main',
        'mission_executor = multi_robot_sim.mission_executor:main',
        'semantic_planner = multi_robot_sim.semantic_planner:main',
        'llm_semantic_planner = multi_robot_sim.llm_semantic_planner:main',
        'semantic_events = multi_robot_sim.semantic_events:main',
    ]},
)
