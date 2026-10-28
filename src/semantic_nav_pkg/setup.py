from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'semantic_nav_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='drone_user',
    maintainer_email='drone_user@todo.todo',
    description='Semantic Navigation Autopilot',
    license='All Rights Reserved',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_sector_node = semantic_nav_pkg.depth_sector_node:main',
            'global_planner_node = semantic_nav_pkg.global_planner_node:main',
            'local_target_node = semantic_nav_pkg.local_target_node:main',
            'mission_manager_node = semantic_nav_pkg.mission_manager_node:main',
            'px4_adapter_node = semantic_nav_pkg.px4_adapter_node:main',
            'safety_supervisor_node = semantic_nav_pkg.safety_supervisor_node:main'
        ],
    },
)
