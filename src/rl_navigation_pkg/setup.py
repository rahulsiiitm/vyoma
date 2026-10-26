from setuptools import setup
import os
from glob import glob

package_name = 'rl_navigation_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rahul',
    maintainer_email='rahul@todo.todo',
    description='Reinforcement Learning based navigation package for Vyoma',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train_node = rl_navigation_pkg.train_node:main',
            'policy_node = rl_navigation_pkg.policy_node:main'
        ],
    },
)
