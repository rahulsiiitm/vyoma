from setuptools import setup

package_name = 'semantic_vision_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name], # FIX: Hard-coded package name
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rahul',
    maintainer_email='rahul@todo.todo',
    description='YOLO Vision Node',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = semantic_vision_pkg.vision_node:run_vision_node',
        ],
    },
)
