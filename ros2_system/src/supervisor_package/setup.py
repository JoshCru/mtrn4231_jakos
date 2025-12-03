from setuptools import setup
import os
from glob import glob

package_name = 'supervisor_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@todo.todo',
    description='Supervisor module with system controller and dashboard UI',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sorting_brain_node = supervisor_package.sorting_brain_node:main',
            'system_dashboard = supervisor_package.system_dashboard:main',
        ],
    },
)
