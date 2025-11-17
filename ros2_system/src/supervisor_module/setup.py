from setuptools import setup
import os
from glob import glob

package_name = 'supervisor_module'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
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
            'system_dashboard = supervisor_module.system_dashboard:main',
            'brain_dashboard = supervisor_module.brain_dashboard:main',
        ],
    },
)
