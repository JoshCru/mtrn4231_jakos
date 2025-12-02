from setuptools import setup
import os
from glob import glob

package_name = 'perception_module'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # install launch, params, rviz, models
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kevin Lloyd Lazaro',
    maintainer_email='z5503635@ad.unsw.edu.au',
    description='YOLO-based red_object detection (RealSense) with 3D projection',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'object_detect_yolo = perception_module.object_detect_yolo:main',
        ],
    },
)
