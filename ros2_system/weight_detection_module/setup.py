from setuptools import find_packages, setup

package_name = 'weight_detection_module'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Weight detection module for UR5e robot using joint torques',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'weight_detector = weight_detection_module.weight_detector:main',
        ],
    },
)
