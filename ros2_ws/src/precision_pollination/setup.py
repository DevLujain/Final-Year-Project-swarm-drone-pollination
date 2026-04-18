from setuptools import find_packages, setup

package_name = 'precision_pollination'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FYP Student',
    description='Swarm drone pollination system',
    entry_points={
        'console_scripts': [
            'pollination_controller = precision_pollination.pollination_controller:main',
            'yolov8_detector = precision_pollination.yolov8_detector:main',
        ],
    },
)
