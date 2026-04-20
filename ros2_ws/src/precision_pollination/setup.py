from setuptools import find_packages, setup

package_name = 'precision_pollination'

setup(
    name=package_name,
    version='0.4.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FYP Student',
    description='Swarm drone pollination — Week 9 Full Integration',
    entry_points={
        'console_scripts': [
            'pollination_controller=precision_pollination.pollination_controller:main',
            'yolov8_detector=precision_pollination.yolov8_detector:main',
            'swarm_coordinator=precision_pollination.swarm_coordinator:main',
            'camera_bridge=precision_pollination.camera_bridge:main',
            'position_estimator=precision_pollination.position_estimator:main',
            'mission_logger=precision_pollination.mission_logger:main',
            'flower_detector_sim=precision_pollination.flower_detector_sim:main',
        ],
    },
)
