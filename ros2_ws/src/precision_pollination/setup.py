from setuptools import find_packages, setup

package_name = 'precision_pollination'

setup(
    name=package_name,
    version='0.15.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FYP Student',
    description='Swarm drone pollination — Stage 15',
    entry_points={
        'console_scripts': [
            # Stage 14 (kept)
            'visual_servoing_controller = precision_pollination.visual_servoing_controller:main',
            'swarm_drone_controller     = precision_pollination.swarm_drone_controller:main',
            'field_survey               = precision_pollination.field_survey:main',
            'swarm_coordinator          = precision_pollination.swarm_coordinator:main',
            'swarm_monitor              = precision_pollination.swarm_monitor:main',
            'mission_logger             = precision_pollination.mission_logger:main',
            # Stage 15 (new)
            'bloom_state_manager        = precision_pollination.bloom_state_manager:main',
            'battery_monitor            = precision_pollination.battery_monitor:main',
            'survey_node                = precision_pollination.survey_node:main',
            'map_merger                 = precision_pollination.map_merger:main',
            'mtsp_optimizer             = precision_pollination.mtsp_optimizer:main',
            'visual_servoing_v15        = precision_pollination.visual_servoing_v15:main',
            'mission_orchestrator       = precision_pollination.mission_orchestrator:main',
            'mission_logger_v15         = precision_pollination.mission_logger_v15:main',
            'swarm_coordinator_v15      = precision_pollination.swarm_coordinator_v15:main',
        ],
    },
)
