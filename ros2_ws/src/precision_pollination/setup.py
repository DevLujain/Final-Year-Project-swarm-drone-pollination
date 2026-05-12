from setuptools import find_packages, setup
package_name = 'precision_pollination'
setup(
    name=package_name, version='0.13.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Lujain Alhumaidah',
    description='Autonomous swarm drone pollination — Supervisor Demo',
    entry_points={'console_scripts': [
        'pollination_controller        = precision_pollination.pollination_controller:main',
        'yolov8_detector               = precision_pollination.yolov8_detector:main',
        'swarm_coordinator             = precision_pollination.swarm_coordinator:main',
        'mission_logger                = precision_pollination.mission_logger:main',
        'flower_detector_sim           = precision_pollination.flower_detector_sim:main',
        'lawnmower_sweep               = precision_pollination.lawnmower_sweep:main',
        'shared_visited_list           = precision_pollination.shared_visited_list:main',
        'flower_readiness_filter       = precision_pollination.flower_readiness_filter:main',
        'readiness_logger              = precision_pollination.readiness_logger:main',
        'persistent_pollination_manager= precision_pollination.persistent_pollination_manager:main',
        'tsp_planner                   = precision_pollination.tsp_planner:main',
        'field_health_mapper           = precision_pollination.field_health_mapper:main',
        'kpi_logger                    = precision_pollination.kpi_logger:main',
        # PHASE 2 HARDWARE - battery_manager deferred
        # PHASE 2 HARDWARE - drone_failure_simulator deferred
        'html_reporter                 = precision_pollination.html_reporter:main',
    ]},
)
