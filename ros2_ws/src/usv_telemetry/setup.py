from setuptools import find_packages, setup

package_name = 'usv_telemetry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Thaysa Roberta da Silva',
    maintainer_email='thaysa@example.com',
    description='Sistema de controle e telemetria para USVs com ROS 2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gps_node            = usv_telemetry.gps_node:main',
            'imu_node            = usv_telemetry.imu_node:main',
            'control_node        = usv_telemetry.control_node:main',
            'mavlink_bridge_node = usv_telemetry.mavlink_bridge_node:main',
        ],
    },
)
