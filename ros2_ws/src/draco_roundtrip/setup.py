from setuptools import find_packages, setup

package_name = 'draco_roundtrip'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=['draco_roundtrip', 'draco_roundtrip.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['draco_roundtrip/configs/qos_override.yaml']),
    ],
    install_requires=['setuptools', 'ament_index_python'],
    zip_safe=False,
    maintainer='GodokSa',
    maintainer_email='ppakdone@gmail.com',
    description='ROS 2 client/server nodes for Draco point cloud roundtrip experiments.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stream_client = draco_roundtrip.cli.stream_client:main',
            'stream_server = draco_roundtrip.cli.stream_server:main',
            'stream_monitor = draco_roundtrip.cli.monitor:main',
            'stream_replay = draco_roundtrip.cli.replay:main',
        ],
    },
)
