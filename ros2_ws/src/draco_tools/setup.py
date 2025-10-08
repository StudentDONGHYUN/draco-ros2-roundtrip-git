from setuptools import find_packages, setup

package_name = 'draco_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=['draco_tools', 'draco_tools.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='GodokSa',
    maintainer_email='ppakdone@gmail.com',
    description='Support utilities for Draco roundtrip experiments.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag_to_ply = draco_tools.bag_to_ply:main',
            'encode_ply_to_draco = draco_tools.cli.encode_ply_to_draco:main',
            'offline_pipeline = draco_tools.offline_pipeline:main',
        ],
    },
)
