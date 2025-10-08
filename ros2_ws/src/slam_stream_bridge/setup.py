from glob import glob
from setuptools import find_packages, setup

package_name = 'slam_stream_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(package_name + '/launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='GodokSa',
    maintainer_email='ppakdone@gmail.com',
    description='Launch files bridging SLAM pipelines with Draco streaming utilities.',
    license='TODO',
    tests_require=['pytest'],
)
