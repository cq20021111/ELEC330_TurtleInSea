from setuptools import setup
from glob import glob

package_name = 'Teri_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', glob('launch/*.py')),
        ('share/' + package_name + '/urdf/', glob('urdf/*')),
        ('share/' + package_name + '/rviz/', glob('rviz/*')),
        ('share/' + package_name + '/meshes/collision/', glob('meshes/collision/*')),
        ('share/' + package_name + '/meshes/visual/', glob('meshes/visual/*')),
        ('share/' + package_name + '/config/', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros-industrial',
    maintainer_email='TODO:',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_controller = Teri_bot.joint_controller:main',
            'object_recognizer = Teri_bot.object_recognizer:main',
            'slam_continuous_updater = Teri_bot.slam_continuous_updater:main', 
            'lidar_converter = Teri_bot.lidar_converter:main',
        ],
    },
)