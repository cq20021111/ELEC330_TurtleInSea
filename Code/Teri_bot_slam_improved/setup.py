from setuptools import setup  # Import setuptools to configure and install the Python package
from glob import glob  # Import glob to match file patterns

# Define the package name
package_name = 'Teri_bot'

# Call the setup function to configure the package metadata and install information
setup(
    name=package_name,  # Name of the package
    version='0.0.0',  # Version of the package
    packages=[package_name],  # Modules or sub-packages included in the package, typically the directory name
    data_files=[  # Used to specify files to be copied during installation and their destination
        ('share/ament_index/resource_index/packages',  # ROS 2 resource index directory
            ['resource/' + package_name]),  # Resource file for the package (e.g., marker file), installed to the resource index
        ('share/' + package_name, ['package.xml']),  # Copy the package.xml file to the share/package_name directory
        ('share/' + package_name + '/launch/', glob('launch/*.py')),  # Copy all Python launch files from the launch folder to share/package_name/launch/
        ('share/' + package_name + '/urdf/', glob('urdf/*')),  # Copy all files from the urdf folder to share/package_name/urdf/
        ('share/' + package_name + '/rviz/', glob('rviz/*')),  # Copy all files from the rviz folder to share/package_name/rviz/
        ('share/' + package_name + '/meshes/collision/', glob('meshes/collision/*')),  # Copy all mesh files from the collision folder to share/package_name/meshes/collision/
        ('share/' + package_name + '/meshes/visual/', glob('meshes/visual/*')),  # Copy all mesh files from the visual folder to share/package_name/meshes/visual/
        ('share/' + package_name + '/config/', glob('config/*.yaml')),  # Copy configuration files for SLAM
    ],
    install_requires=['setuptools'],  # Dependencies required to install this package
    zip_safe=True,  # If True, the package can be distributed safely as a zipped archive
    maintainer='ros-industrial',  # Name of the package maintainer
    maintainer_email='TODO:',  # Email of the package maintainer (should be updated with a valid email)
    description='TODO: Package description',  # A brief description of the package (should be updated with the actual description)
    license='TODO: License declaration',  # The type of license for the package (should be updated with the appropriate license)
    tests_require=['pytest'],  # Dependencies needed for running tests, typically a testing tool like pytest
    entry_points={  # Define console script entry points for creating executable command-line tools
        'console_scripts': [
            'joint_controller = Teri_bot.joint_controller:main',  # Entry point for the joint_controller script
            'object_recognizer = Teri_bot.object_recognizer:main',
            'lidar_converter = Teri_bot.lidar_converter:main',  
            'map_initializer = Teri_bot.map_initializer:main',  
        ],
    },
)


