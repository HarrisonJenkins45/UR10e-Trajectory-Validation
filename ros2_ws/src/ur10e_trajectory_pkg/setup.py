from setuptools import find_packages, setup

package_name = 'ur10e_trajectory_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/launch', ['ur10e_trajectory_pkg/VisualizeTraj.py']),
        ('share/' + package_name + '/launch', ['ur10e_trajectory_pkg/VisualizeTraj_RvizPlayback.py']),
        # ('share/' + package_name + '/launch', ['ur10e_trajectory_pkg/VisTraj_Rviz.py']),

    ],


    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hjenkins33',
    maintainer_email='hjenkins33@gatech.edu',
    description='Trajectory validation server and client nodes',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'validation_server = ur10e_trajectory_pkg.Validate_trajServer:main',
	    'obstacle_markers=ur10e_trajectory_pkg.obstacle_markers:main',

            'trajectory_client = ur10e_trajectory_pkg.ClientNode:main',
            'joint_state_node = ur10e_trajectory_pkg.joint_state_node:main',
            'joint_state_to_gazebo_bridge = ur10e_trajectory_pkg.joint_state_to_gazebo_bridge:main',
        ],
    },
)
