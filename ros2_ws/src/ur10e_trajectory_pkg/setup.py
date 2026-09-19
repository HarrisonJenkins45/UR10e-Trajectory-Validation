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
        ('share/' + package_name + '/rviz', ['rviz/target_preview.rviz']),

    ],


    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hjenkins33',
    maintainer_email='hjenkins33@gatech.edu',
    description='Orientation-only rail-and-arm section planning and playback',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'validation_server = ur10e_trajectory_pkg.Validate_trajServer:main',
	    'obstacle_markers=ur10e_trajectory_pkg.obstacle_markers:main',

            'trajectory_client = ur10e_trajectory_pkg.ClientNode:main',
            'trajectory_pipeline = ur10e_trajectory_pkg.pipeline:main',
            'trajectory_sections = ur10e_trajectory_pkg.section_planner:main',
            'trajectory_fast_section = ur10e_trajectory_pkg.fast_section:main',
            'preview_certified_plan_rviz = ur10e_trajectory_pkg.preview_certified_plan_rviz:main',
            'joint_state_to_gazebo_bridge = ur10e_trajectory_pkg.joint_state_to_gazebo_bridge:main',
        ],
    },
)
