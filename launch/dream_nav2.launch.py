from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, \
        DeclareLaunchArgument, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # launch substiutions
    use_sim_time = LaunchConfiguration('use_sim_time')
    logger = LaunchConfiguration('log_level')
    vehicle = LaunchConfiguration('vehicle')

    # launch arguments
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value=['true'],
        description='use simulation time'
    )

    arg_log_level = DeclareLaunchArgument(
        'log_level',
        default_value=['warn'],
        description='Logging level'
    )

    arg_vehicle = DeclareLaunchArgument(
        'vehicle',
        default_value=['mrb3s'],
        description='vehicle'
    )

    # nodes
    launch_cerebri = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare('cerebri'), 'launch', 'cerebri.launch.py'])),
        launch_arguments={
            'vehicle': vehicle
        }.items()
    )

    launch_corti = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare('corti'), 'launch', 'corti_nav2.launch.py'])),
        launch_arguments={
            'vehicle': vehicle
        }.items()
    )

    launch_electrode = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare('electrode'), 'launch', 'electrode_nav2.launch.py'])),
        launch_arguments={
            'vehicle': vehicle
        }.items()
    )

    launch_gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': [vehicle , '_map.sdf',
                ' --gui-config ', FindPackageShare('dream'), 'config', 'gui.confg',
                ' -v 0',
                ' -r']
        }.items(),
    )

    node_ros_gz_bridge = Node(
       package='ros_gz_bridge',
       output='log',
       executable='parameter_bridge',
       arguments=[
         # Send Gazebo clock to ROS
         '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
         # joystick from ROS to Gazebo
         '/joy@sensor_msgs/msg/Joy@gz.msgs.Joy',
         # trajectory from ROS to Gazesbo
         '/traj@synapse_msgs/msg/BezierTrajectory@gz.msgs.BezierTrajectory',
         # odometry from Gazebo model to ROS
         '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
         #'/model/mrb3s/odometry_with_covariance@nav_msgs/msg/Odometry@'
         #'gz.msgs.OdometryWithCovariance',
         # Sensors
         '/scan@'
         'sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
         # ros args
         '--ros-args', '--log-level', logger,
       ],
       parameters=[{'use_sim_time': use_sim_time}],
       on_exit=Shutdown()
    )

    return LaunchDescription([
        arg_use_sim_time,
        arg_log_level,
        arg_vehicle,
        launch_cerebri,
        launch_corti,
        launch_electrode,
        launch_gazebo,
        node_ros_gz_bridge,
        ])
