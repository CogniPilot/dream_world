from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, \
        DeclareLaunchArgument, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = True
    logger = LaunchConfiguration("log_level")
    gui_config = get_package_share_directory('dream') + '/config/gui.config'
    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value=["warn"],
            description="Logging level"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            get_package_share_directory('cerebri') + '/launch/cerebri.launch.py')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            get_package_share_directory('corti') + '/launch/mrb3s_corti.launch.py')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            get_package_share_directory('electrode') + '/launch/mrb3s_electrode.launch.py')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            get_package_share_directory('ros_gz_sim') + '/launch/gz_sim.launch.py'),
            launch_arguments={
                'gz_args': f'mrb3s_map.sdf -v 0 --gui-config {gui_config} -r -s'
            }.items()),
        Node(
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
             '/model/mrb3s/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
             '/model/mrb3s/odometry_with_covariance@nav_msgs/msg/Odometry@'
             'gz.msgs.OdometryWithCovariance',
             # Sensors
             '/world/default/model/mrb3s/link/RPLIDAR_A1M8/Base/sensor/lidar/scan@'
             'sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
             # ros args
             '--ros-args', '--log-level', logger,
           ],
           parameters=[{'use_sim_time': use_sim_time}],
           on_exit=Shutdown()
        ),
    ])
