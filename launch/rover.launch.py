import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # parameters
    logger = LaunchConfiguration("log_level")
    model = LaunchConfiguration('model')
    sitl_binary = LaunchConfiguration('sitl_binary')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # variables
    gui_config = get_package_share_directory('dream') + '/config/gui.config'
    world = f'{model}_map.sdf'

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "log_level",
            default_value=["warn"],
            description="Logging level"),
        launch.actions.DeclareLaunchArgument(
            "model",
            default_value=["mrb3s"],
            description="model name"),
        launch.actions.DeclareLaunchArgument(
            "sitl_binary",
            default_value=["cerebri"],
            description="SITL binary"),
        launch.actions.DeclareLaunchArgument(
            "use_sim_time",
            default_value=True,
            description="use sim time"),
        ExecuteProcess(
            cmd=f"terminator -u -T cerebri --geometry=500x700+0+0 -e {sitl_binary} 2>&1".split(),
            output="log",
            shell=True
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            get_package_share_directory('corti') +  '/launch/corti.launch.py'),
            launch_arguments={
                'model': model,
                'use_sim_time': use_sim_time,
            }.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            get_package_share_directory('electrode') + '/launch/electrode.launch.py'),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            get_package_share_directory('ros_gz_sim') + '/launch/gz_sim.launch.py'),
            launch_arguments={
                'gz_args': f'{world} -v 0 --gui-config {gui_config} -r'
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
             f'/model/{model_name}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
             f'/model/{model_name}/odometry_with_covariance@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance',
             # Sensors
             '/world/default/model/{model_name}/link/RPLIDAR_A1M8/Base/sensor/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
             # ros args
             '--ros-args', '--log-level', logger,
           ],
           parameters=[{'use_sim_time': use_sim_time}],
           on_exit=launch.actions.Shutdown()
        ),
    ])
