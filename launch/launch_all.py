import os 
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, GroupAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml

def generate_launch_description():
    #Get the launch package and launch directory
    package_dir = get_package_share_directory("my_bot")
    launch_dir = os.path.join(package_dir, 'launch')

    #Config the launch configuaration variables
    slam = LaunchConfiguration('slam')
    slam_select = LaunchConfiguration('slam_select')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    load_state_filename = LaunchConfiguration('load_state_filename')

    # Config variables for simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    pose = {'x': LaunchConfiguration('x_pose', default='0.00'),
            'y': LaunchConfiguration('y_pose', default='0.00'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}
    robot_name = LaunchConfiguration('robot_name')


    #Remap topic for multi robot, if we have only one robot it is not necessary
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    #Declear the launch arguments 
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace !'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='False',
        description='Whether to apply use a namespace !'
    )

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run Slam !'
    )

    declare_slam_select_cmd = DeclareLaunchArgument(
        'slam_select',
        default_value='slam_toolbox',
        description="Choose SLAM core: slamtoolbox or cartographer"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value = os.path.join(package_dir,'map/slam_map/my_map.yaml'),
        description='Full path to map file !'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation if true !'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(package_dir,'config','nav2_params.yaml'),
        description='Full path to ROS2 param file to use for all launch nodes'
    )

    declare_load_state_filename_cmd = DeclareLaunchArgument(
        'load_state_filename',
        default_value='',
        description='Full path to .pbstream file for Cartographer'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack'
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', 
        default_value='True',
        description='Whether to use composed bringup')
    
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Respawn if a node crashes'
    )

    declare_use_simulator_cmd =DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator'
    )

    declare_log_level_cmd = DeclareLaunchArgument(
    'log_level', default_value='info',
    description='log level')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(package_dir,'rviz2','rviz2_config.rviz'),
        description='Full path to the Rviz2 config file'
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher'
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='Huy',
        description='name of the robot')    

    declare_use_rviz_cmd =DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start Rviz'
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(package_dir,'worlds','my_world'),
        description='Full path to world'
    )

    declare_robot_xacro_cmd = DeclareLaunchArgument(
        'robot_xacro',
        default_value=os.path.join(package_dir,'description', 'robot.urdf.xacro'),
        description='Full path to robot xacro file'
    )
    
    #Convert xacro to xml
    xacro_file = os.path.join(package_dir,'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description_xml = robot_description_config.toxml()   

    #Specify the actions 
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        cwd=[launch_dir], output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression(
            [use_simulator, ' and not ', headless])),
        cmd=['gzclient'],
        cwd=[launch_dir], output='screen')

    start_robot_state_publisher_cmd = Node(
        condition = IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'robot_description':robot_description_xml, 'use_sim_time':use_sim_time}],
        remappings=remappings
    )

    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-topic', 'robot_description', 
            '-robot_namespace', namespace,
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']])

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'rviz_config': rviz_config_file}.items())
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"]
    )
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"]
    )
 
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)     
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'cartographer_sim.launch.py')),
            condition=IfCondition(PythonExpression([
                "'", slam, "' == 'True' and '", slam_select, "' == 'cartographer'"
            ])),
            launch_arguments={'use_sim_time': use_sim_time,
                              'load_state_filename': load_state_filename}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                       'localization_launch.py')),
            condition=IfCondition(PythonExpression(["not '", slam, "' == 'True'"])),
            launch_arguments={'namespace': namespace,
                              'map': map_yaml_file,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'twist_mux.launch.py'))
        ),
    ])
   # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_slam_select_cmd)
    ld.add_action(declare_load_state_filename_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)


    # Add any conditioned actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_gazebo_spawner_cmd)
    # ld.add_action(RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=start_gazebo_spawner_cmd,
    #         on_exit=[joint_broad_spawner],
    #     )
    # ))
    # ld.add_action(RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=joint_broad_spawner,
    #             on_exit=[diff_drive_spawner],
    #         )
    #     ))

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd_group)

    return ld
