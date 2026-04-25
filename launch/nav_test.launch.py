import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument,
  IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression, LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml, ReplaceString


def generate_launch_description():
  # Set the path to this package.
  pkg_path = get_package_share_directory('theta_star_smooth_planner')
  # nav2_bringup_pkg_path = get_package_share_directory('nav2_bringup')

  # Set the path to the nav params file
  nav_params_file_name = 'nav2_bringup_params.yaml'
  nav_params_file = os.path.join(pkg_path, 'config', nav_params_file_name)

  # Set the path to the map file used by AMCL
  map_file_name = 'room_with_walls.yaml'
  map_file = os.path.join(pkg_path, 'maps', map_file_name)
 
  #--------------------------------------------------------------------------

  # Launch configuration variables specific to simulation
  use_sim_time = LaunchConfiguration('use_sim_time')
  nav_params = LaunchConfiguration('nav_params')
  map = LaunchConfiguration('map')

  declare_use_sim_time_cmd = DeclareLaunchArgument(
      name='use_sim_time', 
      default_value='True',
      description='Flag to enable use_sim_time'
    )
  
  declare_nav_params_cmd = DeclareLaunchArgument(
      name='nav_params',
      default_value=nav_params_file,
      # default_value=rewritten_nav_params_file,
      description='file path to the parameter file'
    )
  
  declare_map_cmd = DeclareLaunchArgument(
      name='map',
      default_value=map_file,
      description='file path to the map needed for navigation'
    )


  #-----------------------------------------------------------------------------

  rviz_config_file = os.path.join(pkg_path,'config','amcl.rviz')

  # create needed nodes or launch files
  rviz_node = Node(
      package='rviz2',
      executable='rviz2',
      arguments=['-d', rviz_config_file],
      output='screen'
  )

  #-----------------------------------------------------------------------------

  amcl_launch_path = os.path.join(pkg_path, 'launch', 'amcl.launch.py')

  amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(amcl_launch_path),
        launch_arguments={
                'use_sim_time': use_sim_time,
                'nav_params': nav_params,
                'map': map,
        }.items(),
    )

  #--------------------------------------------------------------------------------

  lifecycle_nodes = [
    # 'costmap',
    'planner_server',
    'controller_server',
    'bt_navigator',
    'behavior_server',
    'smoother_server',
    'waypoint_follower',
    # 'velocity_smoother',
  ]

  remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

  # nav2_costmap_2d_node = Node(
  #   package='nav2_costmap_2d',
  #   executable='nav2_costmap_2d',
  #   name='costmap',
  #   output='screen',
  #   parameters=[
  #     nav_params,
  #     {'use_sim_time': use_sim_time}
  #   ],
  # )

  nav2_planner_server_node = Node(
    package='nav2_planner',
    executable='planner_server',
    name='planner_server',
    output='screen',
    parameters=[
      nav_params,
      {'use_sim_time': use_sim_time}
    ],
    remappings=remappings,
  )

  nav2_smoother_server_node = Node(
    package='nav2_smoother',
    executable='smoother_server',
    name='smoother_server',
    output='screen',
    parameters=[
      nav_params,
      {'use_sim_time': use_sim_time}
    ],
    remappings=remappings,
  )

  nav2_controller_server_node = Node(
    package='nav2_controller',
    executable='controller_server',
    name='controller_server',
    output='screen',
    parameters=[
      nav_params,
      {'use_sim_time': use_sim_time}
    ],
    remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
  )

  nav2_bt_navigator_node = Node(
    package='nav2_bt_navigator',
    executable='bt_navigator',
    name='bt_navigator',
    output='screen',
    parameters=[
      nav_params,
      {'use_sim_time': use_sim_time}
    ],
    remappings=remappings,
  )

  nav2_behavior_server_node = Node(
    package='nav2_behaviors',
    executable='behavior_server',
    name='behavior_server',
    output='screen',
    parameters=[
      nav_params,
      {'use_sim_time': use_sim_time}
    ],
    remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
  )

  nav2_waypoint_follower_node = Node(
    package='nav2_waypoint_follower',
    executable='waypoint_follower',
    name='waypoint_follower',
    output='screen',
    parameters=[
      nav_params,
      {'use_sim_time': use_sim_time}
    ],
    remappings=remappings,
  )

  # nav2_velocity_smoother_node = Node(
  #   package='nav2_velocity_smoother',
  #   executable='velocity_smoother',
  #   name='velocity_smoother',
  #   output='screen',
  #   parameters=[
  #     nav_params,
  #     {'use_sim_time': use_sim_time}
  #   ],
  #   remappings=remappings
  #   + [('cmd_vel', 'cmd_vel_nav')],
  # )

  nav2_lifecycle_manager_node = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    output='screen',
    parameters=[{"autostart": True, "bond_timeout": 0.0}, {'node_names': lifecycle_nodes}],
  )

  #--------------------------------------------------------------------------------

  # Create the launch description
  ld = LaunchDescription()
 
  # add the necessary declared launch arguments to the launch description
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_nav_params_cmd)
  ld.add_action(declare_map_cmd)
 
  # Add the nodes to the launch description
  ld.add_action(amcl_launch)
  ld.add_action(nav2_planner_server_node)
  ld.add_action(nav2_smoother_server_node)
  ld.add_action(nav2_controller_server_node)
  ld.add_action(nav2_bt_navigator_node)
  ld.add_action(nav2_behavior_server_node)
  ld.add_action(nav2_waypoint_follower_node)
  # ld.add_action(nav2_velocity_smoother_node)
  ld.add_action(nav2_lifecycle_manager_node)
  ld.add_action(rviz_node)

  return ld
