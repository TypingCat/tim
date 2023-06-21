import launch
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    rviz_config = PathJoinSubstitution([FindPackageShare("tim"), "launch", "config.rviz"])
    
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='tim',
            executable='infra',
            name='infra_0',
            parameters=[
                {"id": 0},
                {"num_objects": 20},
                {"detection_range": 20.},
            ]
        ),
        launch_ros.actions.Node(
            package='tim',
            executable='vehicle',
            name='vehicle_0',
            parameters=[
                {"id": 0},
                {"obu_mode": "rosmsg"},
                {"color": [0., 1., 0.]},
            ]
        ),
        launch_ros.actions.Node(
            package='tim',
            executable='vehicle',
            name='vehicle_1',
            parameters=[
                {"id": 1},
                {"obu_mode": "string"},
                {"color": [1., 1., 0.]},
            ]
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])