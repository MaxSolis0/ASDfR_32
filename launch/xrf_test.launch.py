from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():

    # Get current working directory (should be /ros2_ws)
    ws_root = os.getcwd()
    ws_src = os.path.join(ws_root, 'src')

    return LaunchDescription([
        
        # =============================
        # 1) RELbot adapter
        # =============================
        Node(
            package='relbot_adapter',
            executable='relbot_adapter',
            name='relbot_adapter',
            output='screen',
            parameters=[
                {'robotmode': 'real'}
            ]
        ),

        

        # =============================
        # 2) Setpoint sequence node
        # =============================
        Node(
            package='xrf_test',
            executable='tester',
            name='tester',
            output='screen',
            parameters=[
                os.path.join(
                    ws_src,
                    'xrf_test',
                    'config',
                    'tester.yaml'
                )
            ]
        ),

    ])
