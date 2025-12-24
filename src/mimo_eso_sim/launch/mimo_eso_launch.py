import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 物理仿真节点 (Plant)
    plant_node = Node(
        package='mimo_eso_sim',
        executable='plant_node',
        name='plant_node',
        output='screen'
    )

    # 2. 观测器节点 (ESO)
    eso_node = Node(
        package='mimo_eso_sim',
        executable='eso_node',
        name='eso_node',
        output='screen'
    )

    # 3. 更换为 PlotJuggler
    plotjuggler = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='plotjuggler',
        # 这里不需要预设话题，启动后手动勾选更灵活
        arguments=[]
    )

    return LaunchDescription([
        plant_node,
        eso_node,
        plotjuggler
    ])