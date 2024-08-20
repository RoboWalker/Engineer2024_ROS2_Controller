# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    sim = Node(
        package="engineer_arm_sim",
        executable="sim"
    )
    controller = Node(
        package="engineer_arm_controller",
        executable="engineer_arm_controller"
    )
    communication = Node(
        package="customctrl_serial_communication",
        executable="customctrl_serial_communication"
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [sim, controller, communication])
    # 返回让ROS2根据launch描述执行节点
    return launch_description

