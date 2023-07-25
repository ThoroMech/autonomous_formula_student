from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    car_controller_node = Node(
        package='autonomous_example',
        executable='car_controller',
        name='car_controller'
    )

    throttle_pos_node = Node(
        package='autonomous_example',
        executable='throttle_pos',
        name='throttle_pos'
    )

    steering_angle_node = Node(
        package='autonomous_example',
        executable='steering_angle',
        name='steering_angle'
    )  

    cone_id_node = Node(
        package='autonomous_example',
        executable='cone_id',
        name='cone_id'
    )

    cone_graph_node = Node(
        package='graphing',
        executable='cone_graph',
        name='cone_graph'
    )
    ld.add_action(car_controller_node)
    ld.add_action(cone_id_node)
    ld.add_action(steering_angle_node)
    ld.add_action(throttle_pos_node)
    ld.add_action(cone_graph_node)

    return ld