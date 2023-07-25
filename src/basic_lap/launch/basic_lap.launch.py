from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    car_controller_node = Node(
        package='autonomous_example',
        executable='car_controller',
        name='car_controller'
    )

    cone_id_node = Node(
        package='autonomous_example',
        executable='cone_id',
        name='cone_id'
    )

    throttle_pos_node = Node(
        package='autonomous_example',
        executable='throttle_pos',
        name='throttle_pos',
        parameters=[{'max_speed': 6.5}]
    )

    steering_angle_node = Node(
        package='basic_lap',
        executable='steering_angle',
        name='steering_angle'
    )

    ld.add_action(car_controller_node)
    ld.add_action(cone_id_node)
    ld.add_action(steering_angle_node)
    ld.add_action(throttle_pos_node)

    return ld