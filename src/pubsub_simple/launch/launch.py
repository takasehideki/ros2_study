from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='pubsub_simple',
      executable='publisher',
      name='publisher1'
    ),
    Node(
      package='pubsub_simple',
      executable='publisher',
      name='publisher2'
    ),
    Node(
      package='pubsub_simple',
      executable='subscriber',
      name='subscriber'
    )
  ])
    