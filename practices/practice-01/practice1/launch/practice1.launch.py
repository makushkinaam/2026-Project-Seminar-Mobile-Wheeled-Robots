from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    TimerAction,
    LogInfo,
    RegisterEventHandler,
    DeclareLaunchArgument,
)
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    declare_turtle1_name = DeclareLaunchArgument(
        'turtle_name1', default_value='turtle2'
    )

    declare_turtle2_name = DeclareLaunchArgument(
        'turtle_name2', default_value='turtle3'
    )


    declare_num1 = DeclareLaunchArgument(
        'num1', default_value='0'
    )

    declare_num2 = DeclareLaunchArgument(
        'num2', default_value='0'
    )

    turtlesim = Node(
        package="turtlesim", executable="turtlesim_node", name="turtlesim"
    )

    kill_turtle = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/kill",
            "turtlesim/srv/Kill",
            "{name: turtle1}",
        ],
        output="screen",
    )

    turtle_name1 = LaunchConfiguration('turtle_name1')
    turtle_name2 = LaunchConfiguration('turtle_name2')
    num1 = LaunchConfiguration('num1')
    num2 = LaunchConfiguration('num2')

    def spawn_turtle(name, x, y):
        return ExecuteProcess(
                    cmd=[
                        "ros2",
                        "service",
                        "call",
                        "/spawn",
                        "turtlesim/srv/Spawn",
                        ["{x: ", x, ", y: ", y, ", theta: 0, name: '", name, "'}"],
                    ],
                    output="screen",
                )

    spawn_turtle1 = spawn_turtle(turtle_name1, '5', '3')
    spawn_turtle2 = spawn_turtle(turtle_name2, '9', '3')

    sine1 = Node(
        package="practice1",
        executable="practice1_node",
        name=["practice1_nod_", turtle_name1],
        parameters=[{"topic": ["/", turtle_name1], "num": num1}],
        output="screen",
    )

    sine2 = Node(
        package="practice1",
        executable="practice1_node",
        name=["practice1_nod_", turtle_name2],
        parameters=[{"topic": ["/", turtle_name2], "num": num2}],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_turtle1_name,
            declare_turtle2_name,
            declare_num1,
            declare_num2,
            turtlesim,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=turtlesim,
                    on_start=[
                        LogInfo(msg="Turtlesim started, spawning turtle"),
                        kill_turtle,
                        spawn_turtle1,
                        spawn_turtle2,
                    ],
                )
            ),
            TimerAction(period=0.2, actions=[sine1, sine2]),
        ]
    )