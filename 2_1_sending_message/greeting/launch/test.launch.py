import launch
import launch_ros.actions


def generate_launch_description():
    greeter = launch_ros.actions.Node(
        package='greeting',
        node_executable='greeter',
        output='screen'
        )
    displayer = launch_ros.actions.Node(
        package='displayer_basic_version',
        node_executable='displayer',
        output='screen'
        )
    
    messeage = launch.actions.LogInfo(
        msg="launch greeter and displayer node")

    # Launchしてから5秒後にメッセージを出力
    timer = launch.actions.TimerAction(
        period=5.0,
        actions=[
            launch.actions.LogInfo(msg="----------This is a TimerAction!!!---------"),
            ]
        )

    return launch.LaunchDescription([
        # messeage,
        # timer,
        greeter,
        displayer,
    ])