from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Launch Arguments의 값에 따라 실제 전달할 파라미터를 결정하는 함수"""

    # ```
    # ros2 launch  dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=m0609
    # ```
    # #### real
    # ```
    # ros2 launch  dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.1.100 port:=12345 model:=m0609
    # ```

    # 1. 사용자가 입력한 mode 값을 문자열로 가져옵니다.
    input_mode = LaunchConfiguration("mode").perform(context)

    # 2. mode 값에 따라 상수 파라미터를 설정합니다.
    if input_mode == "v":
        target_mode = "virtual"
        target_host = "127.0.0.1"
    elif input_mode == "r":
        target_mode = "real"
        target_host = "192.168.1.100"
    else:
        # 기본값 설정 또는 에러 처리
        print(f"Error: Invalid mode '{input_mode}'. Defaulting to virtual.")
        target_mode = "virtual"
        target_host = "127.0.0.1"

    target_port = "12345"  # 모든 모드에서 동일
    target_model = "m0609"  # 모든 모드에서 동일

    # 3. IncludeLaunchDescription 액션을 생성합니다.
    dsr_bringup_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("dsr_bringup2"),
                "/launch/dsr_bringup2_rviz.launch.py",
            ]
        ),
        # TextSubstitution을 사용하여 결정된 문자열 상수를 전달합니다.
        launch_arguments={
            "mode": TextSubstitution(text=target_mode),
            "host": TextSubstitution(text=target_host),
            "port": TextSubstitution(text=target_port),
            "model": TextSubstitution(text=target_model),
        }.items(),
    )

    # 4. 함수는 생성된 액션 리스트를 반환합니다.
    return [dsr_bringup_rviz_launch]


def generate_launch_description():
    # 1. 사용자가 입력할 mode Argument만 선언합니다.
    declare_mode_cmd = DeclareLaunchArgument(
        "mode", default_value="v", description="Work environment (v: virtual / r: real)"
    )

    # 2. OpaqueFunction을 사용하여 조건부 로직이 포함된 함수를 실행합니다.
    setup_action = OpaqueFunction(function=launch_setup)

    return LaunchDescription(
        [
            declare_mode_cmd,
            setup_action,
            Node(
                package="dsr_gss",
                executable="control_event_manager",
                name="control_event_manager",
            ),
            Node(
                package="dsr_gss",
                executable="coordinate_uploader",
                name="coordinate_uploader",
            ),
        ]
    )
