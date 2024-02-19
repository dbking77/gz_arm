# MIT License

# Copyright (c) 2024 Derek King

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (LaunchConfiguration,
                                  TextSubstitution)


def generate_launch_description():
    aruco_size_arg = DeclareLaunchArgument(
        "aruco_size", default_value=TextSubstitution(text="0.1")
    )

    target_aruco_id_arg = DeclareLaunchArgument(
        "target_aruco_id", default_value=TextSubstitution(text="0")
    )

    return LaunchDescription([
        aruco_size_arg,
        target_aruco_id_arg,
        Node(
            package='gz_arm',
            executable='gz_arm_exec',
            # name='gz_arm_exec',
            parameters=[{
                "use_sim_time": True,
                "gz_arm_control": {
                    "aruco_size": LaunchConfiguration("aruco_size"),
                    "target_offset": 0.4,
                    "angular_gain": 1.0,
                    "positional_gain": 0.5,
                    "linear_velocity_limit": 0.1,
                    "joint_velocity_limit": 0.2,
                },
                "aruco_detect": {
                    "aruco_size": LaunchConfiguration("aruco_size"),
                    "target_aruco_id": LaunchConfiguration("target_aruco_id"),
                },
            }],
        )
    ])
