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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

import xacro


def generate_launch_description():

    pkg_path = os.path.join(
        get_package_share_directory('gz_arm'))

    res_path = os.path.join(pkg_path, 'resources')
    robot_description_path = os.path.join(res_path, 'gz_arm.xacro.urdf')
    doc = xacro.parse(open(robot_description_path))
    xacro.process_doc(doc)
    robot_description = doc.toxml()

    aruco_img_path = '/tmp/gz_arm_aruco.png'
    gen_aruco = ExecuteProcess(
        cmd=['ros2', 'run', 'gz_arm', 'gen_aruco', '0',
             '--size', '120', '--out', aruco_img_path],
        output='screen'
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'position_controller'],
        output='screen'
    )

    delay = ExecuteProcess(
        cmd=['sleep', '3'],
        output='screen'
    )

    deactivate_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'set_controller_state',
             'position_controller', 'inactive'],
        output='screen'
    )

    unload_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'unload_controller', 'position_controller'],
        output='screen'
    )

    load_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'velocity_controller'],
        output='screen'
    )

    # TODO use tmpfile for sdfpath
    sdf_path = '/tmp/gz_arm.sdf'
    sdf_xacro_path = os.path.join(res_path, 'gz_arm.xacro.sdf')
    doc = xacro.parse(open(sdf_xacro_path))
    xacro.process_doc(doc)
    sdf = doc.toxml()
    with open(sdf_path, 'w') as fd:
        fd.write(sdf)

    run_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', sdf_path, '-v3'],
        output='screen'
    )

    return LaunchDescription([
        gen_aruco,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gen_aruco,
                on_exit=[run_gazebo],
            )
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                "robot_description": robot_description
            }],
        ),
        load_joint_state_broadcaster,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_position_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_position_controller,
                on_exit=[delay],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=delay,
                on_exit=[deactivate_position_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=deactivate_position_controller,
                on_exit=[unload_position_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=unload_position_controller,
                on_exit=[load_velocity_controller],
            )
        ),
    ])
