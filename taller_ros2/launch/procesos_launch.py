# Copyright 2020 Paula Martinez
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    file_proceso_1 = Node(
        package='taller_ros2',
        node_executable='proceso_1',
        ##output='screen'
        )

    file_proceso_2 = Node(
        package='taller_ros2',
        node_executable='proceso_2',
        ##output='screen'
        )

    file_proceso_3 = Node(
        package='taller_ros2',
        node_executable='proceso_3',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(file_proceso_1)
    ld.add_action(file_proceso_2)
    ld.add_action(file_proceso_3)
    return ld
