# Copyright 2026 ktj
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("leader_platooning_beacon")
    params_file = os.path.join(package_share, "config", "beacon_params.yaml")

    return LaunchDescription([
        Node(
            package="leader_platooning_beacon",
            executable="leader_platooning_beacon_node",
            name="leader_platooning_beacon",
            output="screen",
            parameters=[params_file],
        ),
    ])
