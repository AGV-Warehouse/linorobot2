# Copyright (c) 2026 AGV Warehouse
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

# Cart leg detector: publishes detected_dock_pose for the Nav2 docking
# server from the raw 360 deg lidar scan. Runs standalone so it can be
# tested against recorded bags:
#
#   ros2 launch linorobot2_navigation cart_leg_detector.launch.py
#   ros2 bag play <phase0_bag>          # in another terminal, for offline runs
#
# Tune via config/cart_leg_detector.yaml (cart geometry from the Phase 0
# checklist in UNDER_CART_DOCKING.md) or pass params_file:=<your.yaml>.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare('linorobot2_navigation'), 'config',
         'cart_leg_detector.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='params_file',
            default_value=default_params,
            description='cart_leg_detector parameter file'
        ),
        Node(
            package='linorobot2_navigation',
            executable='cart_leg_detector.py',
            name='cart_leg_detector',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
