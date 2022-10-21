# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    namespace_value = LaunchConfiguration('namespace', default='')
    image_value = LaunchConfiguration('image', default='image_raw')
    device_value = LaunchConfiguration('device', default='0')
    camera_info_value = LaunchConfiguration('camera_info', default='camera_info')
    frame_id_value = LaunchConfiguration('frame_id', default="camera")
    logger_name = LaunchConfiguration("log_level", default="info")
    camera_info_url_name = LaunchConfiguration("camera_info_url", default="ost.yaml")
    publish_rate_name = LaunchConfiguration("publish_rate", default='3.')

    image_proc = launch_ros.actions.Node(
        package='image_proc', executable='image_proc', output='screen',
        arguments=['--ros-args', '--log-level', logger_name],
        namespace=namespace_value,
        parameters=[
            # {"height": 360},
            {"height": 480},
            {"width": 640},
            {"use_scale": False},
            ],
        remappings=[('image/image', 'image/image_raw'),
                    ('resize/image', image_value),
                    ('camera_info', camera_info_value)]
    )

    l_info_device_value = LogInfo(msg="Device value {}".format(device_value))

    publisher = launch_ros.actions.Node(
        package='image_publisher', executable='image_publisher_node', output='screen',
        arguments=[device_value, '--ros-args', '--log-level', logger_name],
        namespace=namespace_value,
        parameters=[{"frame_id": frame_id_value},
                    {"publish_rate": publish_rate_name},
                    {"camera_info_url": camera_info_url_name},
                    ],
        remappings=[#('image_raw', 'publisher_out'),
                    ('camera_info', camera_info_value)]
    )

    return LaunchDescription([
        l_info_device_value,
        publisher,
        image_proc,
    ])
