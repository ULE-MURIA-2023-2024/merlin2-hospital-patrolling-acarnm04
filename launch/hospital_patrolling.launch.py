# Copyright (C) 2024  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import ament_index_python
from kant_dao.dao_factory import DaoFamilies
from launch_ros.actions import Node


def generate_launch_description():

    planning_layer_share_dir = get_package_share_directory(
        "merlin2_planning_layer")
    waypoint_navigation_share_dir = get_package_share_directory(
        "waypoint_navigation")
    text_to_speech_share_dir = get_package_share_directory(
        "text_to_speech")

    #
    # ARGS
    #
    dao_family = LaunchConfiguration("dao_family")
    dao_family_cmd = DeclareLaunchArgument(
        "dao_family",
        default_value=str(int(DaoFamilies.ROS2)),
        description="DAO family")

    mongo_uri = LaunchConfiguration("mongo_uri")
    mongo_uri_cmd = DeclareLaunchArgument(
        "mongo_uri",
        default_value="mongodb://localhost:27017/merlin2",
        description="MongoDB URI")

    planner = LaunchConfiguration("planner")
    planner_cmd = DeclareLaunchArgument(
        "planner",
        default_value="1",
        description="PDDL planner")

    #
    # NODES
    #
    merlin2_navigation_action_cmd = Node(
        package="merlin2_basic_actions",
        executable="merlin2_navigation_fsm_action",
        name="navigation",
        parameters=[{
            "dao_family": dao_family,
            "mongo_uri": mongo_uri
        }]
    )

    # TODO: create the patrol action node
    # TODO: create the mission node

    #
    # LAUNCHES
    #
    waypoint_nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(waypoint_navigation_share_dir, "waypoint_navigation.launch.py")),
        launch_arguments={"wps": ament_index_python.get_package_share_directory(
            "merlin2_hospital_patrolling") + "/params/stretcher_room_waypoints.yaml"}.items()
    )

    text_to_speech_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(text_to_speech_share_dir, "text_to_speech.launch.py"))
    )

    merlin2_planning_layer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planning_layer_share_dir, "merlin2_planning_layer.launch.py")),
        launch_arguments={
            "dao_family": dao_family,
            "mongo_uri": mongo_uri,
            "planner": planner
        }.items()
    )

    ld = LaunchDescription()

    #
    # ADD
    #
    ld.add_action(dao_family_cmd)
    ld.add_action(planner_cmd)
    ld.add_action(mongo_uri_cmd)

    # TODO: add the patrol action node
    # TODO: add the mission node
    ld.add_action(merlin2_navigation_action_cmd)

    ld.add_action(text_to_speech_cmd)
    ld.add_action(waypoint_nav_cmd)
    ld.add_action(merlin2_planning_layer_cmd)

    return ld
