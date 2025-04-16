import os
import pytest
import launch
import launch_ros
import launch_testing
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

@pytest.mark.launch_test
def generate_test_description():
    # Get path to the launch file
    launch_file_dir = os.path.join(
        get_package_share_directory('ros2_ign_p2p_controllers'),
        'launch'
    )
    launch_file_path = os.path.join(launch_file_dir, 'simulator.launch.py')

    # Include the actual launch file
    launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_path)
    )

    return LaunchDescription([
        launch_description,
        # Required for launch_testing to know when to shut down
        launch_testing.actions.ReadyToTest()
    ]), {}

# Test that checks if the launch file starts without crashing
def test_processes_launch(context, proc_info):
    # Check all processes finished cleanly (no errors)
    proc_info.assertWaitForShutdown(process=None, timeout=30.0)
    proc_info.assertExitCodes(proc_matcher=None)
