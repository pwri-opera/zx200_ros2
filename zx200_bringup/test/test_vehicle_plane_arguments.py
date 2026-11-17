"""Check vehicle -> COM3 argument forwarding without starting hardware nodes."""
import importlib.util
from pathlib import Path
import unittest

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions


def vehicle_description():
    path = Path(__file__).parents[1] / 'launch' / 'vehicle.launch.py'
    spec = importlib.util.spec_from_file_location('vehicle_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def com3_arguments(overrides=None):
    context = LaunchContext()
    context.launch_configurations.update(overrides or {})
    description = vehicle_description()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    # COM3 must be a top-level include, outside MoveIt's namespace/remap group.
    includes = [a for a in description.entities if isinstance(a, IncludeLaunchDescription)]
    assert len(includes) == 1
    include = includes[0]
    arguments = {name: perform_substitutions(context, normalize_to_list_of_substitutions(value))
                 for name, value in include.launch_arguments}
    return arguments, include.condition.evaluate(context)


class VehiclePlaneArguments(unittest.TestCase):
    def test_default_nine_to_six(self):
        arguments, enabled = com3_arguments()
        self.assertTrue(enabled)
        self.assertEqual(arguments['input_plane'], '9')
        self.assertEqual(arguments['output_plane'], '6')
        self.assertEqual(arguments['ns'], 'zx200')
        self.assertEqual(arguments['pose_timeout'], '0.5')
        self.assertEqual(arguments['can_port'], 'can0')

    def test_override_reaches_com3(self):
        arguments, enabled = com3_arguments({
            'input_plane': '7', 'output_plane': '8', 'robot_name': 'zx200_test',
            'can_port': 'vcan_zx_test', 'pose_timeout': '0.25', 'dbc_path': '/tmp/test.dbc'})
        self.assertTrue(enabled)
        self.assertEqual(arguments, {
            'input_plane': '7', 'output_plane': '8', 'ns': 'zx200_test',
            'can_port': 'vcan_zx_test', 'pose_timeout': '0.25', 'dbc_path': '/tmp/test.dbc'})

    def test_disable_separate_gateway(self):
        _, enabled = com3_arguments({'start_com3': 'false'})
        self.assertFalse(enabled)


if __name__ == '__main__':
    unittest.main()
