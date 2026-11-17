"""Launch file of the template controller: one consensus node for this agent.

The cosimulator starts it once per agent with the arguments listed in docs/CUSTOM_CONTROLLERS.md ("Launch arguments").
Every argument the framework passes must be declared here, used or not; the others are this controller's own
(`scenario.custom.launch.args` and `agents[].launch_args` in the experiment YAML).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

FRAMEWORK_ARGS = ['agent_name', 'agent_id', 'n_agents', 'px4_instance', 'px4_ns', 'spawn_x', 'spawn_y', 'spawn_z', 'spawn_yaw', 'log_dir']
OWN_ARGS = {'start_altitude': '5.0', 'gain': '0.3', 'run_s': '40.0', 'rate_hz': '5.0', 'dry_run': 'false'}


def generate_launch_description():
    cfg = LaunchConfiguration
    number = lambda name, kind: ParameterValue(cfg(name), value_type=kind)
    return LaunchDescription(
        [DeclareLaunchArgument(a, default_value='') for a in FRAMEWORK_ARGS]
        + [DeclareLaunchArgument(a, default_value=d) for a, d in OWN_ARGS.items()]
        + [Node(package='consensus_controller', executable='consensus_node', output='screen', parameters=[{
            'agent_id': number('agent_id', int),
            'px4_instance': number('px4_instance', int),
            'log_dir': cfg('log_dir'),
            'start_altitude': number('start_altitude', float),
            'gain': number('gain', float),
            'run_s': number('run_s', float),
            'rate_hz': number('rate_hz', float),
            'dry_run': number('dry_run', bool),      # true: no vehicle, only the consensus exchange (tests/custom_relay.sh)
        }])])
