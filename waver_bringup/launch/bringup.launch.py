import os
import re

import yaml

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction
)
from launch.launch_description_sources import AnyLaunchDescriptionSource


###########################################################################################################

# <!-- Launcher arguments -->
TOP_LEVEL_ARGUMENTS = (
    'profile',
    'namespace',
    'use_sim_time',
    'map'
)

FORWARDED_ARGUMENTS = (
    'namespace',
    'use_sim_time',
    'map'
)

# <!-- Profile configuration -->
STAGE_KEYS = {
    'enabled',
    'package',
    'launch_file',
    'arguments'
}

STRUCTURAL_OVERRIDES = {
    'enabled',
    'package',
    'launch_file'
}

# <!-- Supported substitutions -->
_SUBSTITUTION = re.compile(r'\$\(([^()]*)\)')
###########################################################################################################


class ProfileError(RuntimeError):
    """Represent an invalid or unresolved bringup profile configuration."""

    def __init__(self, message):
        super().__init__('[waver_bringup] ' + message)


###########################################################################################################

# <!-- Profile discovery -->
def _available_profiles():
    """Return the sorted profile names installed by ``waver_bringup``."""
    profiles_dir = os.path.join(
        get_package_share_directory('waver_bringup'),
        'profiles'
    )

    if not os.path.isdir(profiles_dir):
        return []

    return sorted(
        os.path.splitext(entry)[0]
        for entry in os.listdir(profiles_dir)
        if entry.endswith(('.yaml', '.yml'))
    )


def _resolve_profile_path(profile):
    """Resolve a profile name or YAML file path to an existing file."""
    if os.sep in profile or profile.endswith(('.yaml', '.yml')):
        path = os.path.expanduser(profile)

        if not os.path.isfile(path):
            raise ProfileError(
                f"profile file '{path}' does not exist"
            )

        return path

    path = os.path.join(
        get_package_share_directory('waver_bringup'),
        'profiles',
        profile + '.yaml'
    )

    if not os.path.isfile(path):
        raise ProfileError(
            f"unknown profile '{profile}'. Available profiles: "
            + ', '.join(_available_profiles())
        )

    return path
###########################################################################################################


# <!-- Profile value parsing -->
def _expand_substitutions(value, where):
    """Expand supported package-share and environment substitutions."""

    def replace(match):
        parts = match.group(1).split()

        if not parts:
            raise ProfileError(
                f'empty $() substitution in {where}'
            )

        kind, args = parts[0], parts[1:]

        if kind == 'find-pkg-share':
            if len(args) != 1:
                raise ProfileError(
                    f"$(find-pkg-share) in {where} expects exactly one "
                    f"package name, got: '{match.group(0)}'"
                )

            try:
                return get_package_share_directory(args[0])
            except PackageNotFoundError:
                raise ProfileError(
                    f"package '{args[0]}' referenced in {where} was not "
                    "found in the ament index. Is it built and sourced?"
                )

        if kind == 'env':
            if len(args) == 1:
                if args[0] not in os.environ:
                    raise ProfileError(
                        f"environment variable '{args[0]}' referenced in "
                        f'{where} is not set and has no default'
                    )

                return os.environ[args[0]]

            if len(args) == 2:
                return os.environ.get(args[0], args[1])

            raise ProfileError(
                f"$(env) in {where} expects 'VAR' or 'VAR default', "
                f"got: '{match.group(0)}'"
            )

        raise ProfileError(
            f"unsupported substitution '$({kind} ...)' in {where}; "
            "supported: find-pkg-share, env"
        )

    return _SUBSTITUTION.sub(replace, value)


def _to_argument_string(value, where):
    """Normalize a YAML scalar to the string expected by ROS launch."""
    if isinstance(value, bool):
        return 'true' if value else 'false'

    if isinstance(value, (int, float)):
        return str(value)

    if isinstance(value, str):
        return _expand_substitutions(value, where)

    raise ProfileError(
        f'{where} must be a scalar (string/bool/number), '
        f'got {type(value).__name__}'
    )


def _to_bool(value, where):
    """Convert a supported scalar value to a Python boolean."""
    text = _to_argument_string(value, where).strip().lower()

    if text in ('true', '1'):
        return True

    if text in ('false', '0'):
        return False

    raise ProfileError(
        f"{where} must be a boolean, got '{value}'"
    )


def _parse_arguments_block(block, where):
    """Parse a YAML launch-arguments mapping into string values."""
    if block is None:
        return {}

    if not isinstance(block, dict):
        raise ProfileError(
            f'{where} must be a mapping of argument: value'
        )

    return {
        str(key): _to_argument_string(
            value,
            f"{where} '{key}'"
        )
        for key, value in block.items()
    }
###########################################################################################################


# <!-- Profile loading -->
def _load_profile(path):
    """Load and validate a Waver bringup YAML profile."""
    try:
        with open(path, 'r') as stream:
            data = yaml.safe_load(stream)
    except yaml.YAMLError as error:
        raise ProfileError(
            f"profile '{path}' is not valid YAML: {error}"
        )

    if not isinstance(data, dict):
        raise ProfileError(
            f"profile '{path}' must be a YAML mapping with a 'stages' key"
        )

    unknown = set(data) - {'stages', 'arguments'}

    if unknown:
        raise ProfileError(
            f"profile '{path}' has unknown top-level keys "
            f"{sorted(unknown)}; expected only 'stages' and 'arguments'"
        )

    stages = data.get('stages')

    if not isinstance(stages, dict) or not stages:
        raise ProfileError(
            f"profile '{path}' must define a non-empty 'stages' mapping"
        )

    parsed = {}

    for name, stage in stages.items():
        where = f"stage '{name}' in profile '{path}'"

        if stage is None:
            stage = {}

        if not isinstance(stage, dict):
            raise ProfileError(
                f'{where} must be a mapping'
            )

        unknown = set(stage) - STAGE_KEYS

        if unknown:
            raise ProfileError(
                f'{where} has unknown keys {sorted(unknown)}; '
                f'allowed: {sorted(STAGE_KEYS)}'
            )

        for required in ('package', 'launch_file'):
            if not isinstance(stage.get(required), str) or not stage[required]:
                raise ProfileError(
                    f"{where} must define a non-empty string '{required}'"
                )

        parsed[str(name)] = {
            'enabled': _to_bool(
                stage.get('enabled', True),
                f"'enabled' of {where}"
            ),
            'package': stage['package'],
            'launch_file': stage['launch_file'],
            'arguments': _parse_arguments_block(
                stage.get('arguments'),
                f"'arguments' of {where}"
            )
        }

    return {
        'arguments': _parse_arguments_block(
            data.get('arguments'),
            f"'arguments' of profile '{path}'"
        ),
        'stages': parsed
    }
###########################################################################################################


# <!-- Stage launch resolution -->
def _resolve_stage_launch_file(package, launch_file, stage_name):
    """Resolve a stage launch file inside its package share directory."""
    try:
        share = get_package_share_directory(package)
    except PackageNotFoundError:
        raise ProfileError(
            f"stage '{stage_name}': package '{package}' was not found in "
            'the ament index. Is it built and sourced in this workspace?'
        )

    if os.path.isabs(launch_file):
        candidates = [launch_file]
    else:
        candidates = [
            os.path.join(share, launch_file),
            os.path.join(share, 'launch', launch_file)
        ]

    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate

    raise ProfileError(
        f"stage '{stage_name}': launch file '{launch_file}' not found in "
        f"package '{package}'. Tried: " + ', '.join(candidates)
    )
###########################################################################################################


# <!-- Command-line overrides -->
def _collect_overrides(launch_configurations, stage_names):
    """Collect ``<stage>.<field>`` command-line overrides by stage."""
    overrides = {
        name: {
            'arguments': {}
        }
        for name in stage_names
    }

    for key, value in launch_configurations.items():
        if '.' not in key:
            if key not in TOP_LEVEL_ARGUMENTS:
                raise ProfileError(
                    f"unknown launch argument '{key}'. Expected one of "
                    f'{list(TOP_LEVEL_ARGUMENTS)} or a '
                    "'<stage>.<field>' override"
                )

            continue

        stage, field = key.split('.', 1)

        if stage not in overrides:
            raise ProfileError(
                f"override '{key}' refers to unknown stage '{stage}'. "
                f'Stages in this profile: {sorted(stage_names)}'
            )

        if not field:
            raise ProfileError(
                f"override '{key}' is missing a field name"
            )

        where = f"override '{key}'"

        if field == 'enabled':
            overrides[stage]['enabled'] = _to_bool(
                value,
                where
            )
        elif field in STRUCTURAL_OVERRIDES:
            overrides[stage][field] = value
        else:
            overrides[stage]['arguments'][field] = _expand_substitutions(
                value,
                where
            )

    return overrides
###########################################################################################################


# <!-- Dynamic launch setup -->
def _launch_setup(context, *args, **kwargs):
    """Create launch actions for every enabled stage in the selected profile."""
    profile_name = context.launch_configurations['profile']
    profile_path = _resolve_profile_path(profile_name)
    profile = _load_profile(profile_path)
    overrides = _collect_overrides(
        context.launch_configurations,
        profile['stages'].keys()
    )

    forwarded = {}

    for name in FORWARDED_ARGUMENTS:
        value = context.launch_configurations.get(name, '')

        if name == 'namespace' or value != '':
            forwarded[name] = value

    actions = [
        LogInfo(
            msg=f'[waver_bringup] profile: {profile_path}'
        )
    ]

    for name, stage in profile['stages'].items():
        stage_overrides = overrides[name]
        enabled = stage_overrides.get(
            'enabled',
            stage['enabled']
        )

        if not enabled:
            actions.append(
                LogInfo(
                    msg=f'[waver_bringup] {name}: disabled'
                )
            )
            continue

        package = stage_overrides.get(
            'package',
            stage['package']
        )
        launch_file = stage_overrides.get(
            'launch_file',
            stage['launch_file']
        )
        launch_path = _resolve_stage_launch_file(
            package,
            launch_file,
            name
        )

        # <!-- Argument precedence -->
        # Profile globals < stage arguments < CLI globals < stage overrides.
        arguments = dict(profile['arguments'])
        arguments.update(stage['arguments'])
        arguments.update(forwarded)
        arguments.update(stage_overrides['arguments'])

        summary = ' '.join(
            f'{key}:={value}'
            for key, value in arguments.items()
        )

        actions.append(
            LogInfo(
                msg=(
                    f'[waver_bringup] {name}: '
                    f'{package}/{os.path.basename(launch_path)} {summary}'
                )
            )
        )

        actions.append(
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    launch_path
                ),
                launch_arguments=list(arguments.items())
            )
        )

    return actions
###########################################################################################################


def generate_launch_description():
    """
    Create the generic, profile-driven Waver bringup launch description.

    The launch description declares the profile and shared ROS configuration
    arguments. An opaque setup action then loads the selected YAML profile,
    validates its stages, applies global and per-stage command-line overrides,
    resolves launch files through the ament index, and includes every enabled
    stage in profile order.

    Launch arguments:
        profile (str): Profile name from ``waver_bringup/profiles`` or a path
            to a YAML profile file. Defaults to ``"sim_navigation"``.
        namespace (str): ROS namespace forwarded to every enabled stage.
            Defaults to an empty string.
        use_sim_time (str): Global ``use_sim_time`` override forwarded to every
            enabled stage. An empty value keeps profile values unchanged.
            Defaults to an empty string.
        map (str): Global map YAML path override forwarded to every enabled
            stage. An empty value keeps profile values unchanged. Defaults to
            an empty string.

    Stage overrides:
        <stage>.enabled: Enables or disables a profile stage.
        <stage>.package: Replaces the package that implements a stage.
        <stage>.launch_file: Replaces the launch file used by a stage.
        <stage>.<argument>: Replaces or adds an argument for one stage.

    Supported profile substitutions:
        $(find-pkg-share <package>): Resolves a package share directory.
        $(env <variable> [default]): Resolves an environment variable with an
            optional default value.

    Returns:
        LaunchDescription: Ordered declarations and dynamic stage setup for
        the selected Waver bringup profile.
    """

    ###########################################################################################################

    # <!-- Declare arguments -->
    declare_profile_cmd = DeclareLaunchArgument(
        'profile',
        default_value='sim_navigation',
        description=(
            'Profile name from waver_bringup/profiles or a path '
            'to a profile YAML file'
        )
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='ROS namespace forwarded to every stage'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='',
        description=(
            'Override use_sim_time for every stage '
            '(empty: keep profile values)'
        )
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description=(
            'Override the map yaml path forwarded to every stage '
            'that accepts a map argument (empty: keep profile values)'
        )
    )
    ###########################################################################################################

    # <!-- Dynamic profile launcher -->
    launch_setup_cmd = OpaqueFunction(
        function=_launch_setup
    )
    ###########################################################################################################

    return LaunchDescription(
        [
            declare_profile_cmd,
            declare_namespace_cmd,
            declare_use_sim_time_cmd,
            declare_map_cmd,
            launch_setup_cmd
        ]
    )
