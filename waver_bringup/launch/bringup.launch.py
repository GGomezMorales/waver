import os

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError
)
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction
)
from launch.events import Shutdown
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.logging import get_logger

from waver_bringup.bringup_logic import (
    BuildStagePlans,
    CollectOverrides,
    LoadProfile,
    ResolveProfilePath
)


###############################################################################################

def FindPackageShare(package: str) -> tuple[str | None, str]:
    """
    Resolve a package share directory through the ROS 2 ament index.
    """
    try:
        return get_package_share_directory(package), ''
    except PackageNotFoundError:
        return None, (
            f"package '{package}' was not found in the ament index. "
            'Is it built and sourced?'
        )


def ResolveStageLaunchFile(
    package: str,
    launch_file: str,
    stage_name: str
) -> tuple[str | None, str]:
    """
    Resolve a stage launch file from an absolute or package-relative path.
    """
    share: str | None
    error: str
    share, error = FindPackageShare(package)

    if error:
        return None, (
            f"stage '{stage_name}': package '{package}' was not found in "
            'the ament index. Is it built and sourced in this workspace?'
        )

    if os.path.isabs(launch_file):
        candidates: list[str] = [launch_file]
    else:
        candidates = [
            os.path.join(share, launch_file),
            os.path.join(share, 'launch', launch_file)
        ]

    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate, ''

    return None, (
        f"stage '{stage_name}': launch file '{launch_file}' not found in "
        f"package '{package}'. Tried: " + ', '.join(candidates)
    )


def ErrorActions(message: str) -> list[object]:
    """
    Report a ROS 2 launch error and return an orderly shutdown action.
    """
    full_message: str = '[waver_bringup] ' + message
    get_logger('waver_bringup').error(full_message)

    return [
        EmitEvent(event=Shutdown(reason=full_message))
    ]


def LaunchSetup(
    context: LaunchContext,
    *args: object,
    **kwargs: object
) -> list[object]:
    """
    Convert a validated profile plan into ROS 2 launch actions.

    Profile interpretation is delegated to ``bringup_logic``.  This adapter
    supplies ament package lookup, translates failures into launch events, and
    creates an include action for every enabled stage in profile order.
    """
    bringup_share: str | None
    error: str
    bringup_share, error = FindPackageShare('waver_bringup')

    if error:
        return ErrorActions(error)

    profile_name: str = context.launch_configurations['profile']
    profile_path: str | None
    profile_path, error = ResolveProfilePath(
        profile_name,
        os.path.join(bringup_share, 'profiles')
    )

    if error:
        return ErrorActions(error)

    profile: dict[str, object] | None
    profile, error = LoadProfile(profile_path, FindPackageShare)

    if error:
        return ErrorActions(error)

    overrides: dict[str, dict[str, object]] | None
    overrides, error = CollectOverrides(
        context.launch_configurations,
        list(profile['stages']),
        FindPackageShare
    )

    if error:
        return ErrorActions(error)

    plans: list[dict[str, object]] = BuildStagePlans(
        profile,
        overrides
    )

    for plan in plans:
        if not plan['enabled']:
            continue

        launch_path: str | None
        launch_path, error = ResolveStageLaunchFile(
            plan['package'],
            plan['launch_file'],
            plan['name']
        )

        if error:
            return ErrorActions(error)

        plan['launch_path'] = launch_path

    actions: list[object] = [
        LogInfo(
            msg=f'[waver_bringup] profile: {profile_path}'
        )
    ]

    for plan in plans:
        name: str = plan['name']

        if not plan['enabled']:
            actions.append(
                LogInfo(
                    msg=f'[waver_bringup] {name}: disabled'
                )
            )
            continue

        summary: str = ' '.join(
            f'{key}:={value}'
            for key, value in plan['arguments'].items()
        )

        actions.append(
            LogInfo(
                msg=(
                    f'[waver_bringup] {name}: '
                    f"{plan['package']}/"
                    f"{os.path.basename(plan['launch_path'])} "
                    f'{summary}'
                )
            )
        )

        actions.append(
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    plan['launch_path']
                ),
                launch_arguments=list(plan['arguments'].items())
            )
        )

    return actions
###############################################################################################


def generate_launch_description() -> LaunchDescription:
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
    ###############################################################################################

    # <!-- Declare arguments -->
    declare_profile_cmd: DeclareLaunchArgument = DeclareLaunchArgument(
        'profile',
        default_value='sim_navigation',
        description=(
            'Profile name from waver_bringup/profiles or a path '
            'to a profile YAML file'
        )
    )

    ###############################################################################################

    # <!-- Dynamic profile launcher -->
    launch_setup_cmd: OpaqueFunction = OpaqueFunction(
        function=LaunchSetup
    )
    ###############################################################################################

    return LaunchDescription(
        [
            declare_profile_cmd,
            launch_setup_cmd
        ]
    )
