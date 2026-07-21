import os
import re

import yaml


TOP_LEVEL_ARGUMENTS: tuple[str, ...] = (
    'profile',
)

STAGE_KEYS: set[str] = {
    'enabled',
    'package',
    'launch_file',
    'arguments'
}

STRUCTURAL_OVERRIDES: set[str] = {
    'enabled',
    'package',
    'launch_file'
}

SUBSTITUTION: str = r'\$\(([^()]*)\)'


def AvailableProfiles(profiles_directory: str) -> list[str]:
    """
    Return the sorted profile names found in `profiles_directory`.

    Both `.yaml` `and` `.yml` files are recognized.  A missing directory is
    treated as an empty profile collection so callers can include the result
    in a useful validation message.
    """
    if not os.path.isdir(profiles_directory):
        return []

    return sorted(
        os.path.splitext(entry)[0]
        for entry in os.listdir(profiles_directory)
        if entry.endswith(('.yaml', '.yml'))
        and os.path.isfile(os.path.join(profiles_directory, entry))
    )


def ResolveProfilePath(
    profile: str,
    profiles_directory: str
) -> tuple[str | None, str]:
    """
    Resolve a profile name or YAML path without using ROS facilities.

    A value containing a path separator, or ending in a YAML extension, is
    interpreted as a file path.  Other values are looked up by name in the
    supplied profiles directory.
    """
    if os.sep in profile or profile.endswith(('.yaml', '.yml')):
        path: str = os.path.expanduser(profile)

        if not os.path.isfile(path):
            return None, f"profile file '{path}' does not exist"

        return path, ''

    path = os.path.join(profiles_directory, profile + '.yaml')

    if not os.path.isfile(path):
        available: str = ', '.join(AvailableProfiles(profiles_directory))
        return None, (
            f"unknown profile '{profile}'. Available profiles: {available}"
        )

    return path, ''


def ExpandSubstitutions(
    value: str,
    where: str,
    find_package_share: object,
    environment: dict[str, str] | None = None
) -> tuple[str | None, str]:
    """
    Expand supported substitutions in a profile or command-line value.

    `find_package_share` is a callback supplied by the integration layer. It
    accepts a package name and returns `(path, error)`.  Supplying that small
    callback keeps this module independent from the ament index and ROS 2.

    The supported forms are `$(find-pkg-share PACKAGE)`, `$(env VARIABLE)`,
    and `$(env VARIABLE DEFAULT)`.  The optional `environment` mapping is
    primarily useful to callers that need deterministic evaluation.
    """
    source_environment: dict[str, str] = dict(
        os.environ if environment is None else environment
    )
    pieces: list[str] = []
    cursor: int = 0

    if not callable(find_package_share):
        return None, 'package-share resolver must be callable'

    for match in re.finditer(SUBSTITUTION, value):
        pieces.append(value[cursor:match.start()])
        parts: list[str] = match.group(1).split()

        if not parts:
            return None, f'empty $() substitution in {where}'

        kind: str = parts[0]
        arguments: list[str] = parts[1:]

        if kind == 'find-pkg-share':
            if len(arguments) != 1:
                return None, (
                    f'$(find-pkg-share) in {where} expects exactly one '
                    f"package name, got: '{match.group(0)}'"
                )

            replacement: str | None
            error: str
            replacement, error = find_package_share(arguments[0])

            if error:
                return None, (
                    f"package '{arguments[0]}' referenced in {where} was not "
                    'found in the ament index. Is it built and sourced?'
                )
        elif kind == 'env':
            if len(arguments) == 1:
                if arguments[0] not in source_environment:
                    return None, (
                        f"environment variable '{arguments[0]}' referenced "
                        f'in {where} is not set and has no default'
                    )

                replacement = source_environment[arguments[0]]
            elif len(arguments) == 2:
                replacement = source_environment.get(
                    arguments[0],
                    arguments[1]
                )
            else:
                return None, (
                    f"$(env) in {where} expects 'VAR' or 'VAR default', "
                    f"got: '{match.group(0)}'"
                )
        else:
            return None, (
                f"unsupported substitution '$({kind} ...)' in {where}; "
                'supported: find-pkg-share, env'
            )

        pieces.append(replacement)
        cursor = match.end()

    pieces.append(value[cursor:])
    return ''.join(pieces), ''


def ToArgumentString(
    value: object,
    where: str,
    find_package_share: object
) -> tuple[str | None, str]:
    """
    Normalize a YAML scalar to the string expected by ROS launch.
    """
    if isinstance(value, bool):
        return ('true' if value else 'false'), ''

    if isinstance(value, (int, float)):
        return str(value), ''

    if isinstance(value, str):
        return ExpandSubstitutions(value, where, find_package_share)

    return None, (
        f'{where} must be a scalar (string/bool/number), '
        f'got {type(value).__name__}'
    )


def ToBool(
    value: object,
    where: str,
    find_package_share: object
) -> tuple[bool | None, str]:
    """
    Convert a supported scalar value to a Python boolean.
    """
    text: str | None
    error: str
    text, error = ToArgumentString(value, where, find_package_share)

    if error:
        return None, error

    normalized: str = text.strip().lower()

    if normalized in ('true', '1'):
        return True, ''

    if normalized in ('false', '0'):
        return False, ''

    return None, f"{where} must be a boolean, got '{value}'"


def ParseArgumentsBlock(
    block: object,
    where: str,
    find_package_share: object
) -> tuple[dict[str, str] | None, str]:
    """
    Parse a YAML launch-arguments mapping into string values.
    """
    if block is None:
        return {}, ''

    if not isinstance(block, dict):
        return None, f'{where} must be a mapping of argument: value'

    parsed: dict[str, str] = {}

    for key, value in block.items():
        parsed_value: str | None
        error: str
        parsed_value, error = ToArgumentString(
            value,
            f"{where} '{key}'",
            find_package_share
        )

        if error:
            return None, error

        parsed[str(key)] = parsed_value

    return parsed, ''


def LoadProfile(
    path: str,
    find_package_share: object
) -> tuple[dict[str, object] | None, str]:
    """
    Load and validate a Waver bringup YAML profile.

    Validation covers the document shape, supported keys, required stage
    fields, scalar launch arguments, boolean values, and substitutions.  The
    returned mapping is normalized and ready for override processing.
    """
    try:
        with open(path, 'r', encoding='utf-8') as stream:
            data: object = yaml.safe_load(stream)
    except OSError as error:
        return None, f"profile '{path}' could not be read: {error}"
    except yaml.YAMLError as error:
        return None, f"profile '{path}' is not valid YAML: {error}"

    if not isinstance(data, dict):
        return None, (
            f"profile '{path}' must be a YAML mapping with a 'stages' key"
        )

    unknown: set[str] = set(data) - {'stages', 'arguments'}

    if unknown:
        return None, (
            f"profile '{path}' has unknown top-level keys {sorted(unknown)}; "
            "expected only 'stages' and 'arguments'"
        )

    stages: object = data.get('stages')

    if not isinstance(stages, dict) or not stages:
        return None, (
            f"profile '{path}' must define a non-empty 'stages' mapping"
        )

    parsed_stages: dict[str, dict[str, object]] = {}

    for name, stage in stages.items():
        where: str = f"stage '{name}' in profile '{path}'"
        stage = {} if stage is None else stage

        if not isinstance(stage, dict):
            return None, f'{where} must be a mapping'

        unknown = set(stage) - STAGE_KEYS

        if unknown:
            return None, (
                f'{where} has unknown keys {sorted(unknown)}; '
                f'allowed: {sorted(STAGE_KEYS)}'
            )

        for required in ('package', 'launch_file'):
            if not isinstance(stage.get(required), str) or not stage[required]:
                return None, (
                    f"{where} must define a non-empty string '{required}'"
                )

        enabled: bool | None
        error: str
        enabled, error = ToBool(
            stage.get('enabled', True),
            f"'enabled' of {where}",
            find_package_share
        )

        if error:
            return None, error

        arguments: dict[str, str] | None
        arguments, error = ParseArgumentsBlock(
            stage.get('arguments'),
            f"'arguments' of {where}",
            find_package_share
        )

        if error:
            return None, error

        parsed_stages[str(name)] = {
            'enabled': enabled,
            'package': stage['package'],
            'launch_file': stage['launch_file'],
            'arguments': arguments
        }

    arguments, error = ParseArgumentsBlock(
        data.get('arguments'),
        f"'arguments' of profile '{path}'",
        find_package_share
    )

    if error:
        return None, error

    return {
        'arguments': arguments,
        'stages': parsed_stages
    }, ''


def CollectOverrides(
    launch_configurations: dict[str, str],
    stage_names: list[str],
    find_package_share: object
) -> tuple[dict[str, dict[str, object]] | None, str]:
    """
    Validate and collect command-line overrides by profile stage.

    Keys without a dot must be declared global arguments.  Dotted keys select
    a stage and then either one of its structural fields or a launch argument
    forwarded only to that stage.
    """
    overrides: dict[str, dict[str, object]] = {
        name: {'arguments': {}}
        for name in stage_names
    }

    for key, value in launch_configurations.items():
        if '.' not in key:
            if key not in TOP_LEVEL_ARGUMENTS:
                return None, (
                    f"unknown launch argument '{key}'. Expected one of "
                    f'{list(TOP_LEVEL_ARGUMENTS)} or a '
                    "'<stage>.<field>' override"
                )

            continue

        stage, field = key.split('.', 1)

        if stage not in overrides:
            return None, (
                f"override '{key}' refers to unknown stage '{stage}'. "
                f'Stages in this profile: {sorted(stage_names)}'
            )

        if not field:
            return None, f"override '{key}' is missing a field name"

        where = f"override '{key}'"

        if field == 'enabled':
            parsed_value: object
            error: str
            parsed_value, error = ToBool(
                value,
                where,
                find_package_share
            )
        elif field in STRUCTURAL_OVERRIDES:
            parsed_value, error = ToArgumentString(
                value,
                where,
                find_package_share
            )
        else:
            parsed_value, error = ExpandSubstitutions(
                value,
                where,
                find_package_share
            )

        if error:
            return None, error

        if field in STRUCTURAL_OVERRIDES:
            overrides[stage][field] = parsed_value
        else:
            overrides[stage]['arguments'][field] = parsed_value

    return overrides, ''


def BuildStagePlans(
    profile: dict[str, object],
    overrides: dict[str, dict[str, object]]
) -> list[dict[str, object]]:
    """
    Merge profile data and overrides into ordered stage plans.

    Argument precedence is profile globals, stage arguments, then
    stage-specific command-line arguments. Disabled stages remain in the
    result so the ROS layer can report them consistently.
    """
    plans: list[dict[str, object]] = []

    for name, stage in profile['stages'].items():
        stage_overrides: dict[str, object] = overrides[name]
        arguments: dict[str, str] = dict(profile['arguments'])
        arguments.update(stage['arguments'])
        arguments.update(stage_overrides['arguments'])

        plans.append({
            'name': name,
            'enabled': stage_overrides.get('enabled', stage['enabled']),
            'package': stage_overrides.get('package', stage['package']),
            'launch_file': stage_overrides.get(
                'launch_file',
                stage['launch_file']
            ),
            'arguments': arguments
        })

    return plans
