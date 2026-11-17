"""Shared helpers for the hrobot_bringup launch files (robot namespace handling).

This is a plain Python module (not a launch file): the launch files add their own directory to
sys.path and import it.

NAMESPACE DESIGN (single application, no /r2/r2/...):
  * Every launch file declares `robot_namespace` and applies the namespace ITSELF, only to the
    nodes it creates directly (GroupAction + PushRosNamespace, see `namespaced`).
  * mapping does NOT wrap its IncludeLaunchDescription actions in a group: it forwards
    `robot_namespace` to hardware_base and slam, which push the namespace on their own.
    Wrapping the includes too would apply the namespace twice.
  * Empty namespace = no PushRosNamespace at all and YAMLs are used untouched (simulation).
"""

import os
import re

from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

ENV_VAR = "CREATE3_ROBOT_NAMESPACE"
FALLBACK_NAMESPACE = "r2"
_NAMESPACE_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)*$")


def default_robot_namespace():
    """Env CREATE3_ROBOT_NAMESPACE if defined (even empty = no namespace), else 'r2'."""
    return os.environ.get(ENV_VAR, FALLBACK_NAMESPACE)


def robot_namespace_argument():
    return DeclareLaunchArgument(
        "robot_namespace",
        default_value=default_robot_namespace(),
        description=f"Robot namespace (env {ENV_VAR}, default '{FALLBACK_NAMESPACE}'); "
        "empty = no namespace. Must match the namespace configured in the Create 3.",
    )


def resolve_namespace(context):
    """Return the normalized namespace without slashes at the ends ('' = none)."""
    raw = LaunchConfiguration("robot_namespace").perform(context).strip().strip("/")
    if raw and not _NAMESPACE_RE.match(raw):
        raise RuntimeError(
            f"robot_namespace:='{raw}' is invalid: use letters, digits and '_' "
            "(starting with a letter), e.g. r2."
        )
    return raw


def namespaced(namespace, actions):
    """Group `actions` under `namespace` (PushRosNamespace); with '' return them unchanged."""
    if not namespace:
        return list(actions)
    return [GroupAction([PushRosNamespace(namespace), *actions])]


def tf_remappings(namespace, absolute=False):
    """TF remaps so nodes use <ns>/tf and <ns>/tf_static (the Create 3 publishes them there).

    Relative targets (nav2_bringup pattern) resolve under the node namespace. `absolute=True`
    is for nodes whose tf2_ros::TransformListener creates an internal node in the ROOT
    namespace that only sees global remaps, so a relative target would resolve to /tf.
    """
    if not namespace:
        return []
    if absolute:
        return [("/tf", f"/{namespace}/tf"), ("/tf_static", f"/{namespace}/tf_static")]
    return [("/tf", "tf"), ("/tf_static", "tf_static")]


def rewrite_yaml(yaml_path, namespace):
    """YAML for nodes under `namespace`: RewrittenYaml(root_key=ns); '' = file unchanged."""
    if not namespace:
        return yaml_path
    from nav2_common.launch import RewrittenYaml

    return RewrittenYaml(
        source_file=yaml_path, root_key=namespace, param_rewrites={}, convert_types=True
    )


def substitute_namespace_markers(source_file, namespace):
    """ReplaceString substitution that turns '/<robot_namespace>' into '/<ns>' ('' if no namespace).

    Used for the RViz config ('/<robot_namespace>/scan' -> '/r2/scan'). It ALWAYS returns a ReplaceString
    (also with namespace '', so the marker becomes '' and '/<robot_namespace>/scan' -> '/scan', never '//scan').
    """
    from nav2_common.launch import ReplaceString

    return ReplaceString(
        source_file=source_file,
        replacements={"/<robot_namespace>": f"/{namespace}" if namespace else ""},
    )
