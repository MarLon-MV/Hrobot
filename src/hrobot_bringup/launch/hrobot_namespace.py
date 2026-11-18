"""Gedeelde hulpfuncties voor de launch-bestanden van hrobot_bringup (afhandeling van de robot-namespace).

Dit is een gewone Python-module (geen launch-bestand): de launch-bestanden voegen hun eigen map toe aan
sys.path en importeren hem.

NAMESPACE-ONTWERP (één keer toepassen, geen /r2/r2/...):
  * Elk launch-bestand declareert `robot_namespace` en past de namespace ZELF toe, alleen op de
    nodes die het rechtstreeks aanmaakt (GroupAction + PushRosNamespace, zie `namespaced`).
  * mapping wikkelt zijn IncludeLaunchDescription-acties NIET in een groep: het geeft
    `robot_namespace` door aan hardware_base en slam, die de namespace zelf toepassen.
    De includes ook wikkelen zou de namespace tweemaal toepassen.
  * Lege namespace = helemaal geen PushRosNamespace en YAML-bestanden blijven ongewijzigd (simulatie).
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
    """Env CREATE3_ROBOT_NAMESPACE indien gedefinieerd (ook leeg = geen namespace), anders 'r2'."""
    return os.environ.get(ENV_VAR, FALLBACK_NAMESPACE)


def robot_namespace_argument():
    return DeclareLaunchArgument(
        "robot_namespace",
        default_value=default_robot_namespace(),
        description=f"Robot-namespace (env {ENV_VAR}, standaard '{FALLBACK_NAMESPACE}'); "
        "leeg = geen namespace. Moet overeenkomen met de namespace die in de Create 3 is ingesteld.",
    )


def resolve_namespace(context):
    """Geef de genormaliseerde namespace zonder slashes aan de uiteinden terug ('' = geen)."""
    raw = LaunchConfiguration("robot_namespace").perform(context).strip().strip("/")
    if raw and not _NAMESPACE_RE.match(raw):
        raise RuntimeError(
            f"robot_namespace:='{raw}' is ongeldig: gebruik letters, cijfers en '_' "
            "(beginnend met een letter), bijv. r2."
        )
    return raw


def namespaced(namespace, actions):
    """Groepeer `actions` onder `namespace` (PushRosNamespace); met '' worden ze ongewijzigd teruggegeven."""
    if not namespace:
        return list(actions)
    return [GroupAction([PushRosNamespace(namespace), *actions])]


def tf_remappings(namespace, absolute=False):
    """TF-remaps zodat nodes <ns>/tf en <ns>/tf_static gebruiken (de Create 3 publiceert ze daar).

    Relatieve doelen (patroon van nav2_bringup) worden onder de namespace van de node omgezet. `absolute=True`
    is voor nodes waarvan tf2_ros::TransformListener een interne node in de ROOT-namespace aanmaakt
    die alleen globale remaps ziet, zodat een relatief doel naar /tf zou verwijzen.
    """
    if not namespace:
        return []
    if absolute:
        return [("/tf", f"/{namespace}/tf"), ("/tf_static", f"/{namespace}/tf_static")]
    return [("/tf", "tf"), ("/tf_static", "tf_static")]


def rewrite_yaml(yaml_path, namespace):
    """YAML voor nodes onder `namespace`: RewrittenYaml(root_key=ns); '' = bestand ongewijzigd."""
    if not namespace:
        return yaml_path
    from nav2_common.launch import RewrittenYaml

    return RewrittenYaml(
        source_file=yaml_path, root_key=namespace, param_rewrites={}, convert_types=True
    )


def substitute_namespace_markers(source_file, namespace):
    """ReplaceString-substitutie die '/<robot_namespace>' omzet in '/<ns>' ('' zonder namespace).

    Gebruikt voor de RViz-configuratie ('/<robot_namespace>/scan' -> '/r2/scan'). Geeft ALTIJD een ReplaceString
    terug (ook met namespace '', zodat de markering '' wordt en '/<robot_namespace>/scan' -> '/scan', nooit '//scan').
    """
    from nav2_common.launch import ReplaceString

    return ReplaceString(
        source_file=source_file,
        replacements={"/<robot_namespace>": f"/{namespace}" if namespace else ""},
    )
