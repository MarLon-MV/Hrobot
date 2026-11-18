#!/usr/bin/env bash
# Opent RViz op deze machine (laptop) om de echte robot te bekijken die op de Pi draait.
#
# Start eerst op de Pi:  ros2 launch hrobot_bringup mapping.launch.py lidar_port:=/dev/rplidar rviz:=false
#
# Uso:  src/scripts/rviz_remote.sh [mapping|navigation] [--namespace NS] [--domain ID] [--rmw NAME]
#   mapping|navigation  te gebruiken RViz-configuratie (standaard: mapping)
#   --namespace         namespace van de robot (standaard: $CREATE3_ROBOT_NAMESPACE indien gezet, anders r2; leeg = geen)
#   --domain            ROS_DOMAIN_ID (standaard: de al geëxporteerde waarde; moet overeenkomen met die van de Pi)
#   --rmw               RMW_IMPLEMENTATION (standaard: de al geëxporteerde waarde; moet overeenkomen met die van de Pi)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
NAMESPACE="${CREATE3_ROBOT_NAMESPACE-r2}"
CONFIG="mapping"

die() { echo "ERROR: $*" >&2; exit 1; }

while [ $# -gt 0 ]; do
  arg="$1"; shift
  case "$arg" in
    mapping|navigation) CONFIG="${arg}" ;;
    --namespace) [ $# -gt 0 ] || die "--namespace heeft een waarde nodig (gebruik '' voor geen)"; NAMESPACE="$1"; shift ;;
    --domain) [ $# -gt 0 ] || die "--domain heeft een waarde nodig"; export ROS_DOMAIN_ID="$1"; shift ;;
    --rmw) [ $# -gt 0 ] || die "--rmw heeft een waarde nodig"; export RMW_IMPLEMENTATION="$1"; shift ;;
    -h|--help) sed -n '2,10p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) die "onbekend argument: ${arg}" ;;
  esac
done

# set -u gaat stuk op de setup.bash van ROS; wordt alleen tijdens het laden uitgeschakeld.
set +u
[ -f /opt/ros/jazzy/setup.bash ] || die "kan /opt/ros/jazzy/setup.bash niet vinden"
source /opt/ros/jazzy/setup.bash
[ -f "${WS_ROOT}/install/setup.bash" ] && source "${WS_ROOT}/install/setup.bash"
set -u

command -v rviz2 >/dev/null || die "rviz2 is niet geïnstalleerd (sudo apt install ros-jazzy-rviz2)"

SRC_RVIZ="${WS_ROOT}/src/hrobot_description/rviz/${CONFIG}.rviz"
[ -f "${SRC_RVIZ}" ] || die "${SRC_RVIZ} bestaat niet"

# De .rviz gebruikt de markering /<robot_namespace>; de launch vervangt die, hier doen we het met sed.
TMP_RVIZ="$(mktemp --suffix=.rviz)"
trap 'rm -f "${TMP_RVIZ}"' EXIT
if [ -n "${NAMESPACE}" ]; then
  sed "s#/<robot_namespace>#/${NAMESPACE}#g" "${SRC_RVIZ}" > "${TMP_RVIZ}"
  REMAPS=(-r "/tf:=/${NAMESPACE}/tf" -r "/tf_static:=/${NAMESPACE}/tf_static")
else
  sed "s#/<robot_namespace>##g" "${SRC_RVIZ}" > "${TMP_RVIZ}"
  REMAPS=()
fi

echo "RViz: configuratie=${CONFIG} namespace='${NAMESPACE}' ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} RMW=${RMW_IMPLEMENTATION:-default}"
rviz2 -d "${TMP_RVIZ}" --ros-args "${REMAPS[@]}"
