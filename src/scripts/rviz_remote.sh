#!/usr/bin/env bash
# Abre RViz en esta máquina (laptop) para ver el robot real que corre en la Pi.
#
# En la Pi lanza antes:  ros2 launch hrobot_bringup mapping.launch.py lidar_port:=/dev/ldlidar rviz:=false
#
# Uso:  src/scripts/rviz_remote.sh [mapping|navigation] [--namespace NS] [--domain ID] [--rmw NAME]
#   mapping|navigation  config de RViz a usar (def: mapping)
#   --namespace         namespace del robot (def: $CREATE3_ROBOT_NAMESPACE si existe, si no r2; vacío = ninguno)
#   --domain            ROS_DOMAIN_ID (def: el que ya tengas exportado; debe coincidir con el de la Pi)
#   --rmw               RMW_IMPLEMENTATION (def: el que ya tengas exportado; debe coincidir con el de la Pi)

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
    --namespace) [ $# -gt 0 ] || die "--namespace necesita un valor (usa '' para ninguno)"; NAMESPACE="$1"; shift ;;
    --domain) [ $# -gt 0 ] || die "--domain necesita un valor"; export ROS_DOMAIN_ID="$1"; shift ;;
    --rmw) [ $# -gt 0 ] || die "--rmw necesita un valor"; export RMW_IMPLEMENTATION="$1"; shift ;;
    -h|--help) sed -n '2,10p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) die "argumento desconocido: ${arg}" ;;
  esac
done

# set -u rompe con los setup.bash de ROS; se desactiva solo mientras se cargan.
set +u
[ -f /opt/ros/jazzy/setup.bash ] || die "no encuentro /opt/ros/jazzy/setup.bash"
source /opt/ros/jazzy/setup.bash
[ -f "${WS_ROOT}/install/setup.bash" ] && source "${WS_ROOT}/install/setup.bash"
set -u

command -v rviz2 >/dev/null || die "rviz2 no está instalado (sudo apt install ros-jazzy-rviz2)"

SRC_RVIZ="${WS_ROOT}/src/hrobot_description/rviz/${CONFIG}.rviz"
[ -f "${SRC_RVIZ}" ] || die "no existe ${SRC_RVIZ}"

# El .rviz usa el marcador /<robot_namespace>; el launch lo sustituye, aquí lo hacemos con sed.
TMP_RVIZ="$(mktemp --suffix=.rviz)"
trap 'rm -f "${TMP_RVIZ}"' EXIT
if [ -n "${NAMESPACE}" ]; then
  sed "s#/<robot_namespace>#/${NAMESPACE}#g" "${SRC_RVIZ}" > "${TMP_RVIZ}"
  REMAPS=(-r "/tf:=/${NAMESPACE}/tf" -r "/tf_static:=/${NAMESPACE}/tf_static")
else
  sed "s#/<robot_namespace>##g" "${SRC_RVIZ}" > "${TMP_RVIZ}"
  REMAPS=()
fi

echo "RViz: config=${CONFIG} namespace='${NAMESPACE}' ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} RMW=${RMW_IMPLEMENTATION:-default}"
rviz2 -d "${TMP_RVIZ}" --ros-args "${REMAPS[@]}"
