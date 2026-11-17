#!/usr/bin/env bash
# Abre RViz2 en OTRO equipo (p. ej. un portátil Ubuntu con ROS 2 Jazzy) conectado a la misma red que el
# robot, con la configuración de mapeo y el namespace del robot ya aplicados.
#
# Uso:  scripts/rviz_remote.sh [-n|--namespace NOMBRE] [--dry-run] [-h]
#   -n NOMBRE   namespace del robot (por defecto: $CREATE3_ROBOT_NAMESPACE o "r2"; "" = sin namespace)
#   --dry-run   muestra lo que haría (config generada, topics, comando) sin lanzar RViz
#
# Requisitos en este equipo: ROS 2 Jazzy con rviz2 instalado, el MISMO ROS_DOMAIN_ID y el mismo RMW que
# el robot, la hora sincronizada con la Pi (NTP/chrony: un desfase rompe el TF en RViz) y red sin
# bloqueo de multicast/firewall.

set -euo pipefail

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RVIZ_SRC="${REPO_ROOT}/src/create3_bot_description/rviz/mapping.rviz"
NS="${CREATE3_ROBOT_NAMESPACE-r2}"
DRY_RUN=0

die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

while [ $# -gt 0 ]; do
  case "$1" in
    -n|--namespace)
      [ $# -ge 2 ] || die "$1 necesita un valor (usa \"\" para sin namespace)"
      NS="$2"; shift 2 ;;
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
    *) die "Argumento desconocido: $1 (usa --help)" ;;
  esac
done

[[ "${NS}" =~ ^([a-z][a-z0-9_]{0,31})?$ ]] || die "Namespace inválido: '${NS}' (esperado ^[a-z][a-z0-9_]{0,31}$ o vacío)"
[ -f "${RVIZ_SRC}" ] || die "No se encuentra ${RVIZ_SRC} (¿ejecutas el script desde un clon completo del repo?)"

# '/<robot_namespace>' -> '/<ns>' (o '' sin namespace), igual que hace slam.launch.py.
if [ -n "${NS}" ]; then PREFIX="/${NS}"; else PREFIX=""; fi

TMP_CONFIG="$(mktemp "${TMPDIR:-/tmp}/mapping_rviz.XXXXXX")"
trap 'rm -f "${TMP_CONFIG}"' EXIT
sed "s#/<robot_namespace>#${PREFIX}#g" "${RVIZ_SRC}" > "${TMP_CONFIG}"
if grep -q '<robot_namespace>' "${TMP_CONFIG}"; then die "Quedan marcadores <robot_namespace> sin sustituir"; fi

REMAPS=()
if [ -n "${NS}" ]; then
  REMAPS=(--ros-args -r "/tf:=/${NS}/tf" -r "/tf_static:=/${NS}/tf_static")
fi
CMD=(rviz2 -d "${TMP_CONFIG}")
# Expansión segura de un array posiblemente vacío (bash < 4.4 con set -u).
if [ "${#REMAPS[@]}" -gt 0 ]; then CMD+=("${REMAPS[@]}"); fi

echo "Namespace : '${NS}'"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<sin definir: 0>}  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<por defecto>}"
echo "Topics en la config:"
grep -E 'Value: /' "${TMP_CONFIG}" | sed 's/^ *Value: /  /' | sort -u

if [ "${DRY_RUN}" -eq 1 ]; then
  echo "Comando: ${CMD[*]}"
  exit 0
fi

[ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ] || die "No se encuentra /opt/ros/${ROS_DISTRO_NAME}/setup.bash"
# setup.bash de ROS usa variables sin definir: desactivar -u mientras se carga.
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
set -u
command -v rviz2 >/dev/null || die "rviz2 no está instalado (sudo apt install ros-${ROS_DISTRO_NAME}-rviz2)"

echo "Abriendo RViz..."
"${CMD[@]}"
