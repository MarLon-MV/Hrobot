#!/usr/bin/env bash
# Prepara la Raspberry Pi 5 para compilar y ejecutar este workspace.
#
# Requisito previo: Ubuntu con ROS 2 Jazzy ya instalado en /opt/ros/jazzy (este script NO instala ROS).
# Es idempotente: se puede volver a ejecutar sin romper nada.
#
# Qué hace:
#   1. Instala herramientas base (git, colcon, rosdep).
#   2. Inicializa/actualiza rosdep.
#   3. Clona (o actualiza) el driver del LD19 en src/ldlidar_stl_ros2 (rosdep no lo resuelve).
#   4. Permisos del puerto serie: grupo dialout + regla udev con symlink /dev/ldlidar.
#   5. Instala dependencias con rosdep y compila con colcon.
#
# Uso:  scripts/setup_pi.sh [--skip-udev] [--skip-build]
# Variables opcionales:
#   LDLIDAR_REPO   repositorio del driver   (def: repo oficial de LDRobot)
#   LDLIDAR_REF    rama, tag o commit       (def: master; fíjalo a un commit cuando el build funcione)
#   ROS_DISTRO_NAME  distribución de ROS    (def: jazzy)
#   JOBS           workers de colcon        (def: 2, para no agotar la RAM de la Pi)

set -euo pipefail

LDLIDAR_REPO="${LDLIDAR_REPO:-https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git}"
LDLIDAR_REF="${LDLIDAR_REF:-master}"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
JOBS="${JOBS:-2}"
UDEV_RULE="/etc/udev/rules.d/99-ldlidar.rules"
# CP2102 (USB-serie habitual del LD19). Verifícalo con `lsusb` y ajusta si tu adaptador es otro.
UDEV_CONTENT='SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ldlidar", GROUP="dialout", MODE="0660"'

SKIP_UDEV=0
SKIP_BUILD=0
for arg in "$@"; do
  case "$arg" in
    --skip-udev) SKIP_UDEV=1 ;;
    --skip-build) SKIP_BUILD=1 ;;
    -h|--help) sed -n '2,22p' "$0"; exit 0 ;;
    *) echo "Argumento desconocido: $arg (usa --help)" >&2; exit 2 ;;
  esac
done

log() { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
die() { printf '\033[1;31mERROR: %s\033[0m\n' "$*" >&2; exit 1; }

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

[ "$(id -u)" -ne 0 ] || die "No lo ejecutes como root; usa tu usuario (el script llama a sudo cuando hace falta)."
[ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ] || die "No se encuentra /opt/ros/${ROS_DISTRO_NAME}/setup.bash. Instala ROS 2 ${ROS_DISTRO_NAME} primero."
[ -d src ] || die "No se encuentra el directorio src/ en ${REPO_ROOT}."

log "Instalando herramientas base (apt)"
sudo apt-get update
sudo apt-get install -y git python3-colcon-common-extensions python3-rosdep

log "Inicializando rosdep"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update --rosdistro "${ROS_DISTRO_NAME}"

log "Driver del LD19 (${LDLIDAR_REPO} @ ${LDLIDAR_REF})"
LD_DIR="src/ldlidar_stl_ros2"
if [ -d "${LD_DIR}/.git" ]; then
  git -C "${LD_DIR}" fetch --tags origin
else
  git clone "${LDLIDAR_REPO}" "${LD_DIR}"
fi
git -C "${LD_DIR}" checkout "${LDLIDAR_REF}"
# Si LDLIDAR_REF es una rama, avanzar a su última versión (sin merges); si es tag/commit no hace nada.
if git -C "${LD_DIR}" symbolic-ref -q HEAD >/dev/null; then
  git -C "${LD_DIR}" merge --ff-only "origin/${LDLIDAR_REF}"
fi
echo "Driver en commit: $(git -C "${LD_DIR}" rev-parse --short HEAD)"

if [ "${SKIP_UDEV}" -eq 0 ]; then
  log "Permisos del puerto serie (dialout + udev /dev/ldlidar)"
  if ! id -nG "${USER}" | tr ' ' '\n' | grep -qx dialout; then
    sudo usermod -aG dialout "${USER}"
    echo "Añadido ${USER} al grupo dialout: cierra sesión y vuelve a entrar para que surta efecto."
  fi
  if [ ! -f "${UDEV_RULE}" ] || [ "$(cat "${UDEV_RULE}")" != "${UDEV_CONTENT}" ]; then
    printf '%s\n' "${UDEV_CONTENT}" | sudo tee "${UDEV_RULE}" >/dev/null
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "Regla udev instalada en ${UDEV_RULE}. Reconecta el LiDAR; debería aparecer /dev/ldlidar."
  fi
else
  log "Omitiendo permisos udev (--skip-udev)"
fi

log "Instalando dependencias con rosdep"
# setup.bash de ROS usa variables sin definir: desactivar -u mientras se carga.
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
set -u
rosdep install --from-paths src --ignore-src -y --rosdistro "${ROS_DISTRO_NAME}"

if [ "${SKIP_BUILD}" -eq 0 ]; then
  log "Compilando con colcon (workers: ${JOBS})"
  colcon build --symlink-install --parallel-workers "${JOBS}"
else
  log "Omitiendo compilación (--skip-build)"
fi

log "Listo"
cat <<EOF
Siguientes pasos:
  source ${REPO_ROOT}/install/setup.bash
  ls -l /dev/ldlidar            # o /dev/ttyUSB0 si no usas la regla udev
  ros2 launch create3_bot_bringup master_explore.launch.py lidar_port:=/dev/ldlidar
Comprueba que ves los topics del Create 3 (ros2 topic list) y el QoS de /scan (ros2 topic info /scan -v).
EOF
