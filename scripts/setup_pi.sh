#!/usr/bin/env bash
# Prepara la Raspberry Pi 5 para compilar y ejecutar este workspace.
#
# Requisito previo: Ubuntu con ROS 2 Jazzy ya instalado en /opt/ros/jazzy (este script NO instala ROS).
# Es idempotente: se puede volver a ejecutar sin romper nada.
#
# Qué hace:
#   1. Instala herramientas base (git, colcon, rosdep).
#   2. Inicializa/actualiza rosdep.
#   3. Driver del LD19 (rosdep no lo resuelve): si ya hay un `ldlidar_stl_ros2` disponible en el entorno
#      (otro workspace ya compilado y cargado), lo reutiliza; si no, lo clona en src/ldlidar_stl_ros2.
#   4. Permisos del puerto serie: grupo dialout + regla udev con symlink /dev/ldlidar.
#   5. Instala dependencias con rosdep y compila con colcon.
#
# Uso:  scripts/setup_pi.sh [--skip-udev] [--skip-build] [--force-driver]
#   --force-driver  clona y compila el driver aquí aunque ya exista uno en el entorno
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
FORCE_DRIVER=0
for arg in "$@"; do
  case "$arg" in
    --skip-udev) SKIP_UDEV=1 ;;
    --skip-build) SKIP_BUILD=1 ;;
    --force-driver) FORCE_DRIVER=1 ;;
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

# setup.bash de ROS usa variables sin definir: desactivar -u mientras se carga.
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
set -u

LD_DIR="src/ldlidar_stl_ros2"
USE_EXISTING_DRIVER=0
if [ "${FORCE_DRIVER}" -eq 0 ]; then
  if existing_prefix="$(ros2 pkg prefix ldlidar_stl_ros2 2>/dev/null)"; then
    case "${existing_prefix}" in
      "${REPO_ROOT}"/install/*) ;;  # es el de este workspace: tratarlo como propio
      *) USE_EXISTING_DRIVER=1 ;;
    esac
  fi
fi

if [ "${USE_EXISTING_DRIVER}" -eq 1 ]; then
  log "Driver del LD19 ya disponible en ${existing_prefix}: se reutiliza (usa --force-driver para compilarlo aquí)"
else
  log "Driver del LD19 (${LDLIDAR_REPO} @ ${LDLIDAR_REF})"
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
fi

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
rosdep install --from-paths src --ignore-src -y --rosdistro "${ROS_DISTRO_NAME}"

if [ "${SKIP_BUILD}" -eq 0 ]; then
  log "Compilando con colcon (workers: ${JOBS})"
  build_args=(--symlink-install --parallel-workers "${JOBS}")
  if [ "${USE_EXISTING_DRIVER}" -eq 1 ]; then
    # Evita recompilar/sobrescribir el driver del otro workspace si también hay copia en src/.
    [ -d "${LD_DIR}" ] && build_args+=(--packages-skip ldlidar_stl_ros2)
  else
    # El driver no incluye <pthread.h> y falla con el GCC de Ubuntu 24.04 (pthread_mutex_* sin declarar).
    build_args+=(--cmake-args "-DCMAKE_CXX_FLAGS=-include pthread.h")
  fi
  colcon build "${build_args[@]}"
else
  log "Omitiendo compilación (--skip-build)"
fi

log "Listo"
cat <<EOF
Siguientes pasos:
  source ${REPO_ROOT}/install/setup.bash
  ls -l /dev/ldlidar            # o /dev/ttyUSB0 si no usas la regla udev
  scripts/set_robot.sh r2       # opcional: fija el namespace del robot (por defecto r2; no cambia el del Create 3)
  scripts/set_robot.sh --check  # comprueba que existen /<namespace>/odom y /<namespace>/cmd_vel
  ros2 launch create3_bot_bringup master_explore.launch.py lidar_port:=/dev/ldlidar
  # otro robot sin tocar el entorno: ... master_explore.launch.py robot_namespace:=r3
Namespace en esta terminal: ${CREATE3_ROBOT_NAMESPACE-r2} (variable CREATE3_ROBOT_NAMESPACE o 'r2'). Todo el stack cuelga de él: /r2/scan, /r2/map,
/r2/explore_map, /r2/save_exploration_map... El namespace del Create 3 se cambia en su interfaz web.
Comprueba que ves los topics del Create 3 (ros2 topic list) y el QoS de /r2/scan (ros2 topic info /r2/scan -v).
EOF
