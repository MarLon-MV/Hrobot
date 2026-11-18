#!/usr/bin/env bash
# Bereidt de Raspberry Pi 5 voor om deze workspace te bouwen en uit te voeren.
#
# Vereist: Ubuntu met ROS 2 Jazzy al geïnstalleerd in /opt/ros/jazzy (dit script installeert ROS NIET)
# en met git, colcon en rosdep al geïnstalleerd (dit script installeert die ook niet).
# Idempotent: kan opnieuw worden uitgevoerd zonder iets te breken.
#
# Wat het doet:
#   1. Initialiseert/actualiseert rosdep.
#   2. Driver van de RPLIDAR C1 (rosdep lost die niet op): is er al een `sllidar_ros2` beschikbaar in de omgeving
#      (een andere, al gebouwde en geladen workspace), dan wordt die hergebruikt; zo niet, dan wordt hij gekloond in src/sllidar_ros2.
#   3. Rechten op de seriële poort: groep dialout + udev-regel met symlink /dev/rplidar.
#   4. ROS 2-netwerk: stelt ROS_DOMAIN_ID en RMW_IMPLEMENTATION in (moeten overeenkomen met die van de Create 3),
#      slaat ze op in ~/.config/hrobot/ros_net.env en laadt ze vanuit ~/.bashrc (gemarkeerd blok, één keer).
#   5. Installeert afhankelijkheden met rosdep en bouwt met colcon.
#
# Gebruik:  scripts/setup_pi.sh [--skip-udev] [--skip-build] [--force-driver] [--skip-net]
#                               [--domain-id N] [--rmw rmw_fastrtps_cpp|rmw_cyclonedds_cpp]
#   --skip-udev     voegt je gebruiker niet toe aan dialout en installeert de udev-regel niet (/dev/rplidar
#                   bestaat dan niet: geef de echte poort mee, bijv. lidar_port:=/dev/ttyUSB0)
#   --skip-build    voert colcon build niet uit
#   --force-driver  kloont en bouwt de driver hier ook als er al een in de omgeving bestaat
#   --skip-net      raakt ROS_DOMAIN_ID / RMW_IMPLEMENTATION niet aan
#   --domain-id N   ROS-domein (standaard: $ROS_DOMAIN_ID indien gezet, anders 0)
#   --rmw NAME      middleware (standaard: $RMW_IMPLEMENTATION indien gezet, anders rmw_fastrtps_cpp)
# Optionele variabelen:
#   SLLIDAR_REPO   repository van de driver (standaard: officiële repo van Slamtec)
#   SLLIDAR_REF    branch, tag of commit    (standaard: main; zet vast op een commit zodra de build werkt)
#   ROS_DISTRO_NAME  ROS-distributie        (standaard: jazzy)
#   JOBS           colcon-workers           (standaard: 2, om het RAM van de Pi niet uit te putten)

set -euo pipefail

SLLIDAR_REPO="${SLLIDAR_REPO:-https://github.com/Slamtec/sllidar_ros2.git}"
SLLIDAR_REF="${SLLIDAR_REF:-main}"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
JOBS="${JOBS:-2}"
UDEV_RULE="/etc/udev/rules.d/99-rplidar.rules"
# CP2102 (gebruikelijke USB-serieel-adapter van de RPLIDAR C1). Controleer met `lsusb` en pas aan als jouw adapter anders is.
UDEV_CONTENT='SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar", GROUP="dialout", MODE="0660"'

NET_DIR="${HOME}/.config/hrobot"
NET_FILE="${NET_DIR}/ros_net.env"
BASHRC="${HOME}/.bashrc"
NET_BEGIN="# >>> hrobot ros_net >>>"
NET_END="# <<< hrobot ros_net <<<"
DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
RMW_NAME="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

SKIP_UDEV=0
SKIP_BUILD=0
FORCE_DRIVER=0
SKIP_NET=0
while [ $# -gt 0 ]; do
  arg="$1"; shift
  case "$arg" in
    --skip-udev) SKIP_UDEV=1 ;;
    --skip-build) SKIP_BUILD=1 ;;
    --force-driver) FORCE_DRIVER=1 ;;
    --skip-net) SKIP_NET=1 ;;
    --domain-id) [ $# -gt 0 ] || { echo "--domain-id vereist een waarde" >&2; exit 2; }; DOMAIN_ID="$1"; shift ;;
    --rmw) [ $# -gt 0 ] || { echo "--rmw vereist een waarde" >&2; exit 2; }; RMW_NAME="$1"; shift ;;
    -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "Onbekend argument: $arg (gebruik --help)" >&2; exit 2 ;;
  esac
done

log() { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
die() { printf '\033[1;31mERROR: %s\033[0m\n' "$*" >&2; exit 1; }

[[ "${DOMAIN_ID}" =~ ^[0-9]+$ ]] && [ "${DOMAIN_ID}" -le 232 ] || { echo "Ongeldige ROS_DOMAIN_ID: ${DOMAIN_ID} (0-232)" >&2; exit 2; }
case "${RMW_NAME}" in
  rmw_fastrtps_cpp|rmw_cyclonedds_cpp) ;;
  *) echo "RMW niet ondersteund: ${RMW_NAME} (gebruik rmw_fastrtps_cpp of rmw_cyclonedds_cpp)" >&2; exit 2 ;;
esac

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

[ "$(id -u)" -ne 0 ] || die "Voer dit niet als root uit; gebruik je eigen gebruiker (het script roept sudo aan waar nodig)."
[ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ] || die "/opt/ros/${ROS_DISTRO_NAME}/setup.bash niet gevonden. Installeer eerst ROS 2 ${ROS_DISTRO_NAME}."
[ -d src ] || die "De map src/ is niet gevonden in ${REPO_ROOT}."

log "rosdep initialiseren"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update --rosdistro "${ROS_DISTRO_NAME}"

# setup.bash van ROS gebruikt ongedefinieerde variabelen: schakel -u uit tijdens het laden.
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
set -u

if [ "${SKIP_NET}" -eq 0 ]; then
  log "ROS 2-netwerk (ROS_DOMAIN_ID=${DOMAIN_ID}, RMW_IMPLEMENTATION=${RMW_NAME})"
  # Installeert de gekozen middleware (rmw_fastrtps_cpp zit al in ros-base; cyclonedds niet altijd).
  sudo apt-get install -y "ros-${ROS_DISTRO_NAME}-${RMW_NAME//_/-}"
  mkdir -p "${NET_DIR}"
  printf 'export ROS_DOMAIN_ID=%s\nexport RMW_IMPLEMENTATION=%s\n' "${DOMAIN_ID}" "${RMW_NAME}" > "${NET_FILE}"
  touch "${BASHRC}"
  if ! grep -qxF "${NET_BEGIN}" "${BASHRC}"; then
    if [ -s "${BASHRC}" ] && [ -n "$(tail -c 1 "${BASHRC}")" ]; then printf '\n' >> "${BASHRC}"; fi
    {
      echo "${NET_BEGIN}"
      echo "[ -f \"${NET_FILE}\" ] && . \"${NET_FILE}\""
      echo "${NET_END}"
    } >> "${BASHRC}"
  fi
  # Ook op dit proces toepassen (build/rosdep) en waarschuwen voor de robot.
  export ROS_DOMAIN_ID="${DOMAIN_ID}" RMW_IMPLEMENTATION="${RMW_NAME}"
  echo "Opgeslagen in ${NET_FILE}. Open een nieuwe terminal (of 'source ~/.bashrc')."
  echo "BELANGRIJK: de Create 3 moet dezelfde RMW gebruiken (webinterface > Application > ROS 2) en zijn ROS_DOMAIN_ID"
  echo "moet ${DOMAIN_ID} zijn. Herstart de applicatie van de Create 3 na het wijzigen."
else
  log "ROS-netwerkconfiguratie overgeslagen (--skip-net)"
fi

LD_DIR="src/sllidar_ros2"
USE_EXISTING_DRIVER=0
if [ "${FORCE_DRIVER}" -eq 0 ]; then
  if existing_prefix="$(ros2 pkg prefix sllidar_ros2 2>/dev/null)"; then
    case "${existing_prefix}" in
      "${REPO_ROOT}"/install/*) ;;  # is die van deze workspace: behandel als eigen
      *) USE_EXISTING_DRIVER=1 ;;
    esac
  fi
fi

if [ "${USE_EXISTING_DRIVER}" -eq 1 ]; then
  log "Driver van de RPLIDAR C1 al beschikbaar in ${existing_prefix}: wordt hergebruikt (gebruik --force-driver om hem hier te bouwen)"
else
  log "Driver van de RPLIDAR C1 (${SLLIDAR_REPO} @ ${SLLIDAR_REF})"
  if [ -d "${LD_DIR}/.git" ]; then
    git -C "${LD_DIR}" fetch --tags origin
  else
    git clone "${SLLIDAR_REPO}" "${LD_DIR}"
  fi
  git -C "${LD_DIR}" checkout "${SLLIDAR_REF}"
  # Is SLLIDAR_REF een branch, ga dan naar de laatste versie (zonder merges); bij een tag/commit gebeurt er niets.
  if git -C "${LD_DIR}" symbolic-ref -q HEAD >/dev/null; then
    git -C "${LD_DIR}" merge --ff-only "origin/${SLLIDAR_REF}"
  fi
  echo "Driver op commit: $(git -C "${LD_DIR}" rev-parse --short HEAD)"
fi

if [ "${SKIP_UDEV}" -eq 0 ]; then
  log "Rechten op de seriële poort (dialout + udev /dev/rplidar)"
  if ! id -nG "${USER}" | tr ' ' '\n' | grep -qx dialout; then
    sudo usermod -aG dialout "${USER}"
    echo "${USER} toegevoegd aan de groep dialout: log uit en weer in om het van kracht te laten worden."
  fi
  if [ ! -f "${UDEV_RULE}" ] || [ "$(cat "${UDEV_RULE}")" != "${UDEV_CONTENT}" ]; then
    printf '%s\n' "${UDEV_CONTENT}" | sudo tee "${UDEV_RULE}" >/dev/null
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "Udev-regel geïnstalleerd in ${UDEV_RULE}. Sluit de LiDAR opnieuw aan; /dev/rplidar zou moeten verschijnen."
  fi
else
  log "Udev-rechten overgeslagen (--skip-udev)"
fi

log "Afhankelijkheden installeren met rosdep"
# hrobot_simulation (Gazebo) is alleen voor de ontwikkel-pc: wordt niet op de Pi geïnstalleerd of gebouwd.
rosdep install --from-paths src --ignore-src -y --rosdistro "${ROS_DISTRO_NAME}" \
  --skip-keys "ros_gz_sim ros_gz_bridge"

if [ "${SKIP_BUILD}" -eq 0 ]; then
  log "Bouwen met colcon (workers: ${JOBS})"
  # Er wordt hrobot_bringup gebouwd met zijn workspace-afhankelijkheden (hrobot_description en de driver sllidar_ros2).
  # hrobot_simulation (Gazebo) valt erbuiten: die is alleen voor de ontwikkel-pc.
  build_args=(--symlink-install --parallel-workers "${JOBS}" --packages-up-to hrobot_bringup)
  if [ "${USE_EXISTING_DRIVER}" -eq 1 ]; then
    # Voorkomt dat de driver van de andere workspace opnieuw wordt gebouwd/overschreven als er ook een kopie in src/ staat.
    if [ -d "${LD_DIR}" ]; then build_args+=(--packages-skip sllidar_ros2); fi
  fi
  colcon build "${build_args[@]}"
else
  log "Bouwen overgeslagen (--skip-build)"
fi

log "Klaar"
cat <<EOF
Volgende stappen:
  source ${REPO_ROOT}/install/setup.bash
  ls -l /dev/rplidar            # of /dev/ttyUSB0 als je de udev-regel niet gebruikt
  echo \$ROS_DOMAIN_ID \$RMW_IMPLEMENTATION   # ${ROS_DOMAIN_ID-?} ${RMW_IMPLEMENTATION-?}; moeten overeenkomen met de Create 3
  ros2 topic list | grep -E '/r2/(odom|cmd_vel)'   # controleer de topics van de Create 3
  ros2 launch hrobot_bringup mapping.launch.py lidar_port:=/dev/rplidar
  # andere robot zonder de omgeving aan te passen: ... mapping.launch.py robot_namespace:=r3
Namespace in deze terminal: ${CREATE3_ROBOT_NAMESPACE-r2} (variabele CREATE3_ROBOT_NAMESPACE of 'r2'). De hele stack hangt eronder: /r2/scan, /r2/map,
.../r2/cmd_vel... De namespace van de Create 3 wijzig je in zijn webinterface.
Controleer dat je de topics van de Create 3 ziet (ros2 topic list) en de QoS van /r2/scan (ros2 topic info /r2/scan -v).
EOF
