#!/usr/bin/env bash
# Slaat de kaart op die slam_toolbox aan het bouwen is (mapping.launch.py moet draaien).
#
# Genereert in <maps_dir> de twee formaten die de workflow kaarten -> navigeren gebruikt:
#   <naam>.posegraph + <naam>.data   om de kaart UIT TE BREIDEN:  mapping.launch.py map:=<naam> maps_dir:=<maps_dir>
#   <naam>.yaml + <naam>.pgm         om te NAVIGEREN:               navigation.launch.py map_yaml:=<maps_dir>/<naam>.yaml
#
# Gebruik:  src/scripts/save_map.sh <naam> [--maps-dir DIR] [--namespace NS] [--force]
#                                          [--pull GEBRUIKER@HOST] [--remote-ws DIR]
#   <naam>         alleen letters, cijfers, '_' en '-' (zonder pad of extensie)
#   --maps-dir     uitvoermap op DEZE machine (standaard: <workspace-root>/src/maps)
#   --namespace    namespace van de robot (standaard: $CREATE3_ROBOT_NAMESPACE indien gezet, anders r2; leeg = geen)
#   --force        overschrijft een bestaande kaart met die naam
#   --pull         slam_toolbox draait op een ANDERE machine (de Pi): de bestanden worden daar opgeslagen,
#                  in <remote-ws>/src/maps, en daarna met scp naar --maps-dir gekopieerd. Vereist ssh naar die host.
#   --remote-ws    workspace op de andere machine (standaard: ~/Hrobot); alleen met --pull
#
# slam_toolbox schrijft de bestanden op de machine waar hij draait. Voer dit script zonder --pull op diezelfde
# machine uit; de services antwoorden via het netwerk vanaf een andere, maar de bestanden zouden op de externe schijf belanden.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
MAPS_DIR="${WS_ROOT}/src/maps"
NAMESPACE="${CREATE3_ROBOT_NAMESPACE-r2}"
FORCE=0
NAME=""
PULL=""
REMOTE_WS="~/Hrobot"
EXTS=(posegraph data yaml pgm)

die() { echo "ERROR: $*" >&2; exit 1; }

while [ $# -gt 0 ]; do
  arg="$1"; shift
  case "$arg" in
    --maps-dir) [ $# -gt 0 ] || die "--maps-dir heeft een waarde nodig"; MAPS_DIR="$1"; shift ;;
    --namespace) [ $# -gt 0 ] || die "--namespace heeft een waarde nodig (gebruik '' voor geen)"; NAMESPACE="$1"; shift ;;
    --pull) [ $# -gt 0 ] || die "--pull heeft gebruiker@host nodig"; PULL="$1"; shift ;;
    --remote-ws) [ $# -gt 0 ] || die "--remote-ws heeft een waarde nodig"; REMOTE_WS="$1"; shift ;;
    --force) FORCE=1 ;;
    -h|--help) sed -n '2,19p' "${BASH_SOURCE[0]}"; exit 0 ;;
    -*) die "onbekende optie: ${arg}" ;;
    *) [ -z "${NAME}" ] || die "slechts één kaartnaam is toegestaan"; NAME="${arg}" ;;
  esac
done

[ -n "${NAME}" ] || die "kaartnaam ontbreekt. Gebruik: save_map.sh <naam> [--maps-dir DIR] [--namespace NS] [--force] [--pull gebruiker@host]"
[[ "${NAME}" =~ ^[A-Za-z0-9_-]+$ ]] || die "ongeldige naam '${NAME}': gebruik alleen letters, cijfers, '_' en '-'"

NAMESPACE="${NAMESPACE#/}"; NAMESPACE="${NAMESPACE%/}"
PREFIX=""
[ -z "${NAMESPACE}" ] || PREFIX="/${NAMESPACE}"

command -v ros2 >/dev/null || die "ros2 staat niet in het PATH: voer source uit op /opt/ros/jazzy/setup.bash en op de workspace"

mkdir -p "${MAPS_DIR}"
MAPS_DIR="$(cd "${MAPS_DIR}" && pwd)"   # absoluut pad
LOCAL_BASE="${MAPS_DIR}/${NAME}"

if [ "${FORCE}" -eq 0 ]; then
  for ext in "${EXTS[@]}"; do
    [ ! -e "${LOCAL_BASE}.${ext}" ] || die "${LOCAL_BASE}.${ext} bestaat al (gebruik --force om te overschrijven)"
  done
fi

# BASE = pad (zonder extensie) dat slam_toolbox ontvangt: moet bestaan op de machine waar hij draait.
if [ -n "${PULL}" ]; then
  command -v ssh >/dev/null && command -v scp >/dev/null || die "--pull heeft ssh en scp nodig"
  # slam_toolbox expandeert '~' niet: wordt op de andere machine naar een absoluut pad omgezet.
  REMOTE_WS_ABS="$(ssh "${PULL}" "cd ${REMOTE_WS} && pwd")" \
    || die "kan '${REMOTE_WS}' op ${PULL} niet openen (klopt --remote-ws? werkt ssh?)"
  REMOTE_MAPS="${REMOTE_WS_ABS}/src/maps"
  ssh "${PULL}" "mkdir -p '${REMOTE_MAPS}'" || die "kan ${REMOTE_MAPS} op ${PULL} niet aanmaken"
  if [ "${FORCE}" -eq 0 ]; then
    for ext in "${EXTS[@]}"; do
      ssh "${PULL}" "[ ! -e '${REMOTE_MAPS}/${NAME}.${ext}' ]" \
        || die "${PULL}:${REMOTE_MAPS}/${NAME}.${ext} bestaat al (gebruik --force om te overschrijven)"
    done
  fi
  BASE="${REMOTE_MAPS}/${NAME}"
else
  BASE="${LOCAL_BASE}"
  if ! pgrep -f async_slam_toolbox_node >/dev/null 2>&1; then
    echo "WAARSCHUWING: er draait geen slam_toolbox op deze machine. Draait hij op de Pi, dan worden de bestanden" >&2
    echo "       daar opgeslagen: gebruik --pull gebruiker@host (of voer dit script op de Pi uit)." >&2
  fi
fi

SER="${PREFIX}/slam_toolbox/serialize_map"
SAVE="${PREFIX}/slam_toolbox/save_map"
timeout 10 ros2 service list 2>/dev/null | grep -qx "${SER}" \
  || die "de service ${SER} bestaat niet: draait mapping.launch.py en is de namespace '${NAMESPACE}'?"

# Roept een service aan en toont het antwoord (voorheen werd het weggegooid en bleef de foutreden verborgen).
call_service() {
  local service="$1" type="$2" request="$3" out
  out="$(timeout 60 ros2 service call "${service}" "${type}" "${request}" 2>&1)" \
    || { echo "${out}" >&2; die "aanroep van ${service} mislukt"; }
  echo "${out}" | sed -n '/response/,$p' | sed 's/^/      /'
}

echo "[1/2] Pose graph opslaan in ${BASE}.posegraph/.data ..."
call_service "${SER}" slam_toolbox/srv/SerializePoseGraph "{filename: '${BASE}'}"

echo "[2/2] Afbeelding opslaan in ${BASE}.yaml/.pgm ..."
call_service "${SAVE}" slam_toolbox/srv/SaveMap "{name: {data: '${BASE}'}}"

if [ -n "${PULL}" ]; then
  missing=0
  for ext in "${EXTS[@]}"; do
    ssh "${PULL}" "[ -s '${BASE}.${ext}' ]" || { echo "ERROR: ${PULL}:${BASE}.${ext} is niet aangemaakt" >&2; missing=1; }
  done
  [ "${missing}" -eq 0 ] || die "de kaart is niet volledig opgeslagen in ${PULL}:${REMOTE_MAPS}"
  echo "[+] Kopiëren van ${PULL} naar ${MAPS_DIR} ..."
  for ext in "${EXTS[@]}"; do
    scp -q "${PULL}:${BASE}.${ext}" "${LOCAL_BASE}.${ext}" || die "scp van ${BASE}.${ext} mislukt"
  done
fi

missing=0
for ext in "${EXTS[@]}"; do
  if [ ! -s "${LOCAL_BASE}.${ext}" ]; then echo "ERROR: ${LOCAL_BASE}.${ext} is niet aangemaakt" >&2; missing=1; fi
done
if [ "${missing}" -ne 0 ]; then
  [ -n "${PULL}" ] || echo "De services antwoordden maar er zijn hier geen bestanden: slam_toolbox draait op een andere machine en heeft ze daar opgeslagen." >&2
  [ -n "${PULL}" ] || echo "Herhaal met --pull gebruiker@host, of kopieer de bestanden handmatig met scp." >&2
  die "de kaart is niet volledig opgeslagen in ${MAPS_DIR}"
fi

echo
echo "Kaart '${NAME}' opgeslagen in ${MAPS_DIR}:"
ls -l "${LOCAL_BASE}".{posegraph,data,yaml,pgm}
echo
echo "Navigeren:  ros2 launch hrobot_bringup navigation.launch.py map_yaml:=${LOCAL_BASE}.yaml"
echo "Uitbreiden:  ros2 launch hrobot_bringup mapping.launch.py map:=${NAME} maps_dir:=${MAPS_DIR}"
