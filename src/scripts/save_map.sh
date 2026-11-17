#!/usr/bin/env bash
# Guarda el mapa que slam_toolbox está construyendo (mapping.launch.py debe estar corriendo).
#
# Genera en <maps_dir> los dos formatos que usa el flujo mapear -> navegar:
#   <nombre>.posegraph + <nombre>.data   para AMPLIAR el mapa:  mapping.launch.py map:=<nombre> maps_dir:=<maps_dir>
#   <nombre>.yaml + <nombre>.pgm         para NAVEGAR:          navigation.launch.py map_yaml:=<maps_dir>/<nombre>.yaml
#
# Uso:  src/scripts/save_map.sh <nombre> [--maps-dir DIR] [--namespace NS] [--force]
#   <nombre>       solo letras, dígitos, '_' y '-' (sin ruta ni extensión)
#   --maps-dir     carpeta de salida (def: <raíz del workspace>/maps)
#   --namespace    namespace del robot (def: $CREATE3_ROBOT_NAMESPACE si existe, si no r2; vacío = ninguno)
#   --force        sobrescribe un mapa existente con ese nombre
#
# slam_toolbox escribe los archivos en la máquina donde corre; ejecuta este script en esa misma máquina.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
MAPS_DIR="${WS_ROOT}/maps"
NAMESPACE="${CREATE3_ROBOT_NAMESPACE-r2}"
FORCE=0
NAME=""

die() { echo "ERROR: $*" >&2; exit 1; }

while [ $# -gt 0 ]; do
  arg="$1"; shift
  case "$arg" in
    --maps-dir) [ $# -gt 0 ] || die "--maps-dir necesita un valor"; MAPS_DIR="$1"; shift ;;
    --namespace) [ $# -gt 0 ] || die "--namespace necesita un valor (usa '' para ninguno)"; NAMESPACE="$1"; shift ;;
    --force) FORCE=1 ;;
    -h|--help) sed -n '2,15p' "${BASH_SOURCE[0]}"; exit 0 ;;
    -*) die "opción desconocida: ${arg}" ;;
    *) [ -z "${NAME}" ] || die "solo se admite un nombre de mapa"; NAME="${arg}" ;;
  esac
done

[ -n "${NAME}" ] || die "falta el nombre del mapa. Uso: save_map.sh <nombre> [--maps-dir DIR] [--namespace NS] [--force]"
[[ "${NAME}" =~ ^[A-Za-z0-9_-]+$ ]] || die "nombre inválido '${NAME}': usa solo letras, dígitos, '_' y '-'"

NAMESPACE="${NAMESPACE#/}"; NAMESPACE="${NAMESPACE%/}"
PREFIX=""
[ -z "${NAMESPACE}" ] || PREFIX="/${NAMESPACE}"

command -v ros2 >/dev/null || die "ros2 no está en el PATH: haz source de /opt/ros/jazzy/setup.bash y del workspace"

mkdir -p "${MAPS_DIR}"
MAPS_DIR="$(cd "${MAPS_DIR}" && pwd)"   # ruta absoluta: slam_toolbox no expande '~'
BASE="${MAPS_DIR}/${NAME}"

if [ "${FORCE}" -eq 0 ]; then
  for ext in posegraph data yaml pgm; do
    [ ! -e "${BASE}.${ext}" ] || die "${BASE}.${ext} ya existe (usa --force para sobrescribir)"
  done
fi

SER="${PREFIX}/slam_toolbox/serialize_map"
SAVE="${PREFIX}/slam_toolbox/save_map"
timeout 10 ros2 service list 2>/dev/null | grep -qx "${SER}" \
  || die "no existe el servicio ${SER}: ¿está corriendo mapping.launch.py y el namespace es '${NAMESPACE}'?"

echo "[1/2] Guardando pose graph en ${BASE}.posegraph/.data ..."
timeout 60 ros2 service call "${SER}" slam_toolbox/srv/SerializePoseGraph "{filename: '${BASE}'}" >/dev/null \
  || die "falló la llamada a ${SER}"

echo "[2/2] Guardando imagen en ${BASE}.yaml/.pgm ..."
timeout 60 ros2 service call "${SAVE}" slam_toolbox/srv/SaveMap "{name: {data: '${BASE}'}}" >/dev/null \
  || die "falló la llamada a ${SAVE}"

missing=0
for ext in posegraph data yaml pgm; do
  if [ ! -s "${BASE}.${ext}" ]; then echo "ERROR: no se generó ${BASE}.${ext}" >&2; missing=1; fi
done
[ "${missing}" -eq 0 ] || die "el mapa no se guardó completo (¿slam_toolbox corre en otra máquina?)"

echo
echo "Mapa '${NAME}' guardado en ${MAPS_DIR}:"
ls -l "${BASE}".{posegraph,data,yaml,pgm}
echo
echo "Navegar:  ros2 launch hrobot_bringup navigation.launch.py map_yaml:=${BASE}.yaml"
echo "Ampliar:  ros2 launch hrobot_bringup mapping.launch.py map:=${NAME} maps_dir:=${MAPS_DIR}"
