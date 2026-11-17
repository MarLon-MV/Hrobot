#!/usr/bin/env bash
# Selecciona el robot (namespace ROS 2) con el que trabaja este workspace.
#
# Uso:
#   scripts/set_robot.sh <nombre>   guarda CREATE3_ROBOT_NAMESPACE=<nombre> (p. ej. r2, r3)
#   scripts/set_robot.sh --show     muestra el nombre configurado
#   scripts/set_robot.sh --check    comprueba con `ros2 topic list` que existen /<nombre>/odom y /<nombre>/cmd_vel
#   scripts/set_robot.sh --unset    elimina la configuracion (los launch vuelven al valor por defecto: r2)
#   scripts/set_robot.sh -h         esta ayuda
#
# Escribe ~/.config/create3_bot/robot.env y anade UNA sola vez a ~/.bashrc un bloque marcado que lo
# carga. Es idempotente. Abre una terminal nueva (o `source ~/.bashrc`) para que surta efecto.
#
# IMPORTANTE: este script NO cambia el namespace del propio Create 3. Eso se configura en su
# interfaz web (Application > ROS 2 > Namespace); debe coincidir con el nombre elegido aqui.
# Los launch usan: argumento robot_namespace:=<nombre> > variable CREATE3_ROBOT_NAMESPACE > "r2".

set -euo pipefail

ENV_DIR="${HOME}/.config/create3_bot"
ENV_FILE="${ENV_DIR}/robot.env"
BASHRC="${HOME}/.bashrc"
MARK_BEGIN="# >>> create3_bot robot >>>"
MARK_END="# <<< create3_bot robot <<<"
NAME_RE='^[a-z][a-z0-9_]{0,31}$'
DEFAULT_NAME="r2"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"

usage() { sed -n '2,16p' "$0"; }
die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

current_name() {
  # Nombre guardado en robot.env (vacio si no hay).
  if [ -f "${ENV_FILE}" ]; then
    sed -n 's/^export CREATE3_ROBOT_NAMESPACE=//p' "${ENV_FILE}" | tail -n 1
  fi
}

has_block() { [ -f "${BASHRC}" ] && grep -qxF "${MARK_BEGIN}" "${BASHRC}"; }

remove_block() {
  # Elimina el bloque marcado de ~/.bashrc (si existe) sin tocar el resto.
  has_block || return 0
  local tmp
  tmp="$(mktemp)"
  awk -v b="${MARK_BEGIN}" -v e="${MARK_END}" '
    $0 == b { skip = 1; next }
    skip && $0 == e { skip = 0; next }
    !skip { print }
  ' "${BASHRC}" > "${tmp}"
  cat "${tmp}" > "${BASHRC}"   # conserva permisos/propietario del .bashrc
  rm -f "${tmp}"
}

add_block() {
  has_block && return 0
  touch "${BASHRC}"
  # Linea en blanco previa solo si el archivo no termina ya en una.
  if [ -s "${BASHRC}" ] && [ -n "$(tail -c 1 "${BASHRC}")" ]; then printf '\n' >> "${BASHRC}"; fi
  {
    printf '%s\n' "${MARK_BEGIN}"
    printf '%s\n' '[ -f "$HOME/.config/create3_bot/robot.env" ] && . "$HOME/.config/create3_bot/robot.env"'
    printf '%s\n' "${MARK_END}"
  } >> "${BASHRC}"
}

cmd_set() {
  local name="$1"
  [[ "${name}" =~ ${NAME_RE} ]] || die "nombre invalido '${name}': usa minusculas, digitos y '_' (empieza por letra, max. 32), p. ej. r2."
  mkdir -p "${ENV_DIR}"
  printf 'export CREATE3_ROBOT_NAMESPACE=%s\n' "${name}" > "${ENV_FILE}"
  add_block
  cat <<EOF
Robot configurado: ${name}  (${ENV_FILE})
  - Abre una terminal nueva o ejecuta: source ${ENV_FILE}
  - Topics esperados del Create 3: /${name}/odom, /${name}/cmd_vel, /${name}/tf
  - Este script NO cambia el namespace del Create 3: configuralo en su interfaz web
    (Application > ROS 2 > Namespace) para que sea /${name}.
  - Comprueba la conexion con: $0 --check
EOF
}

cmd_show() {
  local saved
  saved="$(current_name)"
  if [ -n "${saved}" ]; then
    echo "Configurado en ${ENV_FILE}: ${saved}"
  else
    echo "Sin configurar (los launch usan el valor por defecto: ${DEFAULT_NAME})."
  fi
  if [ "${CREATE3_ROBOT_NAMESPACE+set}" = set ]; then
    echo "CREATE3_ROBOT_NAMESPACE en este shell: '${CREATE3_ROBOT_NAMESPACE}'"
  else
    echo "CREATE3_ROBOT_NAMESPACE no esta definida en este shell."
  fi
  if has_block; then echo "Bloque en ${BASHRC}: presente"; else echo "Bloque en ${BASHRC}: ausente"; fi
}

cmd_unset() {
  remove_block
  rm -f "${ENV_FILE}"
  rmdir "${ENV_DIR}" 2>/dev/null || true
  echo "Configuracion eliminada. Los launch usaran el valor por defecto (${DEFAULT_NAME}) en terminales nuevas."
  echo "En esta terminal, si aplica: unset CREATE3_ROBOT_NAMESPACE"
}

cmd_check() {
  local name
  name="${CREATE3_ROBOT_NAMESPACE:-$(current_name)}"
  name="${name:-${DEFAULT_NAME}}"
  [[ "${name}" =~ ${NAME_RE} ]] || die "namespace '${name}' invalido (ver --show)."
  if ! command -v ros2 >/dev/null 2>&1; then
    if [ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]; then
      # setup.bash usa variables sin definir: desactivar -u mientras se carga.
      set +u
      # shellcheck disable=SC1090
      . "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
      set -u
    fi
  fi
  command -v ros2 >/dev/null 2>&1 || die "no se encuentra 'ros2' (instala/carga ROS 2 ${ROS_DISTRO_NAME}: source /opt/ros/${ROS_DISTRO_NAME}/setup.bash)."
  local topics
  topics="$(ros2 topic list 2>/dev/null)" || die "'ros2 topic list' fallo (revisa ROS_DOMAIN_ID y RMW)."
  local missing=0 t
  for t in "/${name}/odom" "/${name}/cmd_vel"; do
    if printf '%s\n' "${topics}" | grep -qxF "${t}"; then
      echo "OK       ${t}"
    else
      echo "FALTA    ${t}"
      missing=1
    fi
  done
  if [ "${missing}" -ne 0 ]; then
    cat >&2 <<EOF

Faltan topics del Create 3 con namespace /${name}. Revisa:
  - El namespace del Create 3 en su interfaz web (Application > ROS 2); este script no lo cambia.
  - Que la Raspberry Pi y el Create 3 compartan ROS_DOMAIN_ID y RMW, y que el robot este encendido y en red.
  - Que el nombre configurado (--show) sea el correcto. Topics /<ns>/... visibles ahora:
EOF
    printf '%s\n' "${topics}" | grep -E '/(odom|cmd_vel)$' >&2 || echo "    (ninguno)" >&2
    exit 1
  fi
  echo "Robot '${name}' detectado."
}

[ "$#" -ge 1 ] || { usage >&2; exit 2; }
[ "$#" -le 1 ] || die "demasiados argumentos (usa -h)."
case "$1" in
  -h|--help) usage ;;
  --show) cmd_show ;;
  --check) cmd_check ;;
  --unset) cmd_unset ;;
  -*) die "opcion desconocida: $1 (usa -h)." ;;
  *) cmd_set "$1" ;;
esac
