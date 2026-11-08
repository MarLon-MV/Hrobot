# TASK-019: Test de higiene del repositorio (sintaxis, marcadores de conflicto, entry points)

- **Rama Git:** `feat/TASK-019`
- **Depende de:** TASK-016
- **Prioridad:** P1

## Contexto y objetivo de negocio
Los merges los hace un humano y no pasan por el gate del orquestador. El merge de TASK-016 dejó marcadores de conflicto (`<<<<<<<`) en `src/hrobot_core/setup.py`; ningún test lo detectó porque nadie importa `setup.py`, y habría roto `colcon build` en la RPi5. Este test es una red de seguridad sobre el árbol completo.

## Alcance técnico
Crear `tests/test_repo_hygiene.py` (sin dependencias de ROS) que, recorriendo el repo desde la raíz y excluyendo `.git`, `.venv`, `__pycache__`, `.mypy_cache`, `.ruff_cache`, `build`, `install`, `log`:
1. **Sintaxis:** `ast.parse` de **todos** los `.py` (incluidos `setup.py`, `scripts/`, launch files y `tasks/` si hubiera).
2. **Marcadores de conflicto:** ningún archivo de texto versionable (`.py .md .yaml .yml .xml .cfg .toml .rviz .msg .txt .json`) tiene una línea que empiece por `<<<<<<< `, `>>>>>>> ` o que sea exactamente `=======`.
3. **Entry points:** para cada `setup.py` con `console_scripts` (leído con `ast`, sin ejecutarlo), cada entrada `nombre = modulo:funcion` apunta a un módulo existente bajo el directorio del paquete y a una función de nivel superior con ese nombre en él.
4. **Paquetes colcon:** cada paquete en `src/*` tiene `package.xml` y, si es ament_python, `setup.py`, `setup.cfg` y `resource/<nombre>`.
5. Tests negativos usando `tmp_path`: el detector de marcadores y el verificador de entry points detectan un caso malo (se implementan como funciones puras que reciben una ruta raíz).

## Archivos permitidos de modificar/crear
- `tests/test_repo_hygiene.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/test_repo_hygiene.py` termina con código 0
- [ ] `ruff check src tests` y `ruff format --check src tests` terminan con código 0
- [ ] Los tests negativos demuestran detección de: marcadores de conflicto, `.py` con sintaxis inválida, entry point a módulo inexistente y entry point a función inexistente

## Fuera de alcance
Corregir hallazgos fuera de `tests/` (si el test real falla por algo preexistente, detenerse con `BLOCKED:` indicando archivo y línea), ejecutar `colcon build`.
