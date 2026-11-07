# TASK-002: Scaffold de los tres paquetes ROS 2 (`hrobot_interfaces`, `hrobot_core`, `hrobot_bringup`) y herramientas

- **Rama Git:** `feat/TASK-002`
- **Depende de:** ninguna
- **Prioridad:** P0

## Contexto y objetivo de negocio
Todas las tareas posteriores necesitan paquetes construibles con responsabilidades separadas y herramientas de calidad (ruff, mypy, pytest) configuradas de forma uniforme. Esta tarea crea el esqueleto sin lógica de negocio.

## Alcance técnico
- **`src/hrobot_interfaces/`** (`ament_cmake`): `package.xml` (formato 3, `rosidl_default_generators`, `rosidl_default_runtime`, `std_msgs`, `<member_of_group>rosidl_interface_packages`), `CMakeLists.txt` con `rosidl_generate_interfaces`, y un mensaje mínimo `msg/MissionStatus.msg` (`string mission_id`, `string state`, `bool delivered`).
- **`src/hrobot_core/`** (`ament_python`): `package.xml` (dependencias `rclpy`, `geometry_msgs`, `sensor_msgs`, `nav2_msgs`, `hrobot_interfaces`), `setup.py`, `setup.cfg`, `resource/hrobot_core`, y módulo `hrobot_core/` con subpaquetes `domain/`, `services/`, `adapters/` (cada uno con `__init__.py`). `hrobot_core/__init__.py` expone `__version__ = "0.1.0"`.
- **`src/hrobot_bringup/`** (`ament_python`): `package.xml` (dependencias de ejecución `hrobot_core`, `nav2_bringup`, `slam_toolbox`), `setup.py` (instala `launch/`, `config/`, `rviz/`, `maps/` vía `data_files`), `setup.cfg`, `resource/hrobot_bringup`, módulo vacío `hrobot_bringup/__init__.py`, y archivos `launch/.gitkeep`, `config/.gitkeep`, `rviz/.gitkeep`, `maps/.gitkeep`.
- **`pyproject.toml`** en la raíz, con `[project]` (nombre `hrobot-dev`, `requires-python = ">=3.11"`) y `[project.optional-dependencies] dev` con las mismas dependencias y rangos de `requirements-dev.txt` (que ya existe y **no se modifica**), más la configuración de herramientas: `ruff` (`extend-exclude = ["tasks", "scripts", ".venv"]`, línea 100, reglas `E,F,I,B,UP,S,SIM,C4`), `mypy` (strict, `mypy_path = src/hrobot_core`), `pytest` (`testpaths = tests`, `pythonpath = src/hrobot_core`).
- **`tests/test_packages.py`**: verifica `hrobot_core.__version__` y que `hrobot_core.domain`, `hrobot_core.services`, `hrobot_core.adapters` sean importables **sin ROS instalado**; y que cada `package.xml` parsea y declara el `build_type` esperado (`ament_cmake`, `ament_python`, `ament_python`).

## Archivos permitidos de modificar/crear
- `pyproject.toml`
- `src/hrobot_interfaces/**`
- `src/hrobot_core/**`
- `src/hrobot_bringup/**`
- `tests/test_packages.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest -q tests/test_packages.py` termina con código 0
- [ ] `ruff check src tests` termina con código 0
- [ ] `ruff format --check src tests` termina con código 0
- [ ] `mypy --strict src/hrobot_core/hrobot_core` termina con código 0
- [ ] Un test en `tests/test_packages.py` verifica que `pyproject.toml` declara `[project.optional-dependencies].dev` e incluye `pytest`, `ruff`, `mypy` y `PyYAML` (usando `tomllib`)
- [ ] `python3 -c "import xml.etree.ElementTree as E; [E.parse(f'src/{p}/package.xml') for p in ('hrobot_interfaces','hrobot_core','hrobot_bringup')]"` termina con código 0
- [ ] `grep -q "ament_cmake" src/hrobot_interfaces/package.xml` y `grep -q "ament_python" src/hrobot_core/package.xml src/hrobot_bringup/package.xml` terminan con código 0
- [ ] `hrobot_core/domain` no importa `rclpy` ni `hrobot_interfaces`: `grep -rE "import (rclpy|hrobot_interfaces)" src/hrobot_core/hrobot_core/domain` sin coincidencias
- [ ] (Opcional, si hay ROS 2) `colcon build --packages-select hrobot_interfaces hrobot_core hrobot_bringup` termina con código 0

## Fuera de alcance
Nodos, launch files con contenido, lógica de dominio, más mensajes/acciones.
