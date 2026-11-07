# TASK-001: Implementar validador de configuración (ConfigValidator)

- **Rama Git:** `feat/TASK-001`
- **Depende de:** TASK-002
- **Prioridad:** P0

## Contexto y objetivo de negocio
Los servicios del sistema se configuran mediante un diccionario (cargado desde JSON/ENV). Hoy no existe validación, por lo que configuraciones inválidas provocan fallos tardíos y difíciles de diagnosticar en producción. Se requiere un validador puro (sin I/O) que detecte todos los errores de configuración de una sola vez y los reporte con mensajes accionables. Esto reduce incidentes de arranque y sirve de base para futuras tareas de carga de configuración.

## Alcance técnico
Crear en la capa `domain` un validador con esta API:

```python
# src/hrobot_core/hrobot_core/domain/config.py
@dataclass(frozen=True)
class AppConfig:
    service_name: str      # no vacío, regex ^[a-z][a-z0-9-]{2,30}$
    port: int              # 1024 <= port <= 65535
    log_level: str         # uno de: DEBUG, INFO, WARNING, ERROR
    max_workers: int       # 1 <= max_workers <= 64, default 4
    database_url: str      # debe empezar por "postgresql://" o "sqlite:///"

class ConfigValidationError(Exception):
    errors: list[str]      # un mensaje por cada campo inválido, orden estable por nombre de campo

def validate_config(raw: Mapping[str, object]) -> AppConfig: ...
```

Comportamiento:
- Campos obligatorios ausentes: `service_name`, `port`, `database_url`. Opcionales con default: `log_level="INFO"`, `max_workers=4`.
- Tipos estrictos: `port="8080"` (str) es inválido; `True` no es un `int` válido.
- Claves desconocidas son un error (`unknown field: <nombre>`).
- Se acumulan **todos** los errores y se lanza un único `ConfigValidationError`.
- La función no muta `raw` y no realiza I/O.

## Archivos permitidos de modificar/crear
- `src/hrobot_core/hrobot_core/domain/__init__.py`
- `src/hrobot_core/hrobot_core/domain/config.py`
- `src/hrobot_core/hrobot_core/__init__.py`
- `tests/domain/__init__.py`
- `tests/domain/test_config.py`

## Criterios de Aceptación (DoD)
- [ ] `PYTHONPATH=src/hrobot_core pytest tests/domain/test_config.py -q` termina con código 0
- [ ] `PYTHONPATH=src/hrobot_core pytest tests/domain/test_config.py --cov=hrobot_core.domain.config --cov-fail-under=95 -q` termina con código 0
- [ ] `ruff check .` termina con código 0
- [ ] `ruff format --check .` termina con código 0
- [ ] `mypy --strict src/hrobot_core/hrobot_core` termina con código 0
- [ ] Un test verifica que una config válida mínima devuelve `AppConfig` con defaults (`log_level == "INFO"`, `max_workers == 4`)
- [ ] Un test verifica que una config con 3 campos inválidos produce `len(errors) == 3`
- [ ] Tests de bordes: `port` en 1023/1024/65535/65536, `max_workers` en 0/1/64/65, `bool` como `int`, clave desconocida, `raw` vacío
- [ ] `git diff --name-only main...HEAD` solo lista archivos de la sección "Archivos permitidos" (más el movimiento de la tarea en `tasks/`)

## Checklist de verificación para el Reviewer
- [ ] No hay imports de `services` ni `adapters` dentro de `src/hrobot_core/hrobot_core/domain/config.py`
- [ ] No se usa `eval`, `exec` ni regex con backtracking catastrófico
- [ ] Los mensajes de error no incluyen valores sensibles (p. ej. no volcar `database_url` completo con credenciales)
- [ ] `raw` no es mutado (test explícito)
- [ ] Commits con formato Conventional Commits y scope `(TASK-001)`

## Fuera de alcance
- Carga desde archivos o variables de entorno (futura tarea en `adapters`).
- Conexión real a la base de datos.
