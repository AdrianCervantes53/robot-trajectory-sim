# Robot 6-DOF Simulator

Simulador cinemático de un brazo robótico de 6 grados de libertad, portado desde MATLAB a Python con una API REST/WebSocket y un frontend 3D en el browser.

## Stack

| Capa | Tecnología |
|------|-----------|
| Cinemática | Python · NumPy |
| API | FastAPI · Uvicorn · WebSockets |
| Cliente HTTP (tests) | HTTPX |
| Frontend | Three.js — ES Modules nativos, sin build tools |
| Tests | Pytest — 101 tests |

## Estructura

```
robot-trajectory-sim/
├── api/
│   ├── routers/
│   │   ├── movement.py       # Endpoints: /joints, /pose, /gripper, /config
│   │   ├── trajectory.py     # Endpoints: /trajectory/*
│   │   └── ws_trajectory.py  # WebSocket: /ws
│   ├── schemas/
│   │   ├── movement.py       # Pydantic models para movimiento
│   │   └── trajectory.py     # Pydantic models para trayectorias
│   ├── dependencies.py       # Inyección de RobotState en los routers
│   └── main.py               # App FastAPI — lifespan, routers, StaticFiles
├── core/
│   ├── state.py              # RobotState — única fuente de verdad del robot
│   └── utils.py              # Utilidades generales
├── kinematics/
│   ├── dh.py                 # Matrices de transformación Denavit-Hartenberg
│   ├── forward.py            # Cinemática directa
│   └── inverse.py            # Cinemática inversa (geométrica + desacoplamiento de muñeca)
├── trajectory/
│   ├── ptp.py                # Punto a punto — perfil polinomial grado 4
│   ├── linear.py             # Trayectoria lineal cartesiana
│   └── circular.py           # Arco circular por 3 puntos
├── frontend/
│   ├── index.html            # Estructura HTML
│   ├── styles.css            # Estilos
│   └── js/
│       ├── config.js         # URLs dinámicas (API, WS, escala 3D)
│       ├── robot3d.js        # Escena Three.js y meshes del robot
│       ├── ui.js             # Sliders, status bar, indicador WS
│       ├── api.js            # Wrappers fetch por endpoint REST
│       ├── websocket.js      # Ciclo de vida de la conexión WebSocket
│       └── main.js           # Entrada — applyState(), wiring de eventos
├── tests/
│   ├── test_kinematics.py
│   ├── test_inverse.py
│   ├── test_ptp.py
│   ├── test_trajectories.py
│   ├── test_state.py
│   └── test_api.py
├── matlab-legacy/            # Código MATLAB original (referencia)
└── requirements.txt
```

## Geometría del robot

Robot articulado de 6 DOF con parámetros DH:

| Eslabón | θ | d | a | α |
|---------|---|---|---|---|
| 1 | q1 | L1=5 | 0 | π/2 |
| 2 | q2 | 0 | L2=5 | 0 |
| 3 | q3 | 0 | L3=5 | 0 |
| 4 | 0 | 0 | 0 | q4 |
| 5 | q5 | 0 | 0 | −π/2 |
| 6 | q6 | 0 | 0 | 0 |

Posición home (todos los ángulos en 0°): EF en `(px=10, py=0, pz=5)`.

## Instalación

```bash
# 1. Clonar el repositorio
git clone https://github.com/AdrianCervantes53/robot-trajectory-sim
cd robot-trajectory-sim

# 2. Crear entorno virtual
python -m venv .venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate

# 3. Instalar dependencias
pip install -r requirements.txt

# 4. Correr los tests
pytest tests/ -v

# 5. Levantar la API
uvicorn api.main:app --host 0.0.0.0 --port 8000 --reload

# 6. Abrir el frontend
# http://localhost:8000/app        ← misma máquina
# http://<tu-IP>:8000/app         ← cualquier dispositivo en la red
# Documentación interactiva: http://localhost:8000/docs
```

## API — Endpoints

### REST

| Método | Ruta | Descripción |
|--------|------|-------------|
| `GET` | `/state` | Estado completo del robot |
| `POST` | `/joints` | Mover a ángulos de junta `[q1..q6]` en grados |
| `POST` | `/pose` | Mover a pose cartesiana `{px, py, pz}` vía IK |
| `POST` | `/gripper` | Abrir/cerrar gripper `{open: bool}` |
| `POST` | `/config` | Configurar `velocity_pct` y `trajectory_duration` |
| `POST` | `/trajectory/save` | Guardar trayectoria `{name}` |
| `POST` | `/trajectory/load` | Cargar trayectoria `{name}` |
| `POST` | `/trajectory/clear` | Limpiar historial |
| `GET` | `/trajectory/list` | Listar trayectorias guardadas |

### WebSocket `/ws`

El cliente envía un mensaje JSON y el servidor emite un frame por cada paso de la trayectoria:

```json
// PTP — punto a punto en espacio de juntas
{ "type": "ptp", "q_end": [45, 30, 20, 0, 0, 0] }

// Lineal — interpolación cartesiana
{ "type": "linear", "p_end": [8, 3, 7] }

// Circular — arco por punto medio y punto final
{ "type": "circular", "p_mid": [4, 4, 5], "p_end": [7, 0, 5], "plane": 1 }
```

Cada frame emitido:

```json
{
  "joints_deg": [q1, q2, q3, q4, q5, q6],
  "position":   { "px": 0.0, "py": 0.0, "pz": 0.0 },
  "links": [
    { "from": [0, 0, 0],    "to": [x1, y1, z1] },
    { "from": [x1, y1, z1], "to": [x2, y2, z2] },
    { "from": [x2, y2, z2], "to": [px, py, pz]  }
  ],
  "trajectory": [{ "px": 0.0, "py": 0.0, "pz": 0.0 }, "..."],
  "frame_type": "trajectory_frame | trajectory_end | error"
}
```

## Tests

```bash
pytest tests/ -v              # todos los tests
pytest tests/test_api.py      # solo la API
pytest tests/test_kinematics.py tests/test_inverse.py  # solo cinemática
```

101 tests cubriendo:
- Matrices DH y cinemática directa
- Round-trip FK→IK→FK para validación de cinemática inversa
- Perfil polinomial PTP (condiciones de contorno, monotonía)
- Trayectorias LIN y CIR (circuncentro, dirección del arco)
- Serialización JSON de `RobotState`
- Todos los endpoints REST y el protocolo WebSocket

## Decisiones de diseño

**Sin álgebra simbólica.** El MATLAB original usaba `syms`/`subs()` para perfiles PTP y `solve()` para la trayectoria circular. Reemplazados con coeficientes NumPy y la fórmula del circuncentro — sin SymPy, sin overhead simbólico.

**Estado centralizado.** El workspace global de MATLAB (`evalin`/`assignin`) se reemplaza con `RobotState`, el único objeto que la API manipula. Diseñado para un cliente a la vez (suficiente para demo y portafolio); escalar a multi-usuario requeriría estado por sesión.

**Generadores para trayectorias.** PTP, LIN y CIR son generadores Python (`yield`). El WebSocket los consume y emite cada frame conforme se calcula — sin acumular toda la trayectoria en memoria.

**Frontend modular sin build step.** El frontend está estructurado en módulos ES nativos (`type="module"`): `robot3d.js` (escena Three.js), `api.js` (cliente REST), `websocket.js` (ciclo WS), `ui.js` (DOM) y `main.js` (orquestador). Cero dependencias locales, cero configuración de bundler.

**URLs dinámicas.** La dirección de la API y el WebSocket se derivan de `window.location.hostname` en tiempo de ejecución, por lo que el simulador funciona desde cualquier dispositivo en la misma red sin ningún cambio de configuración.

**Separación de responsabilidades.** `fcdirecta.m` mezclaba cinemática, graficado y control de Arduino. Ahora son capas independientes: `kinematics/` (matemática pura), `api/` (comunicación HTTP/WS), `frontend/` (visualización 3D).
