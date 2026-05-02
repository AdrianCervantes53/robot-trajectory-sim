# Robot 6-DOF Simulator

Simulador cinemático de un brazo robótico de 6 grados de libertad, portado desde MATLAB a Python con una API REST/WebSocket y un frontend 3D en el browser.

## Stack

| Capa | Tecnología |
|------|-----------|
| Cinemática | Python + NumPy |
| API | FastAPI + WebSockets |
| Frontend | Three.js (sin build tools) |
| Tests | Pytest — 101 tests |

## Estructura

```
robot_sim/
├── core/
│   ├── utils.py          # Utilidades (red)
│   └── state.py          # Estado global del robot
├── kinematics/
│   ├── dh.py             # Matriz Denavit-Hartenberg
│   ├── forward.py        # Cinemática directa
│   └── inverse.py        # Cinemática inversa (geométrica + desacoplamiento de muñeca)
├── trajectory/
│   ├── ptp.py            # Punto a punto — perfil polinomial grado 4
│   ├── linear.py         # Trayectoria lineal cartesiana
│   └── circular.py       # Arco circular por 3 puntos
├── api/
│   ├── routers               # Endpoints
│   │   ├── movement.py       #
│   │   ├── trajectory.py     #
│   │   └── ws_trajectory.py  # 
│   ├── schemas               # Esquemas de pydantic
│   │   ├── movement.py       #
│   │   └── trajectory.py     # 
│   ├── dependencies.py   # Dependencias del estado del robot
│   └── main.py           # FastAPI REST + WebSocket
├── frontend/
│   └── index.html        # Interfaz 3D (Three.js, sin dependencias locales)
└── tests/
    ├── test_kinematics.py
    ├── test_inverse.py
    ├── test_ptp.py
    ├── test_trajectories.py
    ├── test_state.py
    └── test_api.py
```

## Geometría del robot

Robot articulado de 6 DOF con parámetros DH:

| Eslabón | θ | d | a | α |
|---------|---|---|---|---|
| 1 | q1 | L1=5 | 0 | π/2 |
| 2 | q2 | 0 | L2=5 | 0 |
| 3 | q3 | 0 | L3=5 | 0 |
| 4 | 0 | 0 | 0 | q4 |
| 5 | q5 | 0 | 0 | -π/2 |
| 6 | q6 | 0 | 0 | 0 |

Posición home (todos los ángulos en 0°): EF en `(px=10, py=0, pz=5)`.

## Instalación

```bash
# 1. Clonar el repositorio
git clone https://github.com/AdrianCervantes53/robot-sim
cd robot-sim

# 2. Crear entorno virtual
python -m venv .venv
source .venv/bin/activate   # Windows: .venv\Scripts\activate

# 3. Instalar dependencias
pip install fastapi uvicorn websockets numpy pytest httpx

# 4. Correr tests
pytest robot_sim/tests/ -v

# 5. Levantar la API
uvicorn robot_sim.api.main:app --reload

# 6. Abrir el frontend
# Abrir en el browser: http://localhost:8000/app
# O directamente el archivo: robot_sim/frontend/index.html
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

Documentación interactiva en: `http://localhost:8000/docs`

### WebSocket `/ws`

El cliente envía un mensaje JSON y el servidor emite un frame por cada paso de la trayectoria:

```json
// PTP — punto a punto
{ "type": "ptp", "q_end": [45, 30, 20, 0, 0, 0] }

// Lineal — interpolación cartesiana
{ "type": "linear", "p_end": [8, 3, 7] }

// Circular — arco por 3 puntos
{ "type": "circular", "p_mid": [4, 4, 5], "p_end": [7, 0, 5], "plane": 1 }
```

Cada frame emitido:

```json
{
  "joints_deg": [q1, q2, q3, q4, q5, q6],
  "position":   { "px": 0.0, "py": 0.0, "pz": 0.0 },
  "links": [
    { "from": [0,0,0], "to": [x1,y1,z1] },
    { "from": [x1,y1,z1], "to": [x2,y2,z2] },
    { "from": [x2,y2,z2], "to": [px,py,pz] }
  ],
  "trajectory": [{"px":..., "py":..., "pz":...}, ...],
  "frame_type": "trajectory_frame" | "trajectory_end" | "error"
}
```

## Tests

```bash
pytest robot_sim/tests/ -v          # todos los tests
pytest robot_sim/tests/test_api.py  # solo la API
```

101 tests cubriendo:
- Matrices DH y cinemática directa
- Round-trip FK→IK→FK para validación de cinemática inversa
- Perfil polinomial PTP (inicio, fin, monotonía)
- Trayectorias LIN y CIR (circuncentro, dirección del arco)
- Serialización JSON del estado
- Todos los endpoints REST + WebSocket

## Decisiones de diseño

**Sin álgebra simbólica**: El MATLAB original usaba `syms` y `subs()` para los perfiles PTP y `solve()` para la trayectoria circular. Reemplazados con coeficientes numpy y la fórmula del circuncentro — sin sympy, más rápido.

**Estado centralizado**: El workspace global de MATLAB (`evalin`/`assignin`) se reemplaza con `RobotState`, el único objeto que la API manipula.

**Generadores para trayectorias**: PTP, LIN y CIR son generadores Python (`yield`). La API los consume en el WebSocket y emite cada frame conforme se calcula — sin guardar toda la trayectoria en memoria.

**Separación de responsabilidades**: `fcdirecta.m` mezclaba cinemática, graficado y control de Arduino. Ahora son tres capas independientes: `kinematics/forward.py` (matemática pura), `api/main.py` (comunicación), `frontend/index.html` (visualización).