"""
ptp.py
Trayectoria punto a punto (PTP) con perfil polinomial de 4to orden.

MATLAB equivalentes: PTPpoli.m + PTP.m

Perfil de movimiento
--------------------
El movimiento se divide en 3 etapas simétricas:
  1. Aceleración  [0,      T/4]   → polinomio de grado 4
  2. Velocidad    [T/4,  3T/4]   → lineal (velocidad constante)
  3. Desaceleración [3T/4,   T]  → polinomio de grado 4 espejo

El tiempo total T se calcula a partir de la articulación que más se mueve
(dismax) y la velocidad configurada. Cada junta se interpola de forma
independiente y sincronizada para llegar al destino al mismo tiempo.

Diferencia clave respecto al MATLAB original
--------------------------------------------
MATLAB usa `syms t` y `subs()` para evaluar los polinomios en cada paso.
Aquí se calculan los coeficientes numéricos una vez y se evalúan con
np.polyval() — sin álgebra simbólica, mucho más rápido.
"""

import numpy as np
from dataclasses import dataclass
from typing import Generator

from kinematics.forward import forward_kinematics, ForwardKinematicsResult

# Velocidad angular máxima de las juntas (rad/s)
VEL_MAX: float = 2 * np.pi


@dataclass
class PTPCoeffs:
    """
    Coeficientes del perfil polinomial para una junta.

    Atributos
    ---------
    accel : np.ndarray
        Coeficientes [a4, a3, a2, a1, a0] de la etapa de aceleración.
        Se evalúan con np.polyval(accel, t).
    vel : tuple[float, float]
        Coeficientes (m, b) de la rampa lineal: pos = m*(t - dt) + po2.
    decel : tuple[np.ndarray, float]
        (coefs, t2) — coeficientes del polinomio de desaceleración y el
        instante de inicio t2. Se evalúan con np.polyval(decel, t - t2).
    dt : float
        Duración de cada etapa (T/4).
    t2 : float
        Instante de inicio de la etapa de desaceleración (3T/4).
    """
    accel: np.ndarray
    vel_m: float
    vel_po2: float
    vel_dt: float
    decel: np.ndarray
    dt: float
    t2: float


def _poly4_coeffs(
    po: float, pf: float, vo: float, vf: float, T_seg: float
) -> np.ndarray:
    """
    Calcula coeficientes del polinomio de grado 4 para una etapa.

    Restricciones: posición inicial/final, velocidad inicial/final,
    aceleración inicial = 0.

    Retorna coeficientes en orden descendente [a4, a3, a2, a1, a0]
    para usar con np.polyval.
    """
    t1 = T_seg
    a0 = po
    a1 = vo
    a2 = 0.0  # aceleración inicial = 0
    a3 = (1 / (2 * t1**3)) * (20 * (pf - po) - (8 * vf + 12 * vo) * t1)
    a4 = (1 / (2 * t1**4)) * (30 * (po - pf) + (14 * vf + 16 * vo) * t1)
    # np.polyval espera orden descendente: a4*t^4 + a3*t^3 + ...
    return np.array([a4, a3, a2, a1, a0])


def _build_profile(po: float, pf: float, T: float) -> PTPCoeffs:
    """
    Construye el perfil polinomial completo para una junta.

    Parámetros
    ----------
    po : float   Posición inicial (radianes).
    pf : float   Posición final (radianes).
    T  : float   Tiempo total de la trayectoria (segundos).
    """
    dt = T * 0.25          # duración de cada etapa (T/4 en MATLAB era T*0.125*2)
    t1 = 2 * dt            # fin de aceleración = T/2... ajustado a T/4 como MATLAB
    t2 = T - 2 * dt        # inicio de desaceleración

    # Velocidad de crucero (pendiente de la rampa lineal)
    m = (pf - po) / (T - 2 * dt) if (T - 2 * dt) > 1e-10 else 0.0

    # Posición al final de la aceleración y al inicio de la desaceleración
    po2 = m * (t1 - dt) + po   # = m*dt + po
    pf2 = m * (t2 - dt) + po   # posición al inicio de desaceleración

    # Etapa 1: aceleración (0 → t1)
    accel = _poly4_coeffs(po, po2, 0.0, m, t1)

    # Etapa 3: desaceleración (t2 → T), evaluada con tiempo local (t - t2)
    decel = _poly4_coeffs(pf2, pf, m, 0.0, T - t2)

    return PTPCoeffs(
        accel=accel,
        vel_m=m,
        vel_po2=po2,
        vel_dt=dt,
        decel=decel,
        dt=t1,        # fin de aceleración
        t2=t2,        # inicio de desaceleración
    )


def _eval_profile(coeffs: PTPCoeffs, t: float) -> float:
    """Evalúa el perfil en el instante t."""
    if t <= coeffs.dt:
        return float(np.polyval(coeffs.accel, t))
    elif t <= coeffs.t2:
        return coeffs.vel_m * (t - coeffs.vel_dt) + coeffs.vel_po2
    else:
        return float(np.polyval(coeffs.decel, t - coeffs.t2))


def ptp_trajectory(
    q_start_deg: list,
    q_end_deg: list,
    velocity_pct: float = 50.0,
    dt: float = 0.1,
) -> Generator[ForwardKinematicsResult, None, None]:
    """
    Genera la trayectoria PTP como secuencia de estados FK.

    Parámetros
    ----------
    q_start_deg : list[float]
        Ángulos iniciales [q1..q6] en GRADOS.
    q_end_deg : list[float]
        Ángulos finales [q1..q6] en GRADOS.
    velocity_pct : float
        Velocidad como porcentaje 0-100. Equivale al slider de MATLAB.
        100% = movimiento más lento (más tiempo), 0% = más rápido.
        (Misma lógica invertida que el original: Vell = abs(Vel - 100))
    dt : float
        Paso de tiempo en segundos.

    Yields
    ------
    ForwardKinematicsResult
        Estado completo del robot en cada instante de la trayectoria.
        Listo para serializar y enviar por WebSocket.
    """
    # Solo se interpolan las 3 primeras juntas (brazo), igual que en PTP.m
    # q4, q5, q6 se mantienen constantes durante el movimiento PTP
    q_start = np.deg2rad(q_start_deg[:3])
    q_end   = np.deg2rad(q_end_deg[:3])
    q4, q5, q6 = q_start_deg[3], q_start_deg[4], q_start_deg[5]

    # Tiempo total: junta con mayor recorrido determina T.
    # vel_factor va de 0.0 (velocity_pct=100, más rápido) a
    # 1.0 (velocity_pct=0, más lento). Se escala por T_BASE para
    # que el movimiento dure entre ~0.3s y ~3s — rango útil para animación.
    T_BASE = 3.0  # segundos máximos a velocidad mínima
    vel_factor = abs(velocity_pct - 100) / 100  # 0.0 → rápido, 1.0 → lento
    dismax = np.max(np.abs(q_end - q_start))
    # Normaliza el recorrido respecto al máximo posible (pi rad) para escalar T
    dist_norm = dismax / np.pi if dismax > 1e-10 else 0.0
    T = T_BASE * vel_factor * dist_norm
    T = max(T, 0.1)  # mínimo 100ms para evitar T=0

    # Construir perfil para cada junta del brazo
    profiles = [_build_profile(q_start[i], q_end[i], T) for i in range(3)]

    # Iterar en el tiempo y emitir cada frame.
    # Se garantiza siempre un frame final exacto en t=T para que el robot
    # llegue precisamente al destino independientemente del paso dt.
    t = 0.0
    last_emitted_T = False
    while True:
        t_clamped = min(t, T)
        q1 = np.rad2deg(_eval_profile(profiles[0], t_clamped))
        q2 = np.rad2deg(_eval_profile(profiles[1], t_clamped))
        q3 = np.rad2deg(_eval_profile(profiles[2], t_clamped))

        yield forward_kinematics(q1, q2, q3, q4, q5, q6)

        if t >= T:
            break
        t = min(t + dt, T)