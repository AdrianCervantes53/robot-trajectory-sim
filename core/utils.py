"""
utils.py
Utilidades generales del simulador.

MATLAB equivalente: red.m
"""


def red(x: float) -> float:
    """Redondea a 2 decimales. Equivalente a red.m de MATLAB."""
    return round(x * 100) / 100