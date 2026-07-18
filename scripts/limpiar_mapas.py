#!/usr/bin/env python3
"""Postprocesado de mapas de ocupación (PGM) generados por SLAM Toolbox.

Reasigna cualquier valor de píxel al valor trinario más cercano:
  0 (ocupado), 205 (desconocido), 254 (libre).
Criterio conservador: los valores intermedios caen a 205 (desconocido),
nunca a libre, para no crear zonas transitables falsas.

Uso:
    python3 limpiar_mapas.py entrada.pgm salida.pgm
"""

import sys
import numpy as np
from PIL import Image

UMBRAL_LIBRE = 230    # >= 230 se considera libre (254)
UMBRAL_OCUPADO = 50   # <= 50 se considera ocupado (0)


def limpiar(ruta_entrada: str, ruta_salida: str) -> None:
    a = np.array(Image.open(ruta_entrada))
    print(f"Valores de entrada: {np.unique(a)}")

    limpio = np.where(
        a >= UMBRAL_LIBRE, 254,
        np.where(a <= UMBRAL_OCUPADO, 0, 205)
    ).astype(np.uint8)

    Image.fromarray(limpio).save(ruta_salida)
    print(f"Valores de salida:  {np.unique(limpio)}")
    print(f"Guardado en {ruta_salida}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(f"Uso: {sys.argv[0]} entrada.pgm salida.pgm")
    limpiar(sys.argv[1], sys.argv[2])