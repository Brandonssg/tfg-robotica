"""Primitiva 2: detección de objeto oscuro (canal Value, no Hue).

El cono negro no tiene tono definido (S≈0): filtrar por H es inútil y se
filtra por V (brillo). El problema es que el suelo del almacén tiene motas
oscuras que también pasan el filtro V; la separación no la da el umbral de
brillo sino el ANÁLISIS DE COMPONENTES CONEXAS: el cono es una mancha oscura
grande y contigua, las motas son pequeñas y dispersas.

Umbrales empíricos (640x360): componente mayor con cono = 5628 px;
sin cono (solo motas/fondo) = 93 px. Umbral de área: 1000 px (margen >5x
en ambos sentidos).
"""
from comun import cargar, resultado, main_cli, superponer_mascara, dibujar_etiqueta
import cv2, numpy as np

def _componentes(img, v_max):
    v = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)[..., 2]
    mask = (v < v_max).astype(np.uint8)
    # apertura morfológica: elimina las motas del suelo antes de medir áreas
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask)
    idx_mayor = int(np.argmax(stats[1:, 4]) + 1) if n > 1 else None
    return lab, stats, idx_mayor

def detectar(img, v_max=40, area_min=1000):
    _, stats, idx = _componentes(img, v_max)
    area = int(stats[idx, 4]) if idx is not None else 0
    return resultado(area >= area_min, "objeto_oscuro" if area >= area_min else "ninguno",
                     area, area_min)

def anotar(img, res, v_max=40, area_min=1000):
    """Superpone SOLO la componente conexa ganadora (no todos los píxeles
    oscuros) — así se ve por qué gana ella y no una mota del suelo."""
    lab, _, idx = _componentes(img, v_max)
    mask = (lab == idx) if idx is not None else np.zeros(lab.shape, bool)
    out = superponer_mascara(img, mask, (0, 0, 255))
    return dibujar_etiqueta(out, res)

if __name__ == "__main__":
    main_cli(detectar, {}, anotar=anotar)
