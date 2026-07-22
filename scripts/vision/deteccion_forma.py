"""Primitiva 3: detección por forma/contorno (cono de obra en wp04).

El cono de obra no puede detectarse por color puro: sus franjas salmón son
poco saturadas y el naranja intenso colisionaría con las vigas y estanterías
del fondo (naranja ocupa un 16-30 % de la escena en wp04). La firma que sí lo
distingue es geométrica: franjas horizontales claras (blanco/salmón)
apiladas, cada una más ancha que alta.

Método: máscara de franjas salmón claras y poco saturadas -> cierre
morfológico (une franja y borde) -> componentes conexas -> se cuentan solo
las componentes con área suficiente y proporción horizontal (ancho > alto,
descarta reflejos verticales).

Empírico (640x360): con conos = 4 componentes de 282-516 px; sin conos = 0.
"""
from comun import cargar, resultado, main_cli, dibujar_etiqueta
import cv2, numpy as np

def _franjas(img, area_min):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0].astype(int), hsv[..., 1].astype(int), hsv[..., 2].astype(int)
    # franja salmón: tono rojizo-naranja CLARO (v alto) y saturación MEDIA
    # (s 60-160) — el naranja intenso de vigas/estanterías queda fuera por s>160
    mask = ((h <= 15) & (s > 60) & (s < 160) & (v > 150)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    n, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    cajas = [tuple(stats[i, :4]) for i in range(1, n)
             if stats[i, 4] >= area_min and stats[i, 2] > stats[i, 3]]  # w > h
    return cajas

def detectar(img, area_min=150, franjas_min=2):
    franjas = len(_franjas(img, area_min))
    return resultado(franjas >= franjas_min, "cono_obra" if franjas >= franjas_min else "ninguno",
                     franjas, franjas_min)

def anotar(img, res, area_min=150, franjas_min=2):
    """Dibuja un rectángulo sobre cada franja horizontal contada como parte
    del cono — permite ver a simple vista si el conteo tiene sentido."""
    out = img.copy()
    for x, y, w, hh in _franjas(img, area_min):
        cv2.rectangle(out, (x, y), (x + w, y + hh), (0, 140, 255), 2)
    return dibujar_etiqueta(out, res)

if __name__ == "__main__":
    main_cli(detectar, {}, anotar=anotar)
