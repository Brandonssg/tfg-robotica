"""Primitiva 1: detección por color (canal Hue del espacio HSV).

Se usa HSV en lugar de RGB porque separa el tono (H) de la iluminación (V):
el rojo de una boya es el mismo H con sombra o sin ella, mientras que en RGB
los tres canales cambian a la vez con la luz.

Dos modos de uso:
 - detectar():        mide el %% de píxeles del color -> presencia/ausencia
 - detectar_conteo(): cuenta COMPONENTES CONEXAS del color -> nº de elementos.
   Robusto frente a giros/desplazamientos de la cámara: un elemento sigue
   siendo una componente aunque cambie de posición en el encuadre, cosa que
   el diff con referencia no soporta (validado en la 2ª ronda de fotos:
   una foto girada con 1 solo elemento daba diff 50.9%% "cambio", el conteo
   da n=1 "faltan elementos" — correcto y por el motivo correcto).

Particularidad del rojo: en OpenCV H va de 0 a 179 y el rojo queda partido en
los dos extremos (0-8 y 172-179), por lo que un rango con h_lo > h_hi se
interpreta como "envolvente" (wrap-around).

Umbrales empíricos (validados offline contra fotos_waypoints/, 640x360):
  %% de píxeles:  boyas 2.07-2.35%% vs 0 | chaleco lima 0.26-0.32%% vs 0
                 rojo wp08 2.16-2.61%% vs 0
  conteo wp02 (rojo, area>=25px): A=3 (2 toolbox + extintor), B=1, C=0
    en las tres misiones analizadas; áreas reales 854-1897 px
  conteo wp03 (azul mesitas, area>=25px): presentes=4 paneles (2 por
    carrito), ausentes=0; el cielo (S=99, V=243 uniforme) queda excluido
    por el tope V<200
"""
from comun import cargar, resultado, main_cli, superponer_mascara, dibujar_etiqueta
import cv2, numpy as np

# rango por nombre: h=(lo,hi) [envolvente si lo>hi], s=(min,max), v=(min,max)
# umbral_pct solo aplica al modo detectar()
RANGOS = {
    "rojo":  dict(h=(172,   8), s=(120, 256), v=( 60, 256), umbral_pct=0.8),
    "verde": dict(h=( 45,  85), s=(100, 256), v=( 50, 256), umbral_pct=0.8),
    "lima":  dict(h=( 22,  50), s=( 90, 256), v=(120, 256), umbral_pct=0.12),
    # azul de las mesitas/carritos: V<200 excluye el cielo (V=243 uniforme)
    "azul":  dict(h=( 90, 130), s=(120, 256), v=( 40, 200), umbral_pct=0.5),
}
COLOR_OVERLAY = {"rojo": (0, 0, 255), "verde": (0, 200, 0),
                 "lima": (0, 220, 220), "azul": (255, 120, 0)}

def _mascara(hsv, nombre):
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    r = RANGOS[nombre]
    h_lo, h_hi = r["h"]
    m_h = ((h >= h_lo) | (h <= h_hi)) if h_lo > h_hi else ((h >= h_lo) & (h <= h_hi))
    return (m_h & (s > r["s"][0]) & (s < r["s"][1])
                & (v > r["v"][0]) & (v < r["v"][1]))

def _componentes(img, color, area_min):
    mask = _mascara(cv2.cvtColor(img, cv2.COLOR_BGR2HSV), color).astype(np.uint8)
    # apertura: elimina píxeles sueltos; cierre: une partes del mismo elemento
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask)
    cajas = [tuple(stats[i, :4]) for i in range(1, n) if stats[i, 4] >= area_min]
    return cajas, mask

# ------------------------- modo porcentaje -----------------------------------

def detectar(img, colores=("rojo", "verde")):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mejor = resultado(False, "ninguno", 0.0, 0.0)
    for nombre in colores:
        pct = 100.0 * _mascara(hsv, nombre).sum() / hsv[..., 0].size
        umbral = RANGOS[nombre]["umbral_pct"]
        if pct > umbral and pct > mejor["medida"]:
            mejor = resultado(True, nombre, pct, umbral)
        elif not mejor["detectado"]:
            mejor["medida"] = max(mejor["medida"], pct)
    return mejor

def anotar(img, res, colores=("rojo", "verde")):
    """Pinta en su color real la máscara del rango que ha producido la medida."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    nombre = res["etiqueta"] if res["etiqueta"] in RANGOS else colores[0]
    out = superponer_mascara(img, _mascara(hsv, nombre), COLOR_OVERLAY[nombre])
    return dibujar_etiqueta(out, res)

# --------------------------- modo conteo -------------------------------------

def detectar_conteo(img, color="rojo", esperados=3, area_min=25):
    """detectado=True si hay al menos `esperados` componentes del color.
    medida = nº de componentes encontradas (permite distinguir 0/1/2/3...)."""
    cajas, _ = _componentes(img, color, area_min)
    n = len(cajas)
    return resultado(n >= esperados, color if n >= esperados else "ninguno",
                     n, esperados)

def anotar_conteo(img, res, color="rojo", esperados=3, area_min=25):
    """Máscara superpuesta + un rectángulo por componente contada."""
    cajas, mask = _componentes(img, color, area_min)
    out = superponer_mascara(img, mask.astype(bool), COLOR_OVERLAY[color])
    for x, y, w, h in cajas:
        cv2.rectangle(out, (x, y), (x + w, y + h), (255, 255, 255), 2)
    return dibujar_etiqueta(out, res)

if __name__ == "__main__":
    main_cli(detectar, {"colores": ("rojo", "verde", "lima")}, anotar=anotar)
