"""Utilidades comunes del paquete de visión (Fase 4).

Interfaz común de todos los detectores:
    detectar(img_bgr, **params) -> dict con claves:
        detectado (bool)  - si la señal buscada está presente
        etiqueta  (str)   - qué se ha detectado ("boya_roja", "cono_negro"...)
        medida    (float) - magnitud medida (%, píxeles...), para trazabilidad
        umbral    (float) - umbral aplicado, para trazabilidad
Cada módulo es además ejecutable standalone:  python3 deteccion_X.py foto.png
"""
import cv2, sys, numpy as np
from pathlib import Path

# Todas las fotos se reducen a esta resolución antes de procesar:
# - los umbrales de área (px) son así independientes de la resolución de la cámara
# - el coste de proceso baja ~9x respecto a 1920x1080 sin perder señal útil
TAMANO = (640, 360)

# Subcarpeta donde se guardan las fotos con la detección superpuesta.
# No es dato de proyecto (se regenera a partir de fotos_waypoints/), va en .gitignore.
DIR_ANALIZADAS = "analizadas"

def cargar(ruta):
    img = cv2.imread(str(ruta))
    if img is None:
        raise FileNotFoundError(f"No se puede leer la imagen: {ruta}")
    return cv2.resize(img, TAMANO)

def resultado(detectado, etiqueta, medida, umbral):
    return {"detectado": bool(detectado), "etiqueta": etiqueta,
            "medida": round(float(medida), 3), "umbral": umbral}

def superponer_mascara(img, mask, color=(0, 0, 255), alpha=0.45):
    """Tinta en color semitransparente los píxeles activos de una máscara binaria.
    Es la parte visual de cada detector: muestra EXACTAMENTE los píxeles que
    han producido la medida, no solo el resultado final."""
    capa = np.zeros_like(img)
    capa[mask.astype(bool)] = color
    return cv2.addWeighted(capa, alpha, img, 1 - alpha, 0)

def dibujar_etiqueta(img, res):
    """Franja inferior con etiqueta/medida/umbral. Común a los 4 detectores,
    así toda foto analizada lleva la misma cabecera de lectura."""
    out = img.copy()
    h, w = out.shape[:2]
    # verde si hay señal detectada, gris si no — neutro respecto a si eso es
    # bueno o malo (wp01 rojo = aviso; wp06 chaleco = aviso; wp08 rojo = OK)
    color = (60, 170, 60) if res["detectado"] else (95, 95, 95)
    cv2.rectangle(out, (0, h - 26), (w, h), color, -1)
    texto = f"{res['etiqueta']}  medida={res['medida']}  umbral={res['umbral']}"
    cv2.putText(out, texto, (6, h - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (255, 255, 255), 1, cv2.LINE_AA)
    return out

def guardar_anotada(img_anotada, ruta_original):
    """Guarda junto al original, en <mismo_directorio>/analizadas/<nombre>_analizada.png"""
    ruta_original = Path(ruta_original)
    destino_dir = ruta_original.parent / DIR_ANALIZADAS
    destino_dir.mkdir(parents=True, exist_ok=True)
    destino = destino_dir / f"{ruta_original.stem}_analizada.png"
    cv2.imwrite(str(destino), img_anotada)
    return destino

def main_cli(detectar, params_defecto, anotar=None):
    """CLI standalone: valida un detector contra una foto sin levantar el simulador.
    Si el módulo define `anotar`, guarda además la foto con la detección superpuesta."""
    if len(sys.argv) < 2:
        print(f"Uso: python3 {sys.argv[0]} <foto.png>"); sys.exit(1)
    ruta = sys.argv[1]
    img = cargar(ruta)
    res = detectar(img, **params_defecto)
    print(res)
    if anotar is not None:
        destino = guardar_anotada(anotar(img, res, **params_defecto), ruta)
        print(f"Figura guardada en: {destino}")
    sys.exit(0 if res["detectado"] else 2)
