"""Primitiva 4: comparación con imagen de referencia (diff).

Problema encontrado en la validación offline: la repetibilidad de pose de
AMCL entre misiones (± cm y ± grados) desplaza el encuadre lo bastante como
para que el diff píxel a píxel directo sea inservible — entre dos fotos del
MISMO estado el diff crudo daba 10-32 %, del orden de la señal real.

Mitigación en dos pasos:
 1. Registro ECC (findTransformECC, modelo euclídeo): estima la traslación+
    rotación que alinea la foto actual sobre la referencia y la compensa.
    Reduce el ruido de mismo-estado a 4-6 % en vistas de campo lejano.
 2. Recorte de bordes (franjas negras del warp) y de la banda inferior
    (suelo cercano, donde el paralaje es máximo y el ECC no puede
    compensarlo: es un efecto 3D y el warp es 2D).

Límite conocido (documentar en memoria): en vistas con objetos muy cercanos
(p. ej. los pilares de wp08) el paralaje domina y ni el ECC lo corrige
(ruido mismo-estado 24 % > señal 16 %). Para esos waypoints se usa una
primitiva de color en su lugar. El diff queda para vistas de campo
medio/lejano: wp02 (señal 51-52 % frente a ruido 4-6 %) y wp03.
"""
from comun import cargar, resultado, main_cli, superponer_mascara, dibujar_etiqueta
import cv2, numpy as np

# margen de recorte reutilizado por detectar() y anotar() — debe ser idéntico
# en ambos o la máscara no coincidiría con lo que se muestra
BORDE, BANDA_INFERIOR = 30, 80

def _alinear_y_recortar(img, referencia):
    ref_color = cargar(referencia)
    ref, act = cv2.cvtColor(ref_color, cv2.COLOR_BGR2GRAY), cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    warp = np.eye(2, 3, dtype=np.float32)
    act_color_alineada = img
    try:
        _, warp = cv2.findTransformECC(
            ref, act, warp, cv2.MOTION_EUCLIDEAN,
            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 1e-5), None, 5)
        act = cv2.warpAffine(act, warp, (ref.shape[1], ref.shape[0]),
                             flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
        act_color_alineada = cv2.warpAffine(img, warp, (ref.shape[1], ref.shape[0]),
                             flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
    except cv2.error:
        pass  # sin convergencia: se compara sin alinear (peor, pero no falla)
    h, w = ref.shape
    recorte = (slice(BORDE, h - BANDA_INFERIOR), slice(BORDE, w - BORDE))
    return ref[recorte], act[recorte], act_color_alineada[recorte], recorte

def _mascara_diff(ref, act, umbral_gris):
    ref, act = [cv2.GaussianBlur(x, (9, 9), 0) for x in (ref, act)]
    d = (cv2.absdiff(ref, act) > umbral_gris).astype(np.uint8)
    return cv2.morphologyEx(d, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

def detectar(img, referencia=None, umbral_pct=25.0, umbral_gris=40):
    ref, act, _, _ = _alinear_y_recortar(img, referencia)
    mask = _mascara_diff(ref, act, umbral_gris)
    pct = 100.0 * mask.sum() / mask.size
    return resultado(pct > umbral_pct, "cambio_vs_referencia" if pct > umbral_pct else "sin_cambio",
                     pct, umbral_pct)

def anotar(img, res, referencia=None, umbral_pct=25.0, umbral_gris=40):
    """Superpone la máscara de diff sobre la foto YA ALINEADA (no la original):
    solo así la mancha roja coincide con el recorte que realmente se mide,
    sin el desplazamiento que introduciría el registro ECC."""
    ref, act, act_color, _ = _alinear_y_recortar(img, referencia)
    mask = _mascara_diff(ref, act, umbral_gris)
    out = superponer_mascara(act_color, mask, (0, 0, 255))
    return dibujar_etiqueta(out, res)

if __name__ == "__main__":
    main_cli(detectar, {"referencia": "referencias/wp02_ref.png"}, anotar=anotar)
