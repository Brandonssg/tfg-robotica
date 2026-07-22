"""Despachador de inspección: tabla declarativa waypoint -> detector + parámetros.

Mismo patrón que cargar_escenario.py: la lógica vive en una tabla de datos,
no en código; añadir o cambiar la inspección de un waypoint es editar una
entrada, no tocar los detectores (multiplexor: la señal wpNN selecciona qué
módulo procesa la foto).

Uso:
  python3 scripts/vision/inspeccionar.py foto.png     # una foto suelta
  python3 scripts/vision/inspeccionar.py directorio/  # última foto de cada wp
  python3 scripts/vision/inspeccionar.py              # ídem sobre fotos_waypoints/

En modo misión (directorio) genera además un INFORME de inspección con el
estado de los 9 waypoints en <directorio>/analizadas/informe_<fecha>.txt.
El waypoint se deduce del prefijo wpNN del nombre de fichero (el formato que
ya genera ruta_waypoints_foto.py).
"""
import sys, re
from datetime import datetime
from pathlib import Path
import deteccion_hue, deteccion_value, deteccion_forma, deteccion_referencia
from comun import cargar, guardar_anotada, resultado, DIR_ANALIZADAS

AQUI = Path(__file__).resolve().parent
FOTOS = Path.home() / "TFG" / "fotos_waypoints"

# --- Detector compuesto de wp03 ----------------------------------------------
# Lección de la 2ª ronda de validación: en wp03 la pose de llegada varía mucho
# entre misiones (el robot aparca distinto si hay obstáculos cerca), así que
# el diff con referencia NO sirve como detector — pero SÍ como *gate de
# validez del encuadre*: si la foto difiere mucho de la referencia, no se
# puede afirmar nada y se dice honestamente ("no_valida"). Solo si el encuadre
# es comparable se pasa al detector real: conteo de paneles azules de los
# carritos (2 paneles por carrito -> 2 carritos = 4; umbral en 3 para tolerar
# una oclusión parcial). El cielo queda fuera del rango azul por su V=243.

def detectar_wp03(img, referencia=None):
    gate = deteccion_referencia.detectar(img, referencia=referencia)
    if gate["detectado"]:                      # encuadre no comparable
        return resultado(True, "no_valida", gate["medida"], gate["umbral"])
    return deteccion_hue.detectar_conteo(img, color="azul", esperados=3)

def anotar_wp03(img, res, referencia=None):
    if res["etiqueta"] == "no_valida":
        return deteccion_referencia.anotar(img, res, referencia=referencia)
    return deteccion_hue.anotar_conteo(img, res, color="azul", esperados=3)

# --- Tabla de inspección -----------------------------------------------------
# interpreta: etiqueta_detectada -> mensaje de inspección ("AVISO ..." si
# requiere acción). La clave especial None cubre el caso "nada detectado".
TABLA = {
  "wp01": dict(det=deteccion_hue.detectar, anota=deteccion_hue.anotar,
    params={"colores": ("rojo", "verde")},
    interpreta={"rojo": "Envío en PREPARACIÓN (boyas rojas)",
                "verde": "Envío LISTO para entrega (boyas verdes)",
                None: "AVISO: sin señal de estado en zona de entrega"}),
  "wp02": dict(det=deteccion_hue.detectar_conteo, anota=deteccion_hue.anotar_conteo,
    params={"color": "rojo", "esperados": 3},
    interpreta={"rojo": "Carritos completos (2 toolbox + extintor)",
                None: "AVISO: falta material en los carritos"}),
  "wp03": dict(det=detectar_wp03, anota=anotar_wp03,
    params={"referencia": AQUI / "referencias" / "wp03_ref.png"},
    interpreta={"no_valida": "NO VALIDABLE: encuadre desplazado, no se puede "
                             "determinar el estado del pasillo 2",
                "azul": "Carritos presentes al fondo del pasillo 2",
                None: "AVISO: faltan carritos en pasillo 2"}),
  "wp04": dict(det=deteccion_forma.detectar, anota=deteccion_forma.anotar,
    params={"franjas_min": 1},
    interpreta={"cono_obra": "Pasillo con INCIDENCIA señalizada (cono de obra)",
                None: "Pasillo sin incidencia"}),
  "wp05": dict(det=deteccion_value.detectar, anota=deteccion_value.anotar, params={},
    interpreta={"objeto_oscuro": "AVISO: zona pendiente de revisión (cono negro)",
                None: "Esquina despejada"}),
  "wp06": dict(det=deteccion_hue.detectar, anota=deteccion_hue.anotar,
    params={"colores": ("lima",)},
    interpreta={"lima": "Personal PRESENTE en la zona (chaleco alta visibilidad)",
                None: "Sin personal en la zona"}),
  "wp08": dict(det=deteccion_hue.detectar, anota=deteccion_hue.anotar,
    params={"colores": ("rojo",)},
    interpreta={"rojo": "Material presente en estantería (toolbox/extintor)",
                None: "AVISO: falta material en estantería central"}),
}
# Waypoints de la ruta sin tarea de visión (aparecen igualmente en el informe):
SIN_VISION = {
  "wp07": "Zona de demo de replanificación Nav2 (sin inspección visual)",
  "wp09": "Vuelta a base — cierre de ruta (sin inspección)",
}
ORDEN_RUTA = [f"wp{i:02d}" for i in range(1, 10)]
# -----------------------------------------------------------------------------

def inspeccionar_foto(ruta, guardar_figura=True):
    """Devuelve (wp, mensaje, texto_completo) para una foto."""
    nombre = Path(ruta).name
    m = re.match(r"(wp\d\d)", nombre)
    wp = m.group(1) if m else None
    if wp in SIN_VISION:
        return wp, SIN_VISION[wp], f"{wp} | {SIN_VISION[wp]}"
    if wp not in TABLA:
        return wp, None, f"{nombre}: sin tarea de inspección asignada"
    cfg = TABLA[wp]
    img = cargar(ruta)
    r = cfg["det"](img, **cfg["params"])
    clave = r["etiqueta"] if r["detectado"] else None
    msg = cfg["interpreta"].get(clave, f"etiqueta inesperada: {clave}")
    linea_figura = ""
    if guardar_figura:
        destino = guardar_anotada(cfg["anota"](img, r, **cfg["params"]), ruta)
        linea_figura = f"\n       figura: {destino}"
    texto = (f"{wp} | {msg}\n"
             f"       [{r['etiqueta']}  medida={r['medida']}  umbral={r['umbral']}]{linea_figura}")
    return wp, msg, texto

def ultima_por_waypoint(directorio):
    ultimas = {}
    for f in sorted(Path(directorio).glob("wp*.png")):
        ultimas[f.name[:4]] = f          # orden alfabético = orden temporal
    return ultimas

def mision(directorio):
    """Procesa la última foto de cada waypoint y escribe el informe de misión."""
    ultimas = ultima_por_waypoint(directorio)
    estados = {}
    for wp, foto in ultimas.items():
        wp_id, msg, texto = inspeccionar_foto(foto)
        print(texto)
        if msg: estados[wp_id] = msg
    # Informe: los 9 puntos de la ruta en orden, con su estado o su motivo
    lineas = [f"INFORME DE INSPECCIÓN — {datetime.now().strftime('%d/%m/%Y %H:%M')}",
              f"Fotos analizadas: {Path(directorio).resolve()}", ""]
    for wp in ORDEN_RUTA:
        estado = estados.get(wp) or SIN_VISION.get(wp) or "sin foto en esta misión"
        lineas.append(f"  {wp}  {estado}")
    avisos = sum(1 for wp in ORDEN_RUTA
                 if (estados.get(wp) or "").startswith(("AVISO", "NO VALIDABLE")))
    lineas += ["", f"Puntos con aviso o no validables: {avisos}"]
    informe = "\n".join(lineas)
    destino_dir = Path(directorio) / DIR_ANALIZADAS
    destino_dir.mkdir(parents=True, exist_ok=True)
    destino = destino_dir / f"informe_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    destino.write_text(informe + "\n", encoding="utf-8")
    print("\n" + informe + f"\n\nInforme guardado en: {destino}")

if __name__ == "__main__":
    if len(sys.argv) > 1 and Path(sys.argv[1]).is_file():
        print(inspeccionar_foto(Path(sys.argv[1]))[2])
    else:
        mision(Path(sys.argv[1]) if len(sys.argv) > 1 else FOTOS)
