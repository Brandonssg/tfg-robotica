#!/usr/bin/env python3
"""
scripts/cargar_escenario.py

Carga un escenario de inspeccion (escenarios/escenario_X.yaml) en un mundo
Gazebo que ya esta corriendo, mediante DIFF contra el escenario cargado
anteriormente:

    - Claves que estaban antes y ya no estan en el nuevo -> se ELIMINAN.
    - Claves nuevas que no estaban antes                 -> se CREAN.
    - Claves en ambos con el mismo modelo/pose            -> se dejan intactas.
    - Claves en ambos con modelo/pose distintos            -> eliminar + crear.

Por que diff y no "crear todo lo del YAML nuevo": si se pasa de A a B sin
borrar antes lo de A, quedan elementos "fantasma" del escenario anterior
superpuestos con los del nuevo (p. ej. boyas rojas Y verdes a la vez en
wp01). El diff evita ese problema y ademas es mas rapido, porque no toca
lo que no ha cambiado entre un escenario y otro.

Requisitos:
    - Gazebo (gz sim) debe estar corriendo con el mundo ya cargado.
    - pyyaml instalado: pip install pyyaml --break-system-packages

Uso:
    python3 scripts/cargar_escenario.py escenarios/escenario_B.yaml
    python3 scripts/cargar_escenario.py escenarios/escenario_B.yaml --dry-run
"""

import argparse
import math
import subprocess
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    sys.exit(
        "Falta pyyaml. Instala con:\n"
        "    pip install pyyaml --break-system-packages"
    )

# Fichero donde se recuerda que escenario esta cargado ahora mismo.
# Vive junto a los propios escenarios para que sea evidente que es
# estado derivado, no un escenario mas a mano.
ESTADO_PATH = Path(__file__).resolve().parent.parent / "escenarios" / ".estado_actual.yaml"


# --------------------------------------------------------------------------
# Utilidades de geometria
# --------------------------------------------------------------------------

def euler_a_cuaternion(roll, pitch, yaw):
    """Convierte roll/pitch/yaw (radianes, convencion ZYX) a cuaternion
    (x, y, z, w), que es el formato que espera gz.msgs.Pose.orientation.

    Los YAML de escenario guardan roll/pitch/yaw porque es lo que Gazebo
    te da al extraer la pose de la plantilla guardada; el servicio de
    creacion, en cambio, exige cuaternion. Esta funcion hace de puente.
    """
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return x, y, z, w


# --------------------------------------------------------------------------
# E/S de ficheros
# --------------------------------------------------------------------------

def cargar_yaml(ruta):
    with open(ruta, "r", encoding="utf-8") as f:
        contenido = yaml.safe_load(f)
    return contenido or {}


def cargar_estado_anterior():
    if ESTADO_PATH.exists():
        return cargar_yaml(ESTADO_PATH)
    return {}


def guardar_estado(escenario):
    ESTADO_PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(ESTADO_PATH, "w", encoding="utf-8") as f:
        yaml.safe_dump(escenario, f, sort_keys=True, allow_unicode=True)


# --------------------------------------------------------------------------
# Interaccion con Gazebo
# --------------------------------------------------------------------------

def detectar_mundo():
    """Averigua el nombre del mundo activo listando los servicios de Gazebo
    y buscando uno del tipo /world/<nombre>/create.
    """
    resultado = subprocess.run(
        ["gz", "service", "-l"], capture_output=True, text=True, check=True
    )
    for linea in resultado.stdout.splitlines():
        linea = linea.strip()
        if linea.startswith("/world/") and linea.endswith("/create"):
            partes = linea.split("/")
            # ['', 'world', 'NOMBRE_MUNDO', 'create']
            if len(partes) >= 3:
                return partes[2]
    raise RuntimeError(
        "No se encontro ningun servicio '/world/<nombre>/create'. "
        "¿Esta Gazebo corriendo con el mundo cargado?"
    )


def listar_modelos_en_gazebo():
    """Devuelve el conjunto de nombres de modelo que existen AHORA MISMO
    en el mundo activo, segun 'gz model --list'.

    Por que hace falta: el fichero .estado_actual.yaml sobrevive a un
    reinicio de Gazebo, asi que puede afirmar que un escenario esta
    cargado cuando en realidad el mundo acaba de arrancar vacio. Antes de
    fiarnos del estado guardado, se hace "read-back" contra la realidad
    (mismo principio que releer un registro hardware en vez de confiar en
    una copia en RAM).
    """
    resultado = subprocess.run(
        ["gz", "model", "--list"], capture_output=True, text=True
    )
    modelos = set()
    for linea in resultado.stdout.splitlines():
        linea = linea.strip()
        # Formato tipico de salida:
        #     - nombre_modelo
        if linea.startswith("- "):
            modelos.add(linea[2:].strip())
    return modelos


def reconciliar_estado(estado_anterior, modelos_reales):
    """Filtra el estado guardado dejando solo las entidades que Gazebo
    confirma que existen. Las que el fichero recuerda pero ya no estan en
    el mundo (p. ej. tras reiniciar la simulacion) se descartan, de modo
    que el diff las vuelva a crear.
    """
    presentes = {}
    huerfanas = []
    for clave, entidad in estado_anterior.items():
        if clave in modelos_reales:
            presentes[clave] = entidad
        else:
            huerfanas.append(clave)
    return presentes, huerfanas


def construir_req_crear(nombre, entidad):
    modelo = entidad["modelo"]
    x, y, z, roll, pitch, yaw = entidad["pose"]
    qx, qy, qz, qw = euler_a_cuaternion(roll, pitch, yaw)

    return (
        f'sdf_filename: "{modelo}", '
        f'name: "{nombre}", '
        f'pose: {{'
        f'position: {{x: {x}, y: {y}, z: {z}}}, '
        f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}'
        f'}}'
    )


def crear_entidad(mundo, nombre, entidad):
    req = construir_req_crear(nombre, entidad)
    cmd = [
        "gz", "service", "-s", f"/world/{mundo}/create",
        "--reqtype", "gz.msgs.EntityFactory",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000",
        "--req", req,
    ]
    resultado = subprocess.run(cmd, capture_output=True, text=True)
    ok = resultado.returncode == 0 and "true" in resultado.stdout.lower()
    return ok, resultado.stdout.strip(), resultado.stderr.strip()


def eliminar_entidad(mundo, nombre):
    req = f'name: "{nombre}", type: MODEL'
    cmd = [
        "gz", "service", "-s", f"/world/{mundo}/remove",
        "--reqtype", "gz.msgs.Entity",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000",
        "--req", req,
    ]
    resultado = subprocess.run(cmd, capture_output=True, text=True)
    ok = resultado.returncode == 0 and "true" in resultado.stdout.lower()
    return ok, resultado.stdout.strip(), resultado.stderr.strip()


# --------------------------------------------------------------------------
# Diff
# --------------------------------------------------------------------------

def calcular_diff(estado_anterior, estado_nuevo):
    claves_antes = set(estado_anterior.keys())
    claves_ahora = set(estado_nuevo.keys())

    a_eliminar = claves_antes - claves_ahora
    a_crear = claves_ahora - claves_antes

    # Presentes en los dos escenarios: si modelo o pose difieren, se trata
    # como "eliminar + crear" en vez de intentar mover la entidad in-place.
    # Con el catalogo actual esto no ocurre (misma clave siempre implica
    # mismo modelo y misma pose), pero deja el script correcto si algun
    # dia se reutiliza una clave con otra pose.
    comunes = claves_antes & claves_ahora
    for clave in comunes:
        if estado_anterior[clave] != estado_nuevo[clave]:
            a_eliminar.add(clave)
            a_crear.add(clave)

    return a_eliminar, a_crear


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Carga un escenario de inspeccion en Gazebo mediante diff "
                    "contra el escenario cargado anteriormente."
    )
    parser.add_argument(
        "escenario",
        help="Ruta al YAML del escenario a cargar (p. ej. escenarios/escenario_B.yaml)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Calcula y muestra el diff sin llamar a gz service ni tocar el estado guardado",
    )
    args = parser.parse_args()

    ruta_escenario = Path(args.escenario)
    if not ruta_escenario.exists():
        sys.exit(f"Error: no existe el fichero {ruta_escenario}")

    estado_nuevo = cargar_yaml(ruta_escenario)
    estado_anterior = cargar_estado_anterior()

    # Read-back contra Gazebo: no fiarse del fichero de estado a ciegas.
    # Si Gazebo se reinicio, el mundo esta vacio aunque el fichero diga
    # otra cosa; se descartan las entradas que ya no existen realmente.
    try:
        modelos_reales = listar_modelos_en_gazebo()
        estado_anterior, huerfanas = reconciliar_estado(
            estado_anterior, modelos_reales
        )
        if huerfanas:
            print(
                f"Aviso: {len(huerfanas)} entrada(s) del estado guardado ya no "
                f"existen en Gazebo (¿simulacion reiniciada?). Se recrearan:"
            )
            print(f"  {sorted(huerfanas)}")
    except FileNotFoundError:
        # 'gz' no disponible (p. ej. --dry-run en una maquina sin Gazebo):
        # se continua con el estado guardado tal cual, avisando.
        print(
            "Aviso: no se pudo verificar contra Gazebo ('gz' no encontrado). "
            "El diff se calcula solo con el estado guardado."
        )

    a_eliminar, a_crear = calcular_diff(estado_anterior, estado_nuevo)

    if not a_eliminar and not a_crear:
        print("Sin cambios: el escenario ya esta cargado tal cual.")
        return

    print(f"Escenario anterior: {len(estado_anterior)} elemento(s)")
    print(f"Escenario nuevo:    {len(estado_nuevo)} elemento(s)")
    print(f"A eliminar ({len(a_eliminar)}): {sorted(a_eliminar) if a_eliminar else '-'}")
    print(f"A crear    ({len(a_crear)}): {sorted(a_crear) if a_crear else '-'}")

    if args.dry_run:
        print("\n[--dry-run] No se ha llamado a gz service ni se ha guardado estado.")
        return

    mundo = detectar_mundo()
    print(f"\nMundo activo: {mundo}")

    fallos = []

    # Primero eliminar, luego crear: evita colisiones si alguna vez se
    # reutiliza el mismo nombre de entidad entre escenarios.
    for clave in sorted(a_eliminar):
        ok, out, err = eliminar_entidad(mundo, clave)
        print(f"  eliminar {clave}: {'OK' if ok else 'FALLO'}")
        if not ok:
            fallos.append((clave, "eliminar", err or out))

    for clave in sorted(a_crear):
        ok, out, err = crear_entidad(mundo, clave, estado_nuevo[clave])
        print(f"  crear    {clave}: {'OK' if ok else 'FALLO'}")
        if not ok:
            fallos.append((clave, "crear", err or out))

    if fallos:
        print("\nAtencion: algunas operaciones han fallado:")
        for clave, accion, detalle in fallos:
            print(f"  - {accion} {clave}: {detalle}")
        print(
            "\nEl estado guardado NO se actualiza para evitar quedar "
            "desincronizado con lo que realmente hay en Gazebo. "
            "Corrige el problema y vuelve a ejecutar el script."
        )
        sys.exit(1)

    guardar_estado(estado_nuevo)
    print(f"\nEscenario '{ruta_escenario.name}' cargado correctamente.")
    print(f"Estado guardado en {ESTADO_PATH}")


if __name__ == "__main__":
    main()
