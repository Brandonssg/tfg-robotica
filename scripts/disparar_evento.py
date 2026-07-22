#!/usr/bin/env python3
"""
scripts/disparar_evento.py
Dispara un evento runtime (obstáculo dinámico) durante la misión, spawneando
en Gazebo las entidades definidas en un YAML de eventos/ (p. ej.
eventos/barrera_wp02_wp03.yaml). Pensado para ejecutarse EN UN SEGUNDO
TERMINAL mientras la misión está en curso, justo antes de que el robot
entre en el tramo que se quiere cortar.

No usa el mecanismo de escenarios (sin diff, sin estado persistido): un
evento solo se crea, nunca se compara contra un estado anterior. Si se
relanza sin --retirar antes, gz service create devuelve false para los
nombres que ya existen (nombre de entidad duplicado) y el script lo
reporta como "ya existe, omitido" en vez de fallar.

Uso:
    python3 scripts/disparar_evento.py eventos/barrera_wp02_wp03.yaml
    python3 scripts/disparar_evento.py eventos/barrera_wp02_wp03.yaml --retirar
"""
import argparse
import math
import subprocess
import sys
from pathlib import Path

import yaml


def euler_a_cuaternion(roll, pitch, yaw):
    """El YAML guarda roll/pitch/yaw (lo que exporta Gazebo al inspeccionar
    una pose), pero el servicio gz.msgs.EntityFactory exige cuaternión."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * cp * sy
    return x, y, z, w


def detectar_mundo():
    """Averigua el nombre del mundo activo listando los servicios de Gazebo.
    Evita hardcodear 'warehouse_expand' o similar: si el nombre del mundo
    cambia, el script sigue funcionando sin tocar código."""
    resultado = subprocess.run(
        ["gz", "service", "-l"], capture_output=True, text=True, check=True
    )
    for linea in resultado.stdout.splitlines():
        linea = linea.strip()
        if linea.startswith("/world/") and linea.endswith("/create"):
            partes = linea.split("/")
            if len(partes) >= 3:
                return partes[2]
    raise RuntimeError(
        "No se encontró ningún servicio '/world/<nombre>/create'. "
        "¿Está Gazebo corriendo con el mundo cargado?"
    )


def listar_modelos_en_gazebo():
    """Nombres de modelo que existen AHORA MISMO en el mundo, vía
    'gz model --list'. Se usa para no reintentar spawnear algo que ya
    está ahí (idempotencia: relanzar el script no debe romper nada)."""
    resultado = subprocess.run(
        ["gz", "model", "--list"], capture_output=True, text=True, check=True
    )
    nombres = set()
    for linea in resultado.stdout.splitlines():
        linea = linea.strip()
        if not linea or linea.lower().startswith("available models"):
            continue
        nombres.add(linea)
    return nombres


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


def main():
    parser = argparse.ArgumentParser(
        description="Dispara (o retira) un evento runtime de eventos/*.yaml"
    )
    parser.add_argument("evento", help="Ruta al YAML del evento, p. ej. eventos/barrera_wp07.yaml")
    parser.add_argument(
        "--retirar", action="store_true",
        help="En vez de crear, elimina las entidades del evento (para repetir la demo)"
    )
    args = parser.parse_args()

    ruta = Path(args.evento)
    if not ruta.exists():
        sys.exit(f"Error: no existe el fichero {ruta}")

    with open(ruta, "r", encoding="utf-8") as f:
        entidades = yaml.safe_load(f) or {}

    if not entidades:
        sys.exit(f"Aviso: {ruta} no define ninguna entidad.")

    mundo = detectar_mundo()
    print(f"Mundo activo: {mundo}")

    if args.retirar:
        for nombre in sorted(entidades):
            ok, out, err = eliminar_entidad(mundo, nombre)
            print(f"  retirar {nombre}: {'OK' if ok else 'FALLO'} {err or ''}")
        return

    existentes = listar_modelos_en_gazebo()
    fallos = []
    for nombre, entidad in sorted(entidades.items()):
        if nombre in existentes:
            print(f"  crear {nombre}: ya existe, omitido")
            continue
        ok, out, err = crear_entidad(mundo, nombre, entidad)
        print(f"  crear {nombre}: {'OK' if ok else 'FALLO'} {err or ''}")
        if not ok:
            fallos.append(nombre)

    if fallos:
        print(f"\nAtención: fallaron {len(fallos)} entidad(es): {fallos}")
        sys.exit(1)

    print(f"\nEvento '{ruta.name}' disparado correctamente.")


if __name__ == "__main__":
    main()