# Manual de instalación y uso

## Requisitos
- Ubuntu 24.04 LTS

## Instalación de ROS2 Jazzy
```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Instalación de Gazebo Harmonic
```bash
sudo apt install -y ros-jazzy-ros-gz
```

## Herramientas de desarrollo
```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep git
sudo rosdep init
rosdep update
```

## Instalación de TurtleBot3
```bash
sudo apt install -y ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-msgs ros-jazzy-turtlebot3-simulations
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
```

## Dependencias de navegación por waypoints y captura de imágenes

Necesarias para los scripts de ruta predefinida y misión de inspección
(`scripts/ruta_waypoints.py` y `scripts/ruta_waypoints_foto.py`):

```bash
sudo apt install -y ros-jazzy-nav2-simple-commander ros-jazzy-cv-bridge python3-opencv
```

- `nav2-simple-commander`: API Python de alto nivel sobre las acciones de Nav2.
- `cv-bridge`: conversión entre mensajes ROS `sensor_msgs/Image` y matrices OpenCV.
- `python3-opencv`: OpenCV (escritura de imágenes y procesado de visión).

## Uso
### Lanzar simulación base para pruebas con TurtleBot3
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Control manual por teclado (terminal aparte)
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```


## Entorno final: almacén (warehouse)

Como entorno a usar para este proyecto y más avanzado al `turtlebot3_world` de fábrica, se ha preparado un almacén industrial
(modelo Fuel de OpenRobotics/MovAi), del que se ha eliminado el robot original (Tugbot) para
sustituirlo por el TurtleBot3 Waffle. El mundo está en `worlds/warehouse_turtlebot3.sdf` y el
launch file que lo arranca y spawnea el robot, en `launch/warehouse_tb3.launch.py`.

```bash
ros2 launch launch/warehouse_tb3.launch.py
```

**Nota**: en el primer arranque con conexión a internet, Gazebo descarga
automáticamente los modelos de Fuel (estanterías, carros, almacén) a la caché
local `~/.gz/fuel/`. Los arranques posteriores no requieren conexión.

**Nota conocida**: el TurtleBot3 se renderiza en gris plano en este mundo (incompatibilidad
conocida entre las mallas `.dae` del paquete `turtlebot3_gazebo` y el motor de render Ogre2 de
Gazebo Harmonic). No afecta a sensores, TF ni navegación — verificado mediante los topics del
bridge (`/odom`, `/scan`, `/imu`, `/tf`, `/camera/camera_info`, `/cmd_vel`).



## SLAM: construcción del mapa

Con la simulación base lanzada (`warehouse_tb3.launch.py`), en otra terminal:

```bash
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true \
  slam_params_file:=$HOME/TFG/config/slam_params_warehouse.yaml
```

El archivo `config/slam_params_warehouse.yaml` contiene la configuración afinada
para este entorno. Los cambios clave respecto a los valores por defecto:


- `minimum_travel_distance/heading: 0.3` — procesa scans con más frecuencia,
  mejorando la cobertura en giros.

En caso de no conseguir un mapeado correcto puedes probar con los siguientes cambios:
- `do_loop_closing: false` — en el almacén los pasillos de estanterías son
  geométricamente muy similares entre sí, lo que provocaba falsos cierres de
  bucle (el scan-matcher "reconocía" un pasillo equivocado y deformaba el mapa).

- `distance_variance_penalty: 2.0` y `angle_variance_penalty: 2.0` — penalizan
  correcciones grandes de pose, evitando saltos bruscos del mapa.

- `correlation_search_space_dimension: 0.3` (por defecto 0.5) — restringe la
  búsqueda del scan-matcher local para que confíe más en la odometría.

Mueve el robot con teleop para cubrir todo el mapa, a velocidad moderada y
evitando giros bruscos sobre sí mismo. Cuando esté completo, guárdalo:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/TFG/maps/mapa_tfg_semi_3
```

Esto genera `mapa_tfg_semi_3.pgm` (imagen del mapa) y `mapa_tfg_semi_3.yaml`
(metadatos: resolución 0.05 m/celda y origen). El mapa validado para navegación
es `maps/mapa_tfg_semi_3`; los demás archivos de `maps/` son iteraciones
previas del proceso de ajuste, conservadas como evidencia.


## Postprocesado del mapa

El mapa crudo de SLAM Toolbox contiene valores de gris intermedios (bordes
difusos, celdas semi-observadas). El script `scripts/limpiar_mapas.py` lo
reasigna a los tres valores trinarios que espera Nav2 (0 ocupado, 205
desconocido, 254 libre), con criterio conservador: los valores intermedios
caen a desconocido, nunca a libre, para no crear zonas transitables falsas.

```bash
python3 scripts/limpiar_mapas.py maps/mapa_tfg_semi_3.pgm maps/mapa_tfg_semi_3_limpio.pgm
```
 
El `.yaml` del mapa limpio es una copia del original apuntando al nuevo `.pgm`.
**El mapa validado para navegación es `maps/mapa_tfg_semi_3_limpio`.**


## Navegación autónoma (Nav2)

Con la simulación base lanzada (`warehouse_tb3.launch.py`) y ya cargada del
todo (el almacén renderizado y el robot respondiendo), en otra terminal:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true \
  map:=$HOME/TFG/maps/mapa_tfg_semi_3_limpio.yaml \
  params_file:=$HOME/TFG/config/nav2_params_warehouse.yaml
```

`config/nav2_params_warehouse.yaml` parte del `waffle.yaml` del paquete
`turtlebot3_navigation2`, con este cambio clave:

- `set_initial_pose: true` con pose `(0, 0, yaw 0)` — AMCL publica la
  transform `map→odom` desde el arranque, sin esperar a un "2D Pose
  Estimate" manual en RViz. Sin esto, el costmap global no puede activarse
  (necesita esa transform), el lifecycle manager agota su timeout y deja
  `bt_navigator` y el resto de nodos en estado `inactive`: los goals se
  rechazan con "Action server is inactive".
  **Si se cambia el punto de spawn del robot en el launch, hay que
  actualizar esta pose (en coordenadas del frame `map`).**

### Verificación de arranque correcto

En el log deben aparecer, para AMBOS managers:

```
[lifecycle_manager_localization]: Managed nodes are active
[lifecycle_manager_navigation]: Managed nodes are active
```

Solo entonces el sistema acepta goals. Uso: botón "Nav2 Goal" en RViz →
clic en destino → arrastrar para fijar orientación. El robot planifica la
ruta global (línea sobre el mapa) y la sigue esquivando obstáculos con el
costmap local.

### Recuperación sin relanzar

Si la pila quedó en `inactive` (goals rechazados), se puede reintentar la
secuencia de activación sin cerrar nada:

```bash
ros2 service call /lifecycle_manager_navigation/manage_nodes \
nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
```

Comprobar el estado de un nodo concreto: `ros2 lifecycle get /bt_navigator`
(debe devolver `active`).

### Notas de log (benignas)

- `RTPS_TRANSPORT_SHM Error ... open_and_lock_file failed`: restos de
  memoria compartida DDS de sesiones anteriores; Fast DDS usa UDP local
  como alternativa. Sin efecto.
- `incompatible QoS (/particle_cloud)`: discrepancia de QoS entre RViz y
  AMCL en la visualización de partículas. Sin efecto en la navegación.
- `Message Filter dropping message` en RViz durante los primeros segundos:
  desaparecen en cuanto AMCL publica `map→odom`.


## Limpieza de emergencia

`scripts/reinicio_simuladores.sh` mata todos los procesos de simulación
(`pkill -9`) y reinicia el daemon de ROS2. **Es el botón de emergencia, no
el procedimiento estándar**: el cierre normal sigue siendo Ctrl+C en orden
inverso al lanzamiento (teleop → RViz/Nav2 → SLAM → Gazebo), esperando el
prompt en cada paso. Usar el script solo cuando queden procesos zombi
(síntoma típico: dos publishers en `ros2 topic info /clock --verbose`). 


## Ruta predefinida (waypoints) y misión de inspección

Con la simulación y Nav2 lanzados (ver sección anterior) y el arranque
verificado ("Managed nodes are active" en ambos lifecycle managers), la ruta
del caso de uso se ejecuta con uno de estos dos scripts:

**Ruta simple** (recorre los waypoints sin paradas):

```bash
python3 scripts/ruta_waypoints.py
```

**Misión de inspección** (en cada waypoint: parada de 7 s y captura de una
foto de la cámara):

```bash
python3 scripts/ruta_waypoints_foto.py
```

Ambos esperan automáticamente a que Nav2 esté activo, por lo que pueden
lanzarse inmediatamente después del launch. La ruta consta de 8 waypoints
con nombre descriptivo más un noveno de retorno a la base (0, 0), formando
un lazo cerrado: al terminar, el robot queda en su estado inicial y la
misión puede relanzarse sin recolocar nada.

Salida esperada: el log del script anuncia cada waypoint al alcanzarlo
(`-> Waypoint N/9: <nombre>`), y al finalizar imprime el resultado y la
duración total de la misión. Si un waypoint no puede alcanzarse, se
registra el error y la misión continúa con el siguiente.

Las fotos se guardan en `~/TFG/fotos_waypoints/` con el formato
`wpNN_<nombre>_<fecha>_<hora>.png`. Este directorio contiene datos
generados y está excluido del repositorio (`.gitignore`).

La pausa de 7 s por waypoint simula el tiempo de inspección del punto de
interés y garantiza que la captura se realiza con el robot completamente
detenido. Las fotos son la entrada del análisis de visión artificial
(detección de obstáculos, Fase 4).

**Configuración de encuadre**: en el script, el flag
`ORIENTAR_HACIA_SIGUIENTE` controla la orientación en cada waypoint. Con
`False` (valor en uso), el robot adopta la orientación capturada
manualmente en cada punto — el yaw del waypoint define el encuadre de la
foto. Con `True`, se reorienta automáticamente hacia el waypoint
siguiente (navegación más fluida, sin control del encuadre). Para
modificar la ruta basta editar la lista `WAYPOINTS` del script; las poses
se capturan con `ros2 run tf2_ros tf2_echo map base_footprint`.

## Escenarios de inspección: creación, carga y eventos dinámicos

Esta sección documenta el flujo para crear y reproducir escenarios de
inspección (Fase 4), y el mecanismo de inyección de eventos dinámicos
usado para la demo de replanificación de Nav2.

### 1. Concepto: mundo base, escenario y evento

- **Mundo base** (`worlds/warehouse_expand.sdf`): el almacén y todo el
  mobiliario/soporte *permanente*. Se mapea una única vez con SLAM Toolbox
  y da lugar al mapa de navegación (`maps/mapa_inspeccion_limpio`).
- **Escenario** (`escenarios/escenario_*.yaml`): el estado inicial de las
  señales *mutables* de cada punto de inspección (conos, toolboxes,
  personal, etc.). No forma parte del mundo base ni del mapa: se
  inyecta en tiempo de ejecución sobre el mundo ya cargado.
- **Evento** (`eventos/*.yaml`): una inyección que ocurre *a mitad de
  misión*, no al inicio. Es el mecanismo usado para el obstáculo dinámico
  que dispara la replanificación de Nav2.

La separación escenario/evento no es solo organizativa: si un elemento de
inspección se tratara como si fuera un obstáculo inesperado, el detector
de obstáculo dinámico (que compara el escaneo LiDAR contra el mapa)
generaría falsos positivos en cada punto de inspección. Los escenarios
existen para evitar justamente eso.

### 2. Cómo generar un nuevo escenario desde Gazebo

1. Lanza el mundo base y coloca los elementos deseados con
   `Resource Spawner → Fuel Resources` (busca el modelo por nombre, en  
   https://app.gazebosim.org/OpenRobotics encontrarás algunos ejemplos),
   ajustando su posición y orientación con el editor de transformar.
2. Guarda el mundo con los elementos colocados como una **plantilla
   separada** (`File → Save World As` → `worlds/world_escenario2.sdf`).
   No sobrescribas nunca `warehouse_expand.sdf`: la plantilla es
   desechable, el mundo base no.
3. Abre la plantilla guardada y localiza cada elemento añadido buscando
   `<include>` (los del mundo original ya estaban antes; los nuevos son
   los que tú acabas de colocar). Cada bloque trae:

   ```xml
   <include>
     <uri>file:///home/bran/.gz/fuel/fuel.gazebosim.org/openrobotics/models/toolbox/2</uri>
     <name>Toolbox_1</name>
     <pose>-3.907 0.01 0.0 0.0 0.0 0.0</pose>
   </include>
   ```

4. Traslada `uri` + `/model.sdf`, `name` y `pose` a un YAML de escenario
   con el formato:

   ```yaml
   wp08_toolbox1:
     modelo: "/home/bran/.gz/fuel/fuel.gazebosim.org/openrobotics/models/toolbox/2/model.sdf"
     pose: [-3.907, 0.01, 0.0, 0.0, 0.0, 0.0]
   ```

   La clave (`wp08_toolbox1`) es el nombre de instancia que se usará al
   spawnear y eliminar el elemento; no tiene por qué coincidir con el
   `<name>` autogenerado por Gazebo.

   Recomiendo el uso de IA para agilizar este último paso.

Ver `worlds/world_escenario2.sdf` para una plantilla en blanco con esta
misma guía incrustada, pensada para crear escenarios adicionales sin
partir de cero.


### 3. Servicios de Gazebo para crear/eliminar elementos en caliente

Con el mundo ya corriendo (Gazebo, Nav2 y AMCL activos), es posible
añadir y quitar modelos sin reiniciar la simulación ni perder la
localización del robot.

Primero, averigua el nombre del mundo activo:

```bash
gz service -l | grep create
```

Spawnear un elemento:

```bash
gz service -s /world/NOMBRE_MUNDO/create \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf_filename: "/home/bran/.gz/fuel/.../toolbox/2/model.sdf", name: "wp08_toolbox1", pose: {position: {x: -3.907, y: 0.01, z: 0.0}}'
```

Eliminarlo:

```bash
gz service -s /world/NOMBRE_MUNDO/remove \
  --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'name: "wp08_toolbox1", type: MODEL'
```

**Aviso sobre rutas con caracteres especiales:** algunos modelos de Fuel
tienen nombres de directorio con dos puntos (p. ej.
`drc practice: orange jersey barrier`). Encierra siempre `sdf_filename`
entre comillas dobles dentro del `--req`, o el shell puede interpretar
mal la ruta.

### 4. Escenarios actuales

| Escenario | Descripción |
|---|---|
| `escenario_A.yaml` | Estado completo: todos los puntos con señal presente (boyas rojas en wp01). |
| `escenario_B.yaml` | Degradación parcial: boyas verdes en wp01; wp02, wp05, wp06 y wp08 vaciados parcial o totalmente; wp03/wp04 sin cambios. |
| `escenario_C.yaml` | Degradación en wp02/wp03/wp04; recuperación de wp05/wp06/wp08 al estado de A; wp01 se mantiene en verde. |

Entre los tres escenarios, cada waypoint mutable pasa por estado
presente/ausente sin repetir la misma combinación completa — pensado
como evidencia en la defensa de que la detección responde al estado real
de la escena y no a un resultado memorizado.


### 5. Carga automatizada de escenarios: `scripts/cargar_escenario.py`

Automatiza las llamadas a los servicios de la sección 3: lee un YAML de
escenario y crea/elimina en bloque todos sus elementos sobre el mundo ya
en marcha, sin reiniciar la simulación ni perder la localización del
robot (Gazebo, Nav2 y AMCL siguen corriendo durante el cambio).

**Uso:**

```bash
# Con Gazebo, Nav2 y AMCL activos, y SIEMPRE ANTES de lanzar la ruta:
python3 scripts/cargar_escenario.py escenarios/escenario_A.yaml

# Ver qué haría sin tocar nada (recomendado antes de cada carga real):
python3 scripts/cargar_escenario.py escenarios/escenario_B.yaml --dry-run
```

**Orden dentro del flujo de misión:** el escenario debe cargarse *antes*
de ejecutar `ruta_waypoints_foto.py`. Si la ruta arranca primero, el
robot puede fotografiar waypoints aún vacíos y el resultado de la
inspección queda corrompido sin error visible. La secuencia completa de
una iteración es:

```bash
python3 scripts/cargar_escenario.py escenarios/escenario_A.yaml
python3 scripts/ruta_waypoints_foto.py
# ...misión completa... y para la siguiente iteración, sin reiniciar nada:
python3 scripts/cargar_escenario.py escenarios/escenario_B.yaml
python3 scripts/ruta_waypoints_foto.py
```

**Estrategia: diff contra el escenario anterior.** El script no se
limita a crear todo lo que aparece en el YAML nuevo; compara con el
escenario cargado previamente y:

| Situación de la clave | Acción |
|---|---|
| Solo en el escenario anterior | Eliminar (`remove`) |
| Solo en el escenario nuevo | Crear (`create`) |
| En ambos, mismo modelo y pose | No se toca |
| En ambos, modelo o pose distintos | Eliminar + crear |

El porqué: sin diff, al pasar de A a B quedarían elementos "fantasma"
del escenario anterior superpuestos con los del nuevo (p. ej. boyas
rojas y verdes a la vez en wp01). El orden de operaciones es primero
todas las eliminaciones y luego las creaciones, para que una clave
reutilizada con otro modelo (como `wp01_boya1` en A→B) no colisione
consigo misma.

**Estado persistido y verificación read-back.** El último escenario
cargado con éxito se guarda en `escenarios/.estado_actual.yaml`. Este
fichero es solo una caché, **no** la fuente de verdad: en cada ejecución
el script verifica sus entradas contra la realidad con `gz model --list`
y descarta las que ya no existen en el mundo (caso típico: se reinició
la simulación y el mundo arrancó vacío, pero el fichero de estado
sobrevivió). Esas entradas "huérfanas" se notifican con el aviso
`¿simulacion reiniciada?` y se recrean automáticamente, por lo que **no
hace falta borrar `.estado_actual.yaml` a mano tras reiniciar Gazebo**.

`.estado_actual.yaml` es estado de ejecución, no dato del proyecto:
está excluido del repositorio vía `.gitignore`.

**Detalles de implementación relevantes:**

- Las poses de los YAML usan roll/pitch/yaw (lo que Gazebo exporta al
  guardar la plantilla, ver sección 2), pero el servicio
  `gz.msgs.EntityFactory` exige cuaternión: el script hace la
  conversión internamente, por lo que las orientaciones con yaw ≠ 0
  (p. ej. los toolbox de wp02) se spawnean correctamente. Los comandos
  manuales de la sección 3 omiten la orientación y solo son válidos
  para elementos con yaw = 0.
- Timeout de servicio: 2000 ms por llamada (holgado para encadenar las
  ~17 operaciones de un escenario completo).
- Si alguna operación falla, el estado guardado **no** se actualiza,
  para no quedar desincronizado con lo que realmente hay en Gazebo;
  basta corregir el problema y relanzar el script (el diff recalculará
  solo lo pendiente).
- Requiere `pyyaml` (`pip install pyyaml --break-system-packages`).

> **Nota:** la barrera de wp07 (`eventos/barrera_wp07.yaml`) queda fuera
> de este mecanismo deliberadamente: es un evento runtime que se inyecta
> a mitad de misión, no estado inicial de escenario (ver sección 1).


### Validación: persistencia de Nav2 entre misiones consecutivas

Ejecución de tres misiones consecutivas (`escenario` sin cargar, ruta base)
sin tocar Gazebo ni Nav2 entre medias, tras retirar `lifecycleShutdown()`
y ajustar `default_server_timeout` (ver más arriba):

| Ejecución | Duración | Waypoints fallidos | Espera de arranque de Nav2 |
|---|---|---|---|
| 1 | 550.8 s (9.2 min) | 0 | Inmediata |
| 2 | 570.0 s (9.5 min) | 0 | Inmediata |
| 3 | 553.8 s (9.2 min) | 0 | ~5 s (descubrimiento de servicios, no reinicio) |

**Detalles de ejecución**
- Para la ejecución 1 se ha cargado el escenario A. 
- Para la ejecución 2 se ha cargado el escenario B.
- Para la ejecución 3 se ha cargado el escenario C.  

**Confirmado:** Nav2 sobrevive a las tres misiones sin intervención manual
— ya no aparece el bloqueo `amcl/get_state service not available,
waiting...` prolongado que obligaba a reiniciar Nav2 entre iteraciones.
Tampoco se ha repetido el fallo de `compute_path_to_pose` (timeout de
acuse del planner bajo carga) visto en pruebas anteriores.

**Línea base de referencia** (sin escenario cargado, sin obstáculos):
9 waypoints completados, duración típica **550-570 s (~9.2-9.5 min)**.
Cifra de comparación para las misiones con escenario + obstáculos
dinámicos de la Fase 4.

## Análisis de visión de la inspección

La misión de inspección (`scripts/ruta_waypoints_foto.py`, sección anterior)
ya analiza cada foto en el momento de capturarla y, al finalizar, genera el
informe de la misión en `fotos_waypoints/analizadas/informe_<fecha>.txt` —
no requiere ningún paso manual adicional.

Para reanalizar fotos ya capturadas (p. ej. tras ajustar un detector) o
validar un detector suelto sin simulador:

```bash
python3 scripts/vision/inspeccionar.py                    # última foto de
                                                            # cada wp en
                                                            # fotos_waypoints/
```

```bash
python3 scripts/vision/inspeccionar.py ruta/al/directorio/           # otro directorio
python3 scripts/vision/deteccion_hue.py foto.png           # un detector suelto
```

Cada detector puede probarse también por separado sin simulador:
`python3 scripts/vision/deteccion_hue.py foto.png`, etc.

**Arquitectura**: un módulo por primitiva de detección (`deteccion_hue.py`,
`deteccion_value.py`, `deteccion_forma.py`, `deteccion_referencia.py`) con
interfaz común (`comun.py`), y un despachador (`inspeccionar.py`) con una
tabla declarativa `waypoint -> detector + parámetros + interpretación` —
mismo patrón que `cargar_escenario.py`.

**Catálogo de detección por waypoint** (validado sobre 73 fotos reales):

| WP | Método | Qué mide |
|---|---|---|
| wp01 | color (hue) | boyas rojo/verde → envío en preparación/listo |
| wp02 | conteo de componentes (hue rojo) | nº de toolbox+extintor (≥3 = completo) |
| wp03 | compuesto: gate de diff + conteo (hue azul) | valida encuadre; si es válido, cuenta paneles de los carritos (≥3 = presentes) |
| wp04 | forma/contorno | nº de franjas del cono de obra (≥1 = incidencia) |
| wp05 | brillo + componente conexa | mancha oscura grande = cono negro |
| wp06 | color (hue lima) | chaleco de alta visibilidad = personal presente |
| wp07 | — | sin visión; demo de replanificación Nav2 |
| wp08 | color (hue rojo) | material presente en estantería |
| wp09 | — | cierre de ruta, sin inspección |

**Salida**: cada detector guarda además la foto con la detección superpuesta
en `fotos_waypoints/analizadas/<nombre>_analizada.png` (excluido del
repositorio, es dato generado). Al analizar una misión completa (directorio),
se genera también `fotos_waypoints/analizadas/informe_<fecha>.txt` con el
estado de los 9 waypoints.

**Referencias** (`scripts/vision/referencias/wp03_ref.png`, usada solo como
gate de encuadre en wp03): fotografía del pasillo despejado, capturada
manualmente. Si se regenera el escenario o se cambia la pose de wp03,
debe volver a capturarse.

**Requiere** `opencv-python` (`pip install opencv-python --break-system-packages`).

## Detección de obstáculos dinámicos: registro con foto y log

Nodo independiente (`scripts/detector_obstaculos.py`) que vigila `/scan`
durante toda la misión y, cuando el LiDAR devuelve puntos sobre celdas que
el mapa estático marca como libres, registra el evento con foto y pose.

**No interviene en la navegación**: Nav2 ya replanifica solo, porque su
costmap se alimenta directamente de `/scan`. Este nodo es puramente de
evidencia/registro — si fallase, la misión de navegación no se ve afectada
(misma separación navegación-crítica / percepción-analítica que el
análisis de visión de `ruta_waypoints_foto.py`).

### Requisitos (una sola vez)

```bash
pip install pillow pyyaml --break-system-packages
```

(`opencv-python`, `numpy`, `cv_bridge` ya se instalaron para el resto del
proyecto)

### Ejecución

En un tercer terminal, en paralelo a Nav2 y a la misión:

```bash
python3 scripts/detector_obstaculos.py
```

### Cómo detecta

Compara cada scan de LiDAR contra el mapa estático (`mapa_tfg_semi_3_limpio`)
transformando cada punto válido al frame `map` vía TF. Si el punto cae en
una celda marcada libre, se cuenta como anómalo. Con histéresis en dos
niveles para evitar falsos positivos por ruido:

- `MIN_PUNTOS_ANOMALOS` (150): puntos anómalos mínimos en un solo scan.
- `SCANS_CONSECUTIVOS` (3): scans seguidos con anomalía antes de confirmar.
- `COOLDOWN_SEG` (15): tras confirmar un evento, no vuelve a comprobar
  hasta pasado este tiempo, para no loguear el mismo obstáculo repetidas
  veces mientras el robot maniobra cerca.

**Decisión de diseño**: el detector no distingue entre un obstáculo
dinámico real (barreras de `eventos/*.yaml`) y el mobiliario del escenario
de inspección activo (conos, carritos, etc. de `escenarios/*.yaml`) —
ambos son, para el mapa estático generado sin ningún escenario cargado,
celdas libres ocupadas. Se ha decidido conscientemente no filtrarlo: no
importa el origen del obstáculo para el registro, y detectar también el
mobiliario del escenario es válido (posible ampliación futura: alimentar
una capa de costmap adicional, o verificar que el mobiliario del escenario
está en su sitio). Existe un flag opcional `--escenario <ruta_yaml>` que
excluiría por radio las posiciones conocidas del escenario, implementado
pero no usado.

### Salida

- `fotos_obstaculos/obstaculo_<timestamp>.png` — último frame de cámara
  disponible en el momento de la confirmación. **Limitación conocida**:
  al ser la cámara de FOV estrecho frente a un LiDAR de 360°, si la
  anomalía se detecta fuera del eje frontal de la cámara (p. ej. durante
  un giro) la foto puede no mostrar el obstáculo.
- `fotos_obstaculos/log_obstaculos.csv` — una fila por evento:
  `timestamp, x, y, puntos_anomalos, foto`.