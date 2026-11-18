# hrobot_bringup

`config/` en `launch/` voor de **echte** robot (Create 3 + RPi5 + SLAMTEC RPLIDAR C1), plus twee Python-hulpscripts
(`scan_resampler.py`, `odom_relay.py`; er is geen C++). Brengt in kaart met slam_toolbox door de robot via teleop te
bewegen en navigeert op de opgeslagen kaart met AMCL + Nav2.

| Launch / script | Inhoud |
|---|---|
| `mapping.launch.py` | Echte robot: `hardware_base` + `slam`. |
| `navigation.launch.py` | Echte robot: `hardware_base` + `nav` + odom-relay. |
| `mapping_navigation.launch.py` | Echte robot: tegelijk kaarten en navigeren: `hardware_base` + `slam` + `nav` (`localization:=slam`). |
| `hardware_base.launch.py` | `robot_state_publisher` (xacro, `sim:=false`) + driver van de RPLIDAR C1 (publiceert `scan_raw`) + `scan_resampler` (publiceert `scan`). De Create 3 wordt hier niet gestart. |
| `slam.launch.py` | `slam_toolbox` + RViz (`mapping.rviz`). Wordt hergebruikt door `hrobot_simulation`. |
| `nav.launch.py` | `map_server` + `amcl` + Nav2 + RViz (`navigation.rviz`), zonder hardware. Wordt hergebruikt door de simulatie. |
| `hrobot_namespace.py` | Hulpmodule voor namespaces (geen launch). |
| `scripts/scan_resampler.py` | `scan_raw` -> `scan` met 360 vaste stralen. |
| `scripts/odom_relay.py` | `odom` (BEST_EFFORT) -> `odom_reliable`, voor Nav2. |

## 1. De Pi voorbereiden (eenmalig)

Vanuit de root van de workspace op de Pi:

```bash
src/scripts/setup_pi.sh [--domain-id N] [--rmw rmw_fastrtps_cpp|rmw_cyclonedds_cpp] [--skip-net]
```

Installeert afhankelijkheden en de driver van de RPLIDAR C1, maakt de udev-regel `/dev/rplidar`, **stelt `ROS_DOMAIN_ID` en
`RMW_IMPLEMENTATION` in** (`~/.config/hrobot/ros_net.env`, geladen vanuit `~/.bashrc`) en bouwt. Open daarna een nieuwe
terminal en voer `source install/setup.bash` uit.

Pi, laptop en Create 3 moeten dezelfde `ROS_DOMAIN_ID` en RMW delen. De Create 3 stelt ze in via zijn webinterface
(Application > ROS 2); herstart zijn applicatie na het wijzigen. Controleer ook dat de klokken van Pi en Create 3
juist staan (`date -u`); zonder RTC/NTP start de Pi met de tijd van de laatste uitschakeling.

## 2. De laptop voorbereiden (als je RViz daarop wilt bekijken)

Heeft deze repo en `ros-jazzy-rviz2` nodig (bouwen is niet nodig). Exporteer dezelfde waarden als op de Pi:

```bash
export ROS_DOMAIN_ID=..  RMW_IMPLEMENTATION=..     # of geef ze mee met --domain / --rmw aan het script
ros2 topic list                                     # met de robot aan moet je /r2/scan zien
```

## 3. Voor het kaarten: starten vanuit het dock

Het frame `map` ontstaat op de pose van de robot bij het starten van het kaarten; als hij elke keer ergens anders begint,
komt het RViz-model niet overeen met het echte. Voor een herhaalbare referentie (en degene die nodig is om een kaart uit te
breiden, die `map_start_at_dock` gebruikt):

1. Zet de robot **in het dock** en beweeg hem niet totdat RViz de kaart toont.
2. Optioneel, als de Create 3 al lang aan staat en zijn odometrie al bewogen is, zet die op nul **voordat** je het
   kaarten start (niet terwijl slam_toolbox draait):
   ```bash
   ros2 service call /r2/reset_pose irobot_create_msgs/srv/ResetPose "{pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
   ```
   Vereist `ros-jazzy-irobot-create-msgs` op de machine die het aanroept (`sudo apt update && sudo apt install
   ros-jazzy-irobot-create-msgs`); zonder dat geeft het «The passed service type is invalid». Getest: de Create 3 antwoordt zonder fout.
3. Met de robot in het dock moet het RViz-model dezelfde kant op wijzen als de echte. Zo niet, dan blijft er een vaste
   afwijking: die is de yaw van `laser_joint` in `hrobot_rplidar_c1.urdf.xacro` (`laser_yaw`) of `laser_inverted` van de driver, geen probleem van het dock.

## 4. Kaarten

```bash
# Pi, terminal 1 (rviz:=false als je het vanaf de laptop bekijkt)
ros2 launch hrobot_bringup mapping.launch.py lidar_port:=/dev/rplidar rviz:=false
# Pi, terminal 2: de robot bewegen
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/r2/cmd_vel
# Laptop: het kaarten bekijken (of open RViz op de Pi met rviz:=true, de standaardwaarde)
src/scripts/rviz_remote.sh [mapping|navigation] [--namespace NS] [--domain ID] [--rmw NAME]
```

`rviz_remote.sh` vervangt `/<robot_namespace>` in de `.rviz` en remapt `/tf` en `/tf_static`; het stelt niets
permanent in.

Argumenten van `mapping.launch.py`: `robot_namespace:=r2  use_sim_time:=false  rviz:=true|false  maps_dir:=~/.create3_maps
map:=<naam>  slam_lifecycle:=true|false  lidar_port:=/dev/ttyUSB0  lidar_baudrate:=460800  laser_inverted:=true|false
lidar_scan_mode:=Standard`
(`laser_inverted:=true` als de scan tegengesteld aan de robot draait). Voor een andere robot:
`robot_namespace:=r3`; zonder namespace: `robot_namespace:=''`.

## 5. De kaart opslaan

Terwijl het kaarten nog draait (Pi, terminal 3):

```bash
src/scripts/save_map.sh naam_van_de_kaart [--maps-dir DIR] [--namespace NS] [--force] [--pull gebruiker@host] [--remote-ws DIR]
```

Genereert `src/maps/<naam>.{posegraph,data,yaml,pgm}` door de services `serialize_map` en `save_map` van
slam_toolbox aan te roepen; toont het antwoord van elke service, overschrijft niets zonder `--force` en controleert dat alle vier de
bestanden bestaan.

slam_toolbox schrijft de bestanden op de machine waar hij draait. Twee manieren om het te gebruiken:

- **Op de Pi** (waar slam_toolbox draait): `src/scripts/save_map.sh sala1`. Kopieer de bestanden daarna met
  `scp` naar je laptop.
- **Vanaf de laptop**: `src/scripts/save_map.sh sala1 --pull robot2@<IP_PI>` slaat op in `<remote-ws>/src/maps` van de Pi
  (`--remote-ws`, standaard `~/Hrobot`) en kopieert de vier bestanden naar `src/maps` van de laptop. Vereist `ssh` naar de Pi.
  Zonder `--pull` waarschuwt het script dat er geen lokale slam_toolbox is en, als de bestanden ontbreken, legt het uit dat ze op
  de andere machine zijn opgeslagen.

<details>
<summary>Handmatig equivalent</summary>

```bash
WS=$PWD   # de kaarten komen in <workspace>/src/maps
ros2 service call /r2/slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$WS/src/maps/sala1'}"
ros2 service call /r2/slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$WS/src/maps/sala1'}}"
```

De namen van de velden zijn uit het hoofd geschreven: controleer ze met `ros2 interface show`.
</details>

## 6. Een opgeslagen kaart uitbreiden (optioneel)

Zet voor het kaarten de robot in het dock/de oorsprong van de kaart en start het kaarten opnieuw met de kaart geladen:

```bash
ros2 launch hrobot_bringup mapping.launch.py map:=sala1 maps_dir:=$PWD/src/maps
```

slam_toolbox laadt `<maps_dir>/<naam>.posegraph` (`map_start_at_dock: true`, mapping-modus). Bestaat de pose graph niet,
dan stopt de launch met een `RuntimeError` (er start niet stilzwijgend een nieuwe kaart). Paden moeten absoluut zijn (zonder `~`).
Ga daarna terug naar stap 5 om op te slaan.

## 7. Navigeren op de opgeslagen kaart

Stop eerst het kaarten (Ctrl+C): mapping en navigation tegelijk publiceren `map -> odom`.

```bash
ros2 launch hrobot_bringup navigation.launch.py map_yaml:=$PWD/src/maps/sala1.yaml lidar_port:=/dev/rplidar
```

- **Beginpose (zoals in de simulatie):** standaard start AMCL al op `initial_x/y/yaw` = `0 0 0`, de oorsprong van de
  kaart, dus het dock waarvandaan gekaart is. Zet de robot voor het starten in het dock en stuur doelen direct met
  "2D Goal Pose" (topic `goal_pose`). Staat de robot ergens anders, geef dan `initial_x:=.. initial_y:=.. initial_yaw:=..` mee
  (in het kaartframe) of `set_initial_pose:=false` en stel hem in met "2D Pose Estimate" in RViz. Starten met de pose al
  ingesteld voorkomt ook dat Nav2 half geactiveerd blijft wachten op `map -> base_link`.
- Nav2 publiceert snelheden op `cmd_vel` (`/r2/cmd_vel`).
- `nav.launch.py`: `map_server` + `amcl` (publiceert `map -> odom`) + Nav2 (controller RPP, planner Navfn, smoother,
  behaviors, bt_navigator) + RViz. Configuratie in `config/nav2_params.yaml`. `navigation.launch.py` voegt `hardware_base` toe en
  activeert de odom-relay.
- `odom_relay.py`: de Create 3 publiceert `/r2/odom` als BEST_EFFORT en Nav2 abonneert zich als RELIABLE (koppelt niet); de relay publiceert
  `odom_reliable`, dat `controller_server` en `bt_navigator` lezen. Controleer met
  `ros2 topic info /r2/odom_reliable -v`.

## 8. Tegelijk kaarten en navigeren (optioneel)

```bash
ros2 launch hrobot_bringup mapping_navigation.launch.py lidar_port:=/dev/rplidar
# andere terminal: teleop, of "2D Goal Pose" in RViz
src/scripts/save_map.sh naam_van_de_kaart        # de kaart wordt net zo opgeslagen als in stap 5
```

slam_toolbox (mapping-modus) publiceert `map -> odom` en de levende `/map`; Nav2 plant erop (`nav.launch.py` met
`localization:=slam`: zonder `map_server` of AMCL en met `allow_unknown: true` in de planner). Er is geen beginpose om
in te stellen: de kaart ontstaat op de startpose. Gebruik één enkele RViz (`navigation.rviz`); `map:=<naam>` breidt een opgeslagen
kaart uit zoals in stap 6. **Combineer het niet met `mapping` of `navigation`** (alle drie publiceren `map -> odom`).

Houd er rekening mee: de kaart verandert onder de robot als slam_toolbox de pose corrigeert; plannen mislukken eerder door smalle
doorgangen of onverkende zones; en slam_toolbox + Nav2 samen belasten de Pi 5 (houd `top` in de gaten; als hij verzadigt, verhoog
`throttle_scan` of `map_update_interval` in `config/slam_params.yaml`). In simulatie:
`ros2 launch hrobot_simulation sim.launch.py nav:=true localization:=slam`.

## Referentie

### Namespace van de robot

`robot_namespace` is standaard `$CREATE3_ROBOT_NAMESPACE` als die bestaat en anders `r2`; leeg = geen namespace.
Hij wordt **één keer** toegepast: elke launch doet `PushRosNamespace` alleen voor de nodes die hij aanmaakt en `mapping` wikkelt de
`include`s niet (nooit `/r2/r2/...`).

- **TF**: remap `/tf -> tf`, `/tf_static -> tf_static` (de Create 3 publiceert `/r2/tf`). De `frame_id`'s hebben geen prefix.
- **YAML**: met namespace wordt hij herschreven met `RewrittenYaml(root_key=<ns>)`; zonder namespace wordt hij ongewijzigd gebruikt.
- **slam_toolbox** publiceert een absolute `/map`: die wordt geremapt naar `/<ns>/map`.
- **RViz**: `mapping.rviz` gebruikt `/<robot_namespace>/...`, vervangen met `ReplaceString`.

| Naam | `r2` | leeg |
|---|---|---|
| LiDAR / kaart | `/r2/scan`, `/r2/map` | `/scan`, `/map` |
| Create 3 | `/r2/odom`, `/r2/cmd_vel`, `/r2/tf`, `/r2/tf_static` | `/odom`, `/cmd_vel`, `/tf`, `/tf_static` |
| slam_toolbox | `/r2/slam_toolbox/serialize_map`, `/r2/slam_toolbox/save_map` | zonder prefix |

### Parameters

`config/slam_params.yaml`: slam_toolbox online_async, `max_laser_range: 12.0` (limiet van de RPLIDAR C1).

### Aannames en hardware-openstaande punten (niets geverifieerd tegen echt ROS 2)

- **Driver RPLIDAR C1**: `sllidar_ros2` van Slamtec (executable `sllidar_node`; parameters `channel_type: serial`,
  `serial_port`, `serial_baudrate: 460800`, `frame_id`, `inverted`, `angle_compensate: true`, `scan_mode`). De driver publiceert
  op `scan`; `hardware_base.launch.py` remapt dat naar `scan_raw`. Gebruik je een andere driver, pas dan
  `hardware_base.launch.py` en `package.xml` aan. Standaardpoort `/dev/ttyUSB0`, 460800 baud; `laser_inverted` kan
  omgedraaid moeten worden. Controleer met `ros2 param list /r2/sllidar_node` of de parameternamen in jouw versie van de driver kloppen.
- **Oriëntatie**: `laser_yaw` staat standaard op 0 in de xacro (de 0°-straal wijst naar voren). Dit is niet gemeten voor de
  RPLIDAR C1: controleer het in RViz met de robot in het dock en pas `laser_yaw` aan.
- **`scan_resampler.py`**: slam_toolbox meldt `LaserRangeScan contains N range readings, expected M` als het aantal stralen per
  omwenteling varieert. De driver publiceert `scan_raw` en de node publiceert `scan` opnieuw met 360 vaste stralen (0..2π,
  dichtstbijzijnde buur; parameter `num_beams`). Met `angle_compensate: true` levert de driver mogelijk al een vast aantal
  stralen; dan is de resampler overbodig maar onschadelijk. Controleer dat de waarschuwing verdwijnt en dat `ros2 topic hz /r2/scan` op ~10 Hz blijft.
- QoS van `/r2/scan_raw` van de driver (best effort vs reliable) moet compatibel zijn met de abonnee van de resampler
  (`ros2 topic info /r2/scan_raw -v`).
- **Create 3**: verbindt via het netwerk (zie stap 1) en publiceert `odom -> base_link` op `/r2/tf`.
- **TF**: `laser_frame` hangt aan `base_link` met `laser_z: 0.12` m (meet de echte waarde). `map -> odom` komt van slam_toolbox.
- **slam_toolbox lifecycle**: in Jazzy is `async_slam_toolbox_node` een lifecycle-node; zo niet, `slam_lifecycle:=false`.
