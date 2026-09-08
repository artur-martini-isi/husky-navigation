# agrobot_husky_nav

Stack leve de navegação por waypoints GNSS para o Husky A300, sem Nav2 e sem mapa.
Validada em campo em 2026-09-09 (reto 5 m; quadrado de 5 m a 0,5 e 1,0 m/s).

## Nós e tópicos (namespace `a300_00096`)

| Nó | Entrada | Saída |
|---|---|---|
| `livox_static_tf` | — | TF `base_link → livox_frame` (MID360 invertido e girado 90°: xyz `-0.168 0 0.45`, rpy `π 0 −π/2`) |
| `livox_to_scan` (`pointcloud_to_laserscan`) | `/livox/lidar` | `scan` em `base_link`, 720 feixes, 0,6–20 m, alturas 0,2–1,2 m |
| `gps_waypoint_follower` | pose (ver fontes), `scan`, `joy_teleop/joy`, `~/load_mission` (String: caminho do YAML), `~/set_mission` (String: YAML/JSON da missão inline, usado pela web UI) | `cmd_vel` (TwistStamped), `~/status` (String JSON a 2 Hz); serviços `~/start`, `~/pause`, `~/stop` (Trigger) |
| `ntrip_client` (launch separado) | caster NTRIP (`params_file`), `sensors/ins_0/gps_0/fix` como GGA | `sensors/ins_0/rtcm` (`rtcm_msgs/Message`) → driver → sensor |

### Fontes de pose (`pose_source`)
- **`dual_gnss`** (usado em campo): posição = ponto médio de `sensors/ins_0/gps_0/fix` e `gps_1/fix`;
  rumo = bearing GNSS1→GNSS2 + `dual_gnss_yaw_offset_deg` (90°, GNSS1 é a antena esquerda; validado com erro < 1°).
  Exige `gnss1_fix`/`gnss2_fix` ≥ `dual_gnss_min_fix` (7 = RTK float, 8 = fixed) em `fpa/gnsscorr`.
- **`fusion`** (padrão do pacote, não usável hoje): `sensors/ins_0/odom` (yaw ENU) + `fixposition/odometry_llh`;
  exige `init_status = 2` em `fpa/odomstatus`. Volta a valer quando a câmera do Fixposition for calibrada.

### Controle
Bearing até o waypoint (plano tangente local, `geo.py`) → erro de rumo → `ang = k_angular · erro` (sat.
`max_angular`); acima de `turn_in_place_angle` (1 rad) gira parado; linear = `max_linear` reduzido pelo erro
de rumo (`slowdown_angle`) e pela distância (`slowdown_distance`, 2 m), mínimo `min_linear`. Waypoint atingido
dentro de `tolerance` (0,6 m); `waypoint_timeout` 180 s aborta.

### Segurança (ordem de prioridade)
1. E-stop / safety stop da plataforma (hardware, `twist_mux`).
2. Joystick PS4: **L1 ou R1 segurado** → `twist_mux` prefere o joystick e o seguidor entra em `PAUSED`;
   **Círculo** (1) aborta; **Options** (9) inicia/retoma. Sem mensagem de `joy` por `joy_timeout` (1 s) → pausa.
3. Saúde da pose: pose mais velha que `pose_timeout`, fix fraco (`dual_gnss`) ou fusão não inicializada (`fusion`) → pausa.
4. Sem `scan` por `scan_timeout` → pausa.
5. Guarda de obstáculos no setor frontal ±`obstacle_half_angle` (0,6 rad): abaixo de `obstacle_slow_distance`
   reduz linearmente; abaixo de `obstacle_stop_distance` → `BLOCKED` (para) e retoma sozinho quando libera.
   `obstacle_steer: true` adiciona desvio para o lado mais livre (não testado em campo).

Estados: `IDLE`, `RUNNING`, `BLOCKED`, `PAUSED`, `DONE`, `ABORTED`. O campo `health` do status diz por que não anda.
O status também traz `waypoints` (lista lat/lon), `wp_index` e `max_linear`, usados pela web UI (`../webui/`).

## Launches

```bash
# navegação (sem missão; carregar depois pelo tópico)
ros2 launch agrobot_husky_nav nav.launch.py pose_source:=dual_gnss max_linear:=0.5
# a 1,0 m/s, margens maiores
ros2 launch agrobot_husky_nav nav.launch.py pose_source:=dual_gnss max_linear:=1.0 \
    obstacle_stop_distance:=1.8 obstacle_slow_distance:=4.0
# correções NTRIP (caster direto, robô com internet)
ros2 launch agrobot_husky_nav ntrip.launch.py                       # /home/robot/ntrip_ibge.yaml
# correções pelo túnel SSH do laptop (campo sem internet): ~/ntrip_tunnel_up.sh
# tudo junto
ros2 launch agrobot_husky_nav bringup.launch.py
```

Argumentos de `nav.launch.py`: `namespace`, `params`, `mission_file`, `autostart`, `pose_source`,
`max_linear`, `obstacle_stop_distance`, `obstacle_slow_distance`, `livox_topic`, `livox_xyz`, `livox_rpy`.
Demais parâmetros em `config/nav_params.yaml` (lidos na inicialização; mudar exige relançar).

## Missões

```yaml
waypoints:
  gps_points:
    - latitude: -29.79719660
      longitude: -51.15094940
      tolerance: 0.6        # opcional
```
Geradores no robô (leem a pose do `status` do seguidor, valem para as duas fontes):
`make_forward_mission.py <m>` e `make_relative_mission.py out.yaml "frente,esquerda" ...`.
Carregar: `ros2 topic pub --times 5 -r 1 -w 1 /a300_00096/gps_waypoint_follower/load_mission std_msgs/msg/String "{data: /caminho.yaml}"`
(um `--once` pode sair antes da conexão com discovery server). Exemplos testados em `../missions/`.

## Correções RTK via ROS 2
`ntrip_client` (LORD, em `~/colcon_ws/src`) publica RTCM em `sensors/ins_0/rtcm`; o `fixposition_driver`
escreve no stream TCP do sensor. **A fonte de correção do sensor precisa ser "I/O port"** (feito em 2026-09-08):
```bash
curl -s http://192.168.131.35/api/v2/gnss/rtk_get          # source: io | ntripcli | tcpcli
curl -s http://192.168.131.35/api/v2/gnss/rtk_status       # "Connected to localhost" = entrada I/O
# trocar: POST /api/v2/gnss/rtk_set com o objeto completo de rtk_get e source alterado,
#         depois POST /api/v2/ctrl/action {"rtk":"restart"}
```
Verificação: `fpa/gnsscorr` com `corr_update_rate: 1.0`, `gnss1_fix: 8`. Parando o cliente, cai a 0 e o fix vai a 5 em ~80 s.
O `ntrip_client` não recebe o `fix` (QoS incompatível com o driver); irrelevante para base física (RBMC).
Credenciais fora do pacote: `config/ntrip_params.example.yaml` é só o modelo.
