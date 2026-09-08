# Husky A300 (cpr-a300-00096) — navegação por waypoints GNSS RTK

Configuração e operação do Husky A300 físico do Agrobot. Objetivo atingido em 2026-09-09:
navegação por waypoints globais (lat/lon) com o Fixposition Vision-RTK 2, anticolisão com o
Livox MID360 e bypass de emergência pelo joystick, **validada em campo** (5 m reto a 0,3 m/s;
quadrado de 5 m a 0,5 e a 1,0 m/s).

## Conteúdo deste diretório

| Caminho | O que é |
|---|---|
| `README.md` | Este documento: estado atual, acesso, arquitetura, operação em campo, problemas conhecidos |
| `PLAN.md` | Histórico do que foi feito (por data) e próximos passos |
| `agrobot_husky_nav/` | Pacote ROS 2 da navegação (fonte de verdade; implantado em `~/colcon_ws/src` no robô) |
| `agrobot_husky_nav/README.md` | Uso do pacote: launches, parâmetros, tópicos, joystick, NTRIP |
| `tools/` | Scripts. Os que rodam **no laptop**: `deploy.py`, `rssh.py`, `ntrip_tunnel.py`. Os que rodam **no robô** (copiados para `/home/robot/` pelo deploy): `field_up.sh`, `nav_kill.sh`, `fixposition_wired.sh`, `ntrip_tunnel_up.sh`, `make_forward_mission.py`, `make_relative_mission.py`, `heading_check.py` |
| `missions/` | Missões usadas nos testes de campo (formato do seguidor) |
| `inventory/` | Cópias de configuração do robô: `robot.yaml`, `ins_0.yaml`, `localization.yaml`, `twist_mux.yaml`, `netplan-50-clearpath-bridge.yaml` (senhas removidas), listas de tópicos |
| `robot-home/` | Cópia dos scripts do Livox que já existiam no robô (`start_livox_pc2.sh`, `restart_livox.sh`, `decimate.py`, ...) |

Sincronização: **o diretório local é a fonte de verdade**. `python3 tools/deploy.py` copia o pacote
e os scripts para o robô e compila. Auditado em 2026-09-10: pacote e scripts idênticos nos dois lados.

## Acesso

| Item | Valor |
|---|---|
| Hostname / serial | `cpr-a300-00096` / a300-00096 |
| Usuário / senha | `robot` / `clearpath` (sudo liberado) |
| Wi-Fi do robô | SSID **Agriwing** (2,4 GHz, para alcance) → `10.0.0.60`. `ssh robot@10.0.0.60`. A `Agriwing_5G` foi removida do netplan do robô em 2026-09-09 (backup `.bak-2026-09-09`) para ele não voltar à 5 GHz |
| Rede cabeada do robô | `192.168.131.1/24` (bridge `br0`, sem DHCP). MCU em `.2`, Livox MID360 em `.109`, segundo Livox em `.112`, **Fixposition em `.35`** |
| Fixposition | `http://192.168.131.35` (web UI e API JSON em `/api/v2/`). Acesso do laptop via túnel: `ssh -L 8080:192.168.131.35:80 robot@10.0.0.60`. O Wi-Fi do sensor (antes `10.0.0.199`) não é mais usado |
| Foxglove | `ws://10.0.0.60:8765`. URDF em `/a300_00096/robot_description` (adicionar como URDF por tópico no painel 3D; frame `base_link`). Nuvem reduzida para Wi-Fi fraco: `/livox/lidar_lite` (`~/run_dec.sh`) |
| ROS 2 no laptop | Jazzy em `/opt/ros/jazzy`. Para ver o grafo do robô: `ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DISCOVERY_SERVER=10.0.0.60:11811 ROS_SUPER_CLIENT=True`, depois `ros2 daemon stop; ros2 topic list --spin-time 8` |

Sem `sshpass` no laptop: `tools/rssh.py '<comando>'` roda comandos no robô (pexpect).
Recomendado instalar chave SSH (`ssh-copy-id robot@10.0.0.60`).

## Robô

- Ubuntu 24.04, ROS 2 **Jazzy**, pacotes Clearpath 2.9.x, namespace ROS **`a300_00096`**.
- Discovery: FastDDS Discovery Server em `127.0.0.1:11811`. Em shell não interativo exportar
  `ROS_SUPER_CLIENT=True`; `ros2 topic list` precisa de `--spin-time 8`.
- `/etc/clearpath/robot.yaml` é symlink para `~/colcon_ws/src/senai02_a300/senai02_bringup/config/robot.yaml`
  (pacote customizado da SENAI). Após editar: `sudo systemctl restart clearpath-robot` regenera tudo.
- Workspace `~/colcon_ws/src`: `fixposition_driver` (v6.1.x), `senai02_a300`, `ntrip_client` (LORD), `agrobot_husky_nav`.
- Serviços Clearpath: `clearpath-robot`, `-platform`, `-sensors`, `-discovery`, `-vcan`. `platform-extras` está vazio.
- Odometria de rodas + EKF (`platform/odom/filtered`, frame `odom`) a 50 Hz. `cmd_vel` externo entra no
  `twist_mux` com prioridade 1; joystick 10; RC 12; e-stop e safety stop travam tudo.

### Sensores

| Sensor | Onde | Estado |
|---|---|---|
| IMU Phidgets Spatial | `sensors/imu_0/data`, 50 Hz | OK |
| Fixposition Vision-RTK 2 (2 antenas) | cabo `192.168.131.35:21000`, driver via `clearpath-sensors`; tópicos `sensors/ins_0/gps_0/fix`, `gps_1/fix` (~5 Hz), `odom` (ENU), `fixposition/odometry_llh`, `fpa/gnsscorr`, `fpa/odomstatus`, `rtcm` (entrada) | OK. **Fusão desligada** (câmera sem calibração): sem `odom`/`odometry_llh` úteis; usa-se as duas antenas |
| Livox MID360 | `192.168.131.109` → `/livox/lidar` (10 Hz, ~20k pts). Montado **invertido e girado 90°** sob o Fixposition, ~0,61 m do chão; TF no launch: xyz `-0.168 0 0.45`, rpy `π 0 −π/2` (validado com pessoa na frente) | OK, sobe com `~/start_livox_pc2.sh` (não é serviço) |
| Joystick PS4 | `/dev/input/ps4`, `joy_teleop/joy` 20 Hz; L1 habilita, R1 turbo | OK |

## Arquitetura da navegação (arquitetura "B")

```
IBGE RBMC (NTRIP, RSSL0) ──internet──> laptop ──túnel SSH reverso──> robô:127.0.0.1:2101
                                                                        │ ntrip_client (ROS)
                                                                        ▼ sensors/ins_0/rtcm
Fixposition (cabo, fonte de correção = "I/O port") <── fixposition_driver ──┘
   │ gps_0/fix + gps_1/fix (RTK fixed)            Livox MID360 ──> pointcloud_to_laserscan ──> scan
   ▼                                                                                       │
gps_waypoint_follower (pose_source=dual_gnss): posição = ponto médio das antenas,          │
   rumo = linha GNSS1→GNSS2 + 90°; controlador P de rumo; guarda de obstáculos <───────────┘
   ▼ cmd_vel (TwistStamped, prioridade 1)        joystick PS4 (prioridade 10) = bypass
twist_mux ──> plataforma
```

- **Sem Nav2 e sem mapa** por decisão de projeto (stack leve).
- **Correções RTK via ROS 2**: a fonte de correção do sensor é a entrada I/O (RTCM chega pelo stream TCP do
  driver). Provado: parando o cliente NTRIP a taxa de correção vai a 0 e o fix cai de RTK para 3D em ~80 s.
- **Internet em campo**: o roteador Agriwing não tem WAN. O laptop (internet pelo iPhone via USB) mantém
  `tools/ntrip_tunnel.py`; no robô `~/ntrip_tunnel_up.sh` aponta o cliente para `127.0.0.1:2101`.
  Se o laptop cair, o RTK degrada em ~80 s e o seguidor pausa sozinho (fix < RTK float).
- **Rumo por antena dupla** validado em campo: erro < 1° contra o deslocamento RTK em 3 m. Precisa de RTK
  (`gnss1_fix`/`gnss2_fix` ≥ 7; 8 = fixed). Baseline 0,46 m.
- **Segurança em camadas**: e-stop físico → joystick (L1 sobrepõe e pausa; Círculo aborta; Options inicia/retoma;
  sem joystick por 1 s pausa) → saúde do sensor (pose velha, fix fraco, sem scan) → guarda de obstáculos
  (reduz de `obstacle_slow_distance`, para em `obstacle_stop_distance`, retoma sozinho).

## Operação em campo (checklist)

1. Ligar o robô e o Fixposition (alimentado pelo painel). Esperar ~2 min. Liberar o e-stop.
2. No laptop (com internet): `python3 tools/ntrip_tunnel.py &` (mantém o túnel; reconecta sozinho).
3. No robô: `~/field_up.sh` (reinicia o driver se o sensor acabou de subir; sobe Livox e NTRIP) e
   `~/ntrip_tunnel_up.sh` (NTRIP pelo túnel). Conferir `fpa/gnsscorr`: `gnss1_fix: 8 gnss2_fix: 8`, `corr_update_rate: 1.0`.
4. Subir a navegação sem missão:
   `ros2 launch agrobot_husky_nav nav.launch.py pose_source:=dual_gnss max_linear:=0.5`
   (a 1,0 m/s usar `obstacle_stop_distance:=1.8 obstacle_slow_distance:=4.0`).
5. Validar o rumo uma vez por montagem: `~/heading_check.py start`, andar 3 m reto com o joystick,
   `~/heading_check.py end` (erro deve ser de poucos graus).
6. Missão: `~/make_relative_mission.py /home/robot/m.yaml "5,0" "5,5" "0,5" "0,0"` (frente,esquerda em m)
   ou `~/make_forward_mission.py 5`; carregar com
   `ros2 topic pub --times 5 -r 1 -w 1 /a300_00096/gps_waypoint_follower/load_mission std_msgs/msg/String "{data: /home/robot/m.yaml}"`.
7. Conferir `gps_waypoint_follower/status` (`health: null`, distância e erro de rumo coerentes) e iniciar com
   **Options** no joystick ou `ros2 service call /a300_00096/gps_waypoint_follower/start std_srvs/srv/Trigger`.
8. Parar tudo: `~/nav_kill.sh all` (nunca usar `pkill -f` com o texto do launch no mesmo comando: mata o próprio shell).

Resultados de campo (2026-09-09): 5 m reto a 0,3 m/s em 18 s (0,58 m do ponto); quadrado 5 m a 0,5 m/s em
68 s; a 1,0 m/s em 48 s; chegada sempre a ~0,57 m (tolerância 0,6 m); guarda de obstáculos bloqueou e
retomou sozinha nas duas corridas do quadrado.

## Problemas conhecidos e decisões

- **Fusão do Fixposition parada**: a câmera não está calibrada e a fusão dá erro; decisão de não usar por ora.
  `fusion autostart` ficou habilitado na API mas o start falha (`request fail`). Consequência: só o modo
  `dual_gnss`, que exige RTK. Calibrar a câmera devolve rumo e pose mesmo sem RTK fixed.
- **Rede do sensor**: em 2026-09-09 ele apareceu sem IP no cabo; o IP estático `192.168.131.35/24` foi fixado
  pela interface do sensor. Se sumir de novo, `ip neigh`/`tcpdump -i br0` no robô mostram se ele fala algo.
- **Livox e NTRIP não são serviços**: precisam do `field_up.sh` após cada boot (pendência: systemd).
- **`restart_livox.sh` pode deixar instâncias duplicadas** do driver (as portas UDP entram em conflito e a nuvem para).
  `nav_kill.sh livox` limpa.
- **FOV vertical do MID360 invertido**: só 7° acima do lidar (~0,85 m do chão); obstáculos altos e finos só de perto.
  `range_min: 0.6` remove a própria estrutura do robô.
- **Deploy**: `scp -r` em diretório existente aninha a cópia; `deploy.py` apaga a fonte remota antes.
- **Fixposition pelo Wi-Fi** gerava atrasos de 0,1–0,4 s no driver; no cabo, zero avisos.
- **Foxglove**: allowlist de assets do Clearpath só aceita extensões minúsculas; `sensor_arch.STL` foi duplicada como `.stl`.
- **API do sensor**: `rtk_set` exige o objeto completo de `rtk_get` e só vale após `ctrl/action {"rtk":"restart"}`;
  com fonte I/O o `rtk_status` mostra "Connected to localhost". Voltar ao cliente interno: `source: ntripcli` + credenciais.
- Credenciais NTRIP (IBGE) ficam **só no robô** (`~/ntrip_ibge.yaml`, `~/ntrip_tunnel.yaml`); no repositório há apenas o modelo.

## Interface web de teste (`webui/`)

Página estática servida pelo próprio robô: **http://10.0.0.60:8088** (o `field_up.sh` sobe um
`python3 -m http.server 8088` em `/home/robot/webui`). Também funciona abrindo `webui/index.html`
direto do laptop. Ela fala com o **Foxglove bridge** do robô (`ws://10.0.0.60:8765`, subprotocolo
`foxglove.sdk.v1`), sem nenhum servidor extra:

- assina `gps_waypoint_follower/status` (posição, rumo, estado, saúde, obstáculo, fix RTK, joystick, waypoints);
- desenha um mapa local em metros (norte para cima, origem na primeira pose ou "origem aqui"), com rastro,
  seta do robô, arco do obstáculo mais próximo e os waypoints da missão em curso;
- cria waypoints clicando no mapa, por atalhos relativos ao rumo atual (5 m frente, quadrado 5 m, fileiras)
  ou editando o YAML; **Enviar missão** publica em `gps_waypoint_follower/set_mission` (JSON via `clientPublish`);
- **Start / Pause / Stop** chamam os serviços do seguidor (requisição `Trigger` em CDR).

Testada em 2026-09-10 com um cliente Node contra o robô (assinatura, envio de missão, chamada de serviço).
Requer o `nav.launch.py` no ar; sem ele a página mostra "seguidor fora do ar". Não substitui o joystick:
L1/Círculo continuam sendo o bypass de emergência.
