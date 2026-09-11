# Husky A300 (cpr-a300-00096) — navegação outdoor por GNSS RTK e indoor por mapa

Configuração e operação do Husky A300 físico do Agrobot. O robô navega hoje em **dois modos
independentes**, cada um validado no seu ambiente, e um **modo híbrido** com transição entre os dois
está planejado (seção no fim).

| | **Outdoor (campo)** | **Indoor (laboratório)** |
|---|---|---|
| Onde o robô se acha | Fixposition Vision-RTK 2, duas antenas GNSS, RTK fixed | `slam_toolbox` casando o `scan` do MID360 contra o mapa |
| Frame de trabalho | lat/lon convertido para metros no plano local | `map`, criado pelo SLAM |
| Rumo | linha entre as duas antenas (erro < 1° em 3 m) | pose do SLAM |
| Quem escolhe o destino | missão de waypoints lat/lon, gravada ou calculada | clique no mapa (Foxglove) ou fronteiras, na exploração |
| Como chega lá | controlador P de rumo, sem mapa e sem planejador | A* sobre o mapa + perseguição por curvatura |
| Anticolisão | guarda frontal pelo `scan` do MID360 | mesma guarda, mais o A*, que já desvia no papel |
| Nó | `gps_waypoint_follower` | `goto_point` (marcador) e `explore` (autônomo) |
| Validado em | 2026-09-09, campo | 2026-09-11, laboratório |

O que os dois têm em comum: o mesmo robô, o mesmo MID360 como sensor de colisão, o mesmo joystick
como segurança, o mesmo `twist_mux` e a mesma ponte de telemetria para o sistema Agrobot. **Sem Nav2
em nenhum dos dois**, por decisão de projeto (stack leve).

> **Regra que vale nos dois modos**: apenas um controlador pode publicar em `cmd_vel`.
> `ros2 topic info /a300_00096/cmd_vel` tem que mostrar **um** publicador; mais que isso são dois
> controladores disputando o robô. `~/nav_kill.sh nav|goto|explore|follow|all` encerra o que sobrar.

## Conteúdo deste diretório

| Caminho | O que é |
|---|---|
| `README.md` | Este documento: modos de navegação, acesso, operação, problemas conhecidos |
| `PLAN.md` | Histórico do que foi feito (por data) e próximos passos |
| `agrobot_husky_nav/` | Pacote ROS 2 da navegação (fonte de verdade; implantado em `~/colcon_ws/src` no robô) |
| `agrobot_husky_nav/README.md` | Uso do pacote: launches, parâmetros, tópicos, joystick, NTRIP, SLAM, planejador |
| `tools/` | Scripts. Os que rodam **no laptop**: `deploy.py`, `rssh.py`, `ntrip_tunnel.py`, `make_aruco.py`. Os que rodam **no robô** (copiados para `/home/robot/` pelo deploy): `field_up.sh`, `nav_kill.sh`, `fixposition_wired.sh`, `ntrip_tunnel_up.sh`, `make_forward_mission.py`, `make_relative_mission.py`, `heading_check.py`, `record_waypoints.py`, `save_map.sh`, `goto.py` |
| `calib/` | Calibração de fábrica da ZED 2i e o alvo ArUco impresso do follow-me |
| `missions/` | Missões usadas nos testes de campo (formato do seguidor), incluindo as gravadas com o joystick |
| `proximos_passos_plaac.md` | Plano de integração com o PLAAC e as camadas de comunicação/serviços |
| `webui/` | Interface web de teste (posição ao vivo, waypoints por clique, start/pause/stop) |
| `inventory/` | Cópias de configuração do robô: `robot.yaml`, `ins_0.yaml`, `localization.yaml`, `twist_mux.yaml`, `netplan-50-clearpath-bridge.yaml` (senhas removidas), listas de tópicos |
| `robot-home/` | Cópia dos scripts do Livox que já existiam no robô (`start_livox_pc2.sh`, `restart_livox.sh`, `decimate.py`, ...) |

Sincronização: **o diretório local é a fonte de verdade**. `python3 tools/deploy.py` copia o pacote
e os scripts para o robô e compila. Auditado em 2026-09-10: pacote e scripts idênticos nos dois lados.

## Acesso

| Item | Valor |
|---|---|
| Hostname / serial | `cpr-a300-00096` / a300-00096 |
| Usuário / senha | `robot` / `clearpath` (sudo liberado) |
| Wi-Fi do robô | **Agriwing_5G** em laboratório, **Agriwing** (2,4 GHz) em campo → `10.0.0.60`. Os dois perfis estão no netplan e o wpa_supplicant escolhe pelo sinal; para forçar 2,4 GHz em campo, remover o bloco `Agriwing_5G`. `ssh robot@10.0.0.60`. A `Agriwing_5G` foi removida do netplan do robô em 2026-09-09 (backup `.bak-2026-09-09`) para ele não voltar à 5 GHz |
| Rede cabeada do robô | `192.168.131.1/24` (bridge `br0`, sem DHCP). MCU em `.2`, Livox MID360 em `.109`, segundo Livox em `.112`, **Fixposition em `.35`** |
| Fixposition | `http://192.168.131.35` (web UI e API JSON em `/api/v2/`). A conexão cabeada dele (`fp-navvr2-eth0-static-ip`) tinha **autoconnect desligado** e sumia a cada boot; em 2026-09-09 foi ligado por `POST /api/v2/net/conn_set {"connection":"fp-navvr2-eth0-static-ip","auto":true}` e passou a subir sozinho (verificado após reboot). Acesso do laptop via túnel: `ssh -L 8080:192.168.131.35:80 robot@10.0.0.60`. O Wi-Fi do sensor (antes `10.0.0.199`) não é mais usado |
| Sistema Agrobot | RabbitMQ em `10.0.0.96:5672` (usuário `agrobot`, management em `:15672`, UI do manager em `:8090`). O Husky publica telemetria nele pela ponte `agrobot_bridge` (ver `agrobot_husky_nav/README.md`); o drone aparece como agente `x650-jetson` e o Husky como `husky` |
| Foxglove | `ws://10.0.0.60:8765`. URDF em `/a300_00096/robot_description` (adicionar como URDF por tópico no painel 3D; frame `base_link`). Nuvem reduzida para Wi-Fi fraco: `/livox/lidar_lite` (`~/run_dec.sh`) |
| ROS 2 no laptop | Jazzy em `/opt/ros/jazzy`. Para ver o grafo do robô: `ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DISCOVERY_SERVER=10.0.0.60:11811 ROS_SUPER_CLIENT=True`, depois `ros2 daemon stop; ros2 topic list --spin-time 8` |

Sem `sshpass` no laptop: `tools/rssh.py '<comando>' [timeout_s]` roda comandos no robô (pexpect).
Recomendado instalar chave SSH (`ssh-copy-id robot@10.0.0.60`).

## Robô

- Ubuntu 24.04, ROS 2 **Jazzy**, pacotes Clearpath 2.9.x, namespace ROS **`a300_00096`**.
- Discovery: FastDDS Discovery Server em `127.0.0.1:11811`. Em shell não interativo exportar
  `ROS_SUPER_CLIENT=True`; `ros2 topic list` precisa de `--spin-time 8`.
- `/etc/clearpath/robot.yaml` é symlink para `~/colcon_ws/src/senai02_a300/senai02_bringup/config/robot.yaml`
  (pacote customizado da SENAI). Após editar: `sudo systemctl restart clearpath-robot` regenera tudo.
- Workspace `~/colcon_ws/src`: `fixposition_driver` (v6.1.x), `senai02_a300`, `ntrip_client` (LORD), `agrobot_husky_nav`.
- Serviços Clearpath: `clearpath-robot`, `-platform`, `-sensors`, `-discovery`, `-vcan`. `platform-extras` está vazio.
- Odometria de rodas + EKF (`platform/odom/filtered`, frame `odom`) a 50 Hz. **É o único elo comum aos
  dois modos**: contínua, sem saltos, e é sobre ela que o modo híbrido vai se apoiar.
- `cmd_vel` externo entra no `twist_mux` com prioridade 1; joystick 10; RC 12; e-stop e safety stop travam tudo.

### Sensores

| Sensor | Onde | Usado em | Estado |
|---|---|---|---|
| IMU Phidgets Spatial | `sensors/imu_0/data`, 50 Hz | ambos (EKF) | OK |
| Fixposition Vision-RTK 2 (2 antenas) | cabo `192.168.131.35:21000`, driver via `clearpath-sensors`; tópicos `sensors/ins_0/gps_0/fix`, `gps_1/fix` (~5 Hz), `odom` (ENU), `fixposition/odometry_llh`, `fpa/gnsscorr`, `fpa/odomstatus`, `rtcm` (entrada) | **outdoor** | OK. **Fusão desligada** (câmera sem calibração): sem `odom`/`odometry_llh` úteis; usa-se as duas antenas |
| Livox MID360 | `192.168.131.109` → `/livox/lidar` (10 Hz, ~20k pts). Montado **invertido e girado 90°** sob o Fixposition, ~0,61 m do chão; TF no launch: xyz `-0.168 0 0.45`, rpy `π 0 −π/2` (validado com pessoa na frente) | **ambos**: colisão nos dois, mapa no indoor | OK, sobe com `~/start_livox_pc2.sh` (não é serviço) |
| ZED 2i (estéreo, sem SDK) | `/dev/video0` UVC lado a lado → `zed/left|right/image_raw` | indoor (follow-me, inspeção) | OK, só o par estéreo |
| Joystick PS4 | `/dev/input/ps4`, `joy_teleop/joy` 20 Hz; L1 habilita, R1 turbo | **ambos** (segurança) | OK |

## Segurança (igual nos dois modos)

Em camadas, da mais forte para a mais fraca:

1. **E-stop físico** — corta a potência, independe de software.
2. **Joystick** (`twist_mux` prioridade 10 contra 1 do autônomo): segurar **L1/R1** sobrepõe o
   comando autônomo e pausa o nó; **Círculo** aborta; **Options** inicia ou retoma; sem joystick por
   1 s o nó pausa sozinho. A pausa por serviço gruda até um start explícito.
3. **Saúde das entradas** — pose velha, fix fraco (outdoor), sem `scan`, sem mapa ou sem TF (indoor):
   o nó pausa e publica o motivo em `~/status`.
4. **Guarda de obstáculos pelo `scan`** — reduz a velocidade a partir de `obstacle_slow_distance`,
   para em `obstacle_stop_distance`, retoma sozinha. É a última barreira, para o que o mapa não sabe.

> **Pendência conhecida**: o índice do botão **Options** está errado nos parâmetros (o controle expõe
> 15 botões; Círculo = 1, L1 = 4, R1 = 5 confirmados; 9 não inicia). Até resolver, iniciar pelos
> serviços (`~/goto.py --start`, `ros2 service call .../start`).

---

# Modo outdoor: waypoints globais por GNSS RTK

Validado em campo em 2026-09-09. Sem mapa e sem planejador: o robô sabe onde está pelo RTK, aponta
para o próximo waypoint e anda, com o lidar apenas como guarda de colisão.

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

- **Correções RTK via ROS 2**: a fonte de correção do sensor é a entrada I/O (RTCM chega pelo stream
  TCP do driver). Provado: parando o cliente NTRIP a taxa de correção vai a 0 e o fix cai de RTK para
  3D em ~80 s.
- **Internet em campo**: o roteador Agriwing não tem WAN. O laptop (internet pelo iPhone via USB)
  mantém `tools/ntrip_tunnel.py`; no robô `~/ntrip_tunnel_up.sh` aponta o cliente para `127.0.0.1:2101`.
  Se o laptop cair, o RTK degrada em ~80 s e o seguidor pausa sozinho (fix < RTK float).
- **Rumo por antena dupla** validado em campo: erro < 1° contra o deslocamento RTK em 3 m. Precisa de
  RTK (`gnss1_fix`/`gnss2_fix` ≥ 7; 8 = fixed). Baseline 0,46 m.

## Operação em campo (checklist)

1. Ligar o robô e o Fixposition (alimentado pelo painel). Esperar ~2 min. Liberar o e-stop.
2. No laptop (com internet): `python3 tools/ntrip_tunnel.py &` (mantém o túnel; reconecta sozinho).
3. No robô: `~/field_up.sh` (reinicia o driver se o sensor acabou de subir; sobe Livox e NTRIP) e
   `~/ntrip_tunnel_up.sh` (NTRIP pelo túnel). Conferir `fpa/gnsscorr`: `gnss1_fix: 8 gnss2_fix: 8`,
   `corr_update_rate: 1.0`.
4. Subir a navegação sem missão:
   `ros2 launch agrobot_husky_nav nav.launch.py pose_source:=dual_gnss max_linear:=0.5`
   (a 1,0 m/s usar `obstacle_stop_distance:=1.8 obstacle_slow_distance:=4.0`).
5. Gravar waypoints dirigindo (opcional): `~/record_waypoints.py /home/robot/m.yaml`; qualquer botão
   de face do joystick marca um ponto, **Share** desfaz. O log em `/tmp/recorder.log` mostra o índice
   de cada botão apertado, útil porque o mapeamento muda conforme o driver do controle.
6. Validar o rumo uma vez por montagem: `~/heading_check.py start`, andar 3 m reto com o joystick,
   `~/heading_check.py end` (erro deve ser de poucos graus).
7. Missão: `~/make_relative_mission.py /home/robot/m.yaml "5,0" "5,5" "0,5" "0,0"` (frente,esquerda em m)
   ou `~/make_forward_mission.py 5`; carregar com
   `ros2 topic pub --times 5 -r 1 -w 1 /a300_00096/gps_waypoint_follower/load_mission std_msgs/msg/String "{data: /home/robot/m.yaml}"`.
8. Conferir `gps_waypoint_follower/status` (`health: null`, distância e erro de rumo coerentes) e
   iniciar pelo serviço `start` (ou **Options**, quando o índice do botão estiver corrigido).
9. Parar tudo: `~/nav_kill.sh all` (nunca usar `pkill -f` com o texto do launch no mesmo comando:
   mata o próprio shell).

**Resultados de campo (2026-09-09)**: 5 m reto a 0,3 m/s em 18 s (0,58 m do ponto); quadrado de 5 m a
0,5 m/s em 68 s; a 1,0 m/s em 48 s; chegada sempre a ~0,57 m (tolerância 0,6 m); guarda de obstáculos
bloqueou e retomou sozinha nas duas corridas do quadrado. Circuito de 5 waypoints gravado com o
joystick, com retorno ao ponto de partida capturado no início da missão: 0,53 m de erro no retorno.

---

# Modo indoor: mapa 2D, marcadores e exploração

Validado no laboratório em 2026-09-11. Sem GNSS: quem diz onde o robô está é o `slam_toolbox`,
casando o `scan` do MID360 contra o mapa que ele mesmo constrói.

```
Livox MID360 ──> pointcloud_to_laserscan ──> scan (10 Hz) ─┬─> slam_toolbox ──> map + TF map→odom
                                                           │                      │
                                                           │   grid_planner (A*) <─┘
   Foxglove: clique no mapa ──> goal_pose / clicked_point ─┼─> goto_point ─┐
   fronteiras do próprio mapa ────────────────────────────┴─> explore ─────┤
                                                           │               ▼ path_follow (pure pursuit)
                                                           └──> guarda de obstáculos ──> cmd_vel
                             joystick PS4 (prioridade 10) = bypass ──────────────────────┘
```

Três nós, na ordem em que sobem:

```bash
~/start_livox_pc2.sh                                  # nuvem do MID360
ros2 launch agrobot_husky_nav indoor.launch.py        # scan + mapa local voxelizado
ros2 launch agrobot_husky_nav slam.launch.py          # mapa global e frame map
ros2 launch agrobot_husky_nav goto_point.launch.py    # ir até marcador   (um OU outro)
ros2 launch agrobot_husky_nav explore.launch.py       # exploração autônoma
```

- **Construir o mapa**: dirigir devagar pelo ambiente com o joystick, acompanhando `/a300_00096/map`
  no Foxglove, e gravar com `~/save_map.sh`. Ou deixar o `explore` fazer isso sozinho.
- **Ir até um ponto**: clicar no painel 3D do Foxglove. Cada clique entra numa fila, então três
  cliques viram uma rota de três waypoints. Pela linha de comando, `~/goto.py 3.5 -1.2`.
- **Explorar**: o `explore` escolhe fronteiras (livre encostado em desconhecido) e ranqueia pelo
  **caminho real** do A*, não pela distância em linha reta.
- **Como se move**: A* sobre o mapa, caminho suavizado (encurtamento por visada livre, arredondamento
  de quina, reamostragem uniforme) e perseguição por curvatura com rampa de aceleração. Os detalhes e
  o porquê de cada etapa estão em `agrobot_husky_nav/README.md`.

**Resultados de laboratório (2026-09-11)**: marcador a 15,75 m em linha reta, caminho de 18,6 m,
percorrido em ~62 s a 0,3 m/s, incluindo vão estreito (obstáculo frontal a 0,60 m) sem travar.
Suavidade medida em 100 s de corrida com dois marcadores: desvio padrão do comando angular de
0,064 rad/s e 11 trocas de sinal, a 0,19 m/s de média. Exploração autônoma: 8 destinos seguidos, área
conhecida de 137,5 para 172,5 m².

Também no laboratório, fora da navegação: par estéreo da ZED 2i (`zed.launch.py`), mapa local
voxelizado do MID360 (`indoor.launch.py`) e follow-me por marcador ArUco (`follow_me.launch.py`).

---

# Modo híbrido (planejado)

O objetivo é uma missão só, que comece no galpão, saia para o campo e volte, sem o operador trocar de
sistema no meio. Hoje os dois modos **não se falam**: cada um tem a sua noção de "onde estou", e não
existe transformada entre elas.

**O que já ajuda**: a odometria fundida (`odom`) é contínua e comum aos dois modos, o MID360 serve aos
dois, a camada de segurança é a mesma e os dois seguidores já publicam estado em JSON no mesmo
formato. O planejador e o seguidor do indoor não dependem de SLAM: dependem de uma grade de ocupação
e de uma TF até `base_link`, que também podem vir de outra fonte.

**O que falta**, em ordem de dependência:

1. **Georreferenciar o mapa do SLAM** — guardar junto do mapa a lat/lon e o rumo da origem dele
   (capturados com RTK fixed na hora de criar o mapa). Sem essa âncora não há como dizer que um
   waypoint lat/lon fica "ali" no mapa.
2. **Uma árvore de TF única** — `earth → map → odom → base_link`, com o SLAM publicando `map→odom`
   indoor e um nó GNSS publicando o mesmo elo outdoor. A transição passa a ser uma troca de **quem
   publica `map→odom`**, não uma troca de controlador. Como `odom` é contínua, o robô não salta.
3. **Um supervisor dono do `cmd_vel`** — hoje a regra é "um controlador por vez, encerre o outro".
   No híbrido, um supervisor escolhe a fonte e mantém a garantia de publicador único, que é o que
   evita dois controladores disputando o robô.
4. **Critério de transição** — candidatos: qualidade do fix (RTK fixed por N segundos), qualidade do
   casamento do SLAM, densidade de retornos do lidar (céu aberto devolve pouco), ou uma cerca
   geográfica desenhada à mão na entrada do galpão. Precisa de histerese, senão ele oscila na porta.
5. **Formato de missão único** — um waypoint passa a declarar o seu frame (`map` ou `wgs84`), e a
   missão vira uma lista que pode misturar os dois.

**Decisões em aberto**: se o modo híbrido usa o mapa do SLAM em campo (o MID360 enxerga pouco em
lavoura aberta) ou só a guarda de colisão; se a transição é automática ou confirmada pelo operador; e
se a fusão do Fixposition volta a ser usada (hoje desligada, ver abaixo), porque ela daria pose
contínua mesmo sem RTK fixed e simplificaria muito o item 4.

---

## Problemas conhecidos e decisões

**Outdoor**

- **Fusão do Fixposition parada**: a câmera não está calibrada e a fusão dá erro; decisão de não usar
  por ora. `fusion autostart` ficou habilitado na API mas o start falha (`request fail`). Consequência:
  só o modo `dual_gnss`, que exige RTK. Calibrar a câmera devolve rumo e pose mesmo sem RTK fixed.
- **Rede do sensor**: resolvido em 2026-09-09 ligando o autoconnect da conexão cabeada pela API (ver
  Acesso). Se sumir de novo: `curl http://10.0.0.199/api/v2/net/status` pelo Wi-Fi do sensor, ou
  `ip neigh`/`tcpdump -i br0` no robô para ver se ele fala alguma coisa no cabo.
- **API do sensor**: `rtk_set` exige o objeto completo de `rtk_get` e só vale após
  `ctrl/action {"rtk":"restart"}`; com fonte I/O o `rtk_status` mostra "Connected to localhost".
  Voltar ao cliente interno: `source: ntripcli` + credenciais.
- **Fixposition pelo Wi-Fi** gerava atrasos de 0,1–0,4 s no driver; no cabo, zero avisos.
- Credenciais NTRIP (IBGE) ficam **só no robô** (`~/ntrip_ibge.yaml`, `~/ntrip_tunnel.yaml`); no
  repositório há apenas o modelo.

**Indoor**

- **`slam_toolbox` é nó de ciclo de vida** e sobe em `unconfigured`, sem assinar nada e sem reclamar.
  O `slam.launch.py` emite `configure` e `activate` sozinho.
- **Freio de proximidade conservador**: com obstáculo a ~0,87 m no cone frontal o robô anda a
  0,07-0,09 m/s. Não é travamento; se for demais para o ambiente, baixar `obstacle_slow_distance`,
  nunca `obstacle_stop_distance`.
- **FOV vertical do MID360 invertido**: só 7° acima do lidar (~0,85 m do chão); obstáculos altos e
  finos só de perto. `range_min: 0.6` remove a própria estrutura do robô.

**Ambos**

- **Livox e NTRIP não são serviços**: precisam do `field_up.sh` após cada boot (pendência: systemd).
- **`restart_livox.sh` pode deixar instâncias duplicadas** do driver (as portas UDP entram em conflito
  e a nuvem para). `nav_kill.sh livox` limpa.
- **Um publicador em `cmd_vel`**: os padrões do `nav_kill.sh` já casaram errado uma vez
  (`"agrobot_husky_nav explore"` não aparece na linha de comando do executável instalado) e três
  instâncias sobreviveram publicando juntas. Conferir sempre com `ros2 topic info`.
- **Deploy**: `scp -r` em diretório existente aninha a cópia; `deploy.py` apaga a fonte remota antes.
- **Foxglove**: a allowlist de assets do Clearpath só aceita extensões minúsculas (`sensor_arch.STL`
  foi duplicada como `.stl`), e o gerador serializa listas com `repr()`, o que dobra as barras
  invertidas de uma regex e quebra a busca das meshes.

## Interface web de teste (`webui/`)

Página estática servida pelo próprio robô: **http://10.0.0.60:8088** (o `field_up.sh` sobe um
`python3 -m http.server 8088` em `/home/robot/webui`). Também funciona abrindo `webui/index.html`
direto do laptop. Ela fala com o **Foxglove bridge** do robô (`ws://10.0.0.60:8765`, subprotocolo
`foxglove.sdk.v1`), sem nenhum servidor extra. É uma ferramenta do **modo outdoor**:

- assina `gps_waypoint_follower/status` (posição, rumo, estado, saúde, obstáculo, fix RTK, joystick, waypoints);
- desenha um mapa local em metros (norte para cima, origem na primeira pose ou "origem aqui"), com rastro,
  seta do robô, arco do obstáculo mais próximo e os waypoints da missão em curso;
- cria waypoints clicando no mapa, por atalhos relativos ao rumo atual (5 m frente, quadrado 5 m, fileiras)
  ou editando o YAML; **Enviar missão** publica em `gps_waypoint_follower/set_mission` (JSON via `clientPublish`);
- **Start / Pause / Stop** chamam os serviços do seguidor (requisição `Trigger` em CDR).

Testada em 2026-09-10 com um cliente Node contra o robô (assinatura, envio de missão, chamada de serviço).
Requer o `nav.launch.py` no ar; sem ele a página mostra "seguidor fora do ar". Não substitui o joystick:
L1/Círculo continuam sendo o bypass de emergência.

No **modo indoor** o painel equivalente é o próprio Foxglove: mapa, caminho planejado (`goto_point/path`),
fila de marcadores (`goto_point/markers`) e a mesh do robô pelo URDF em `/a300_00096/robot_description`.
