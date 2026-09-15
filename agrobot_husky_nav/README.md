# agrobot_husky_nav

Stack leve de navegação do Husky A300, **sem Nav2**, em dois modos: **outdoor** por waypoints GNSS
RTK, sem mapa (validado em campo em 2026-09-09), e **indoor** sobre o mapa do SLAM, com planejamento
A* (validado no laboratório em 2026-09-11). O `../README.md` explica os dois modos, a diferença entre
eles e o modo híbrido planejado; aqui está o uso do pacote.

Onde procurar cada coisa:

| Assunto | Modo | Seção |
|---|---|---|
| Seguidor de waypoints, missões, fontes de pose | outdoor | *Nós e tópicos*, *Launches*, *Missões* |
| Correções RTK pelo ROS 2 (NTRIP) | outdoor | *Correções RTK via ROS 2* |
| Mapa global 2D e como salvá-lo | indoor | *Mapeamento global 2D* |
| Ir até um marcador clicado no mapa | indoor | *Ir até um marcador no mapa* |
| Exploração autônoma por fronteiras | indoor | *Exploração autônoma por fronteiras* |
| Planejador A*, suavização e seguimento | indoor | *Suavidade do movimento* |
| Mapa local voxelizado, follow-me, ZED estéreo | indoor | seções próprias |
| Telemetria para o sistema Agrobot | ambos | *Telemetria para o sistema Agrobot* |

**Vale nos dois modos**: só um nó pode publicar em `cmd_vel` por vez. `gps_waypoint_follower`,
`goto_point`, `explore` e `follow_me` disputam o mesmo tópico; `~/nav_kill.sh nav|goto|explore|follow|all`
encerra o que não for usar, e `ros2 topic info /a300_00096/cmd_vel` confirma que sobrou um.

## Scripts de operação (`scripts/`)

Rodam **no robô**. O `colcon build` os instala em `lib/agrobot_husky_nav`, então valem tanto
`~/nome` (atalho criado pelo deploy) quanto `ros2 run agrobot_husky_nav nome`.

| Script | Modo | O que faz |
|---|---|---|
| `field_up.sh` | outdoor | Sobe o que o teste de campo precisa: reinicia o driver do Fixposition se ele acabou de aparecer, Livox, NTRIP, ponte de telemetria e a interface web. `BRIDGE=0` pula a ponte |
| `indoor_up.sh` | indoor | Sobe a pilha indoor: nuvem, scan, mapa local e SLAM, mais `goto` ou `explore` se pedido. `MAPA=<nome>` continua de um mapa salvo, em modo localização |
| `nav_kill.sh` | ambos | Encerra o que estiver rodando: `nav`, `ntrip`, `livox`, `bridge`, `indoor`, `follow`, `explore`, `goto`, `slam`, `zed` ou `all` |
| `save_map.sh` | indoor | Grava o mapa em `~/maps`: `.pgm`/`.yaml` para o servidor de mapas e `.data`/`.posegraph` para continuar o mapeamento |
| `goto.py` | indoor | Manda o robô a um ponto do mapa sem depender do Foxglove; também `--status`, `--clear`, `--skip` |
| `record_waypoints.py` | outdoor | Grava waypoints dirigindo com o joystick |
| `make_forward_mission.py`, `make_relative_mission.py` | outdoor | Montam missões a partir da pose atual |
| `heading_check.py` | outdoor | Confere o rumo das duas antenas contra o deslocamento medido |
| `fixposition_wired.sh` | outdoor | Acerta a rede cabeada do Fixposition pela API |
| `ntrip_tunnel_up.sh` | outdoor | Aponta o cliente NTRIP para o túnel SSH do laptop |
| `start_livox_pc2.sh`, `restart_livox.sh`, `decimate.py`, `run_dec.sh`, `start_decimator.sh` | ambos | Driver do MID360 e a nuvem reduzida para Wi-Fi fraco |

O `field_up.sh` e o `indoor_up.sh` sobem também o `urdf_beacon`, sem o qual o robô desaparece do
painel 3D do Foxglove de tempos em tempos. O motivo está no cabeçalho de `urdf_beacon.py`: o URDF é
publicado uma vez só e a ponte pode ter assinado o tópico como volátil, caso em que a mensagem
retida nunca chega. A camada de URDF do Foxglove deve apontar para `robot_description_foxglove`.

Uma armadilha vale a pena registrar: o `field_up.sh` exporta `FASTRTPS_DEFAULT_PROFILES_FILE`
apontando para `~/fastdds_big_msg.xml`, e esse perfil **bloqueia a recepção de tópicos
transient_local**. O `map` do SLAM é um deles, então o `indoor_up.sh` explicitamente não exporta
esse perfil. Um shell que herdou a variável vê o planejador reclamar de "sem mapa do SLAM" com o
SLAM perfeitamente no ar.

## Nós e tópicos (namespace `a300_00096`)

| Nó | Entrada | Saída |
|---|---|---|
| `livox_static_tf` | — | TF `base_link → livox_frame` (MID360 invertido e girado 90°: xyz `-0.168 0 0.45`, rpy `π 0 −π/2`) |
| `livox_to_scan` (`pointcloud_to_laserscan`) | `/livox/lidar` | `scan` em `base_link`, 720 feixes, 0,6–20 m, alturas 0,2–1,2 m |
| `gps_waypoint_follower` | pose (ver fontes), `scan`, `joy_teleop/joy`, `~/load_mission` (String: caminho do YAML), `~/set_mission` (String: YAML/JSON da missão inline, usado pela web UI) | `cmd_vel` (TwistStamped), `~/status` (String JSON a 2 Hz); serviços `~/start`, `~/pause`, `~/stop` (Trigger) |
| `agrobot_bridge` | pose das 2 antenas, `platform/bms/state`, `platform/odom/filtered`, `gps_waypoint_follower/status` | **AMQP direto** para o RabbitMQ do sistema Agrobot: filas `agent_telemetry` (1 s), `agent_keep_alive` (10 s), `agent_details` (20 s) |
| `follow_me` | `zed/left/image_raw` + `camera_info`, `scan`, `joy_teleop/joy` | `cmd_vel`, `follow_me/status` (JSON), `follow_me/debug_image`; serviços `start`/`pause`/`stop` |
| `explore` | `map` (SLAM), `scan`, TF `map→base_link`, `joy_teleop/joy` | `cmd_vel`, `explore/status` (JSON); serviços `start`/`pause`/`stop` |
| `goto_point` | `map` (SLAM), `scan`, TF `map→base_link`, `joy_teleop/joy`, marcadores (`goal_pose`, `clicked_point`, `~/set_goal`) | `cmd_vel`, `goto_point/status` (JSON), `goto_point/path`, `goto_point/markers`; serviços `start`/`pause`/`stop`/`clear`/`skip` |
| `slam_toolbox` | `scan`, TF `odom→base_link` | frame **`map`**, transformada `map→odom` e `map` (OccupancyGrid global) |
| `voxel_local_map` | `/livox/lidar`, `platform/odom/filtered` | `voxel_cloud` (PointCloud2 voxelizada) e `local_map` (OccupancyGrid, janela rolante 20x20 m) |
| `urdf_beacon` | `robot_description` (retido) | `robot_description_foxglove` a cada 5 s, para o painel 3D do Foxglove enxergar o robô mesmo quando a ponte assina como volátil |
| `zed_stereo` | `/dev/video0` (ZED 2i como UVC, quadro lado a lado) | `zed/{left,right}/image_raw`, `.../compressed`, `.../camera_info`; serviço `zed/save` grava o par em PNG |
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

## Telemetria para o sistema Agrobot (`agrobot_bridge`)

O drone `x650-jetson` publica os 9 tópicos do contrato **`Agrobot_v2`** como tópicos ROS globais, que um
conversor `rabbitros` leva até o RabbitMQ. O Husky **não pode** fazer igual: ele roda atrás de um FastDDS
Discovery Server, então os tópicos dele não aparecem no grafo multicast onde aquele conversor escuta.
Por isso a ponte do Husky fala **AMQP direto** com o broker.

```bash
export PYTHONPATH=/home/robot/pylibs:$PYTHONPATH        # pika mora fora dos pacotes do sistema
ros2 run agrobot_husky_nav agrobot_bridge --ros-args \
  --params-file $(ros2 pkg prefix agrobot_husky_nav)/share/agrobot_husky_nav/config/agrobot_bridge.yaml \
  -r __ns:=/a300_00096
# ou simplesmente: ~/field_up.sh   (BRIDGE=0 ~/field_up.sh pula a ponte)
```

Envelope: `{source, destination:"service", agentId, protocol:"ROS2", version:"Agrobot_v2", correlationId, timestamp, seq, payload}`.
O drone manda só `source`/`destination`; o orquestrador, porém, casa a telemetria com a linha do banco por
**`agentId` numérico** (`updateByTelemetry` faz `where id = Number(agentId)`), então a ponte manda os quatro campos.

**O agente precisa estar cadastrado na UI antes** (http://10.0.0.96:8090/agents): sem a linha em `agents`
nada aparece, por mais que o `agent_keep_alive` esteja saindo. O Husky foi cadastrado em 2026-09-09 como
**id 34**, `destination_id: "husky"`, serial `a300-00096`. No YAML: `agent_id: '34'` e `agent_source: 'husky'`
(que é o `destination_id`). Ao recriar o agente, atualizar os dois campos.

| Fila | Período | Payload |
|---|---|---|
| `agent_telemetry` | 1 s | `position{lat,lng,altitude}`, `battery` (%), `speed` (m/s), `heading` (bússola, 0 = norte), `status` |
| `agent_keep_alive` | 10 s | `name`, `serialNumber`, `battery`, `status` |
| `agent_details` | 20 s | ficha do agente e `supportedActivities` |

`status` sai do estado do seguidor: RUNNING/BLOCKED/PAUSED → `IN_MISSION`; IDLE/DONE/ABORTED → `IDLE`
(em `agent_keep_alive`, `ONLINE`); sem pose válida → `ERROR`. Posição e rumo vêm das duas antenas do
Fixposition, então a telemetria funciona mesmo sem missão e sem o seguidor no ar.
Os números de `physical` no YAML são estimativas e estão marcados com `# conferir`.

## Câmera estéreo ZED 2i (`zed_stereo`)

Captura **só o par estéreo**, sem o SDK da Stereolabs, sem CUDA e sem nenhuma função de IA.
A ZED 2i aparece como câmera UVC comum em `/dev/video0` e entrega um único quadro com as duas
imagens lado a lado; o nó corta ao meio e publica cada lado com o `CameraInfo` da calibração de fábrica.

```bash
ros2 launch agrobot_husky_nav zed.launch.py                      # HD 1280x720 por olho, 30 fps
ros2 launch agrobot_husky_nav zed.launch.py resolution:=FHD fps:=30
ros2 launch agrobot_husky_nav zed.launch.py rectify:=true        # + stereo_image_proc (disparidade e nuvem)
ros2 service call /a300_00096/zed/save std_srvs/srv/Trigger      # grava o par em /home/robot/zed_captures
```

| Resolução | Por olho | Quadro UVC | fps |
|---|---|---|---|
| `VGA` | 672x376 | 1344x376 | até 100 |
| `HD` (padrão) | 1280x720 | 2560x720 | até 60 |
| `FHD` | 1920x1080 | 3840x1080 | 30 |
| `2K` | 2208x1242 | 4416x1242 | 15 |

**Calibração**: baixada por número de série (esta câmera é a **SN 33943780**) e guardada em
`../calib/zed_SN33943780.conf`, instalada no robô como `/home/robot/zed_calib.conf`:

```bash
curl -o zed_calib.conf "https://calib.stereolabs.com/?SN=33943780"
```

O nó lê as intrínsecas da resolução escolhida e a seção `[STEREO]` (linha de base de 120,0 mm),
roda `cv2.stereoRectify` e preenche `K`, `D`, `R` e `P` no `CameraInfo`. Por isso o `stereo_image_proc`
funciona direto por cima, sem calibrar nada à mão. Sem o arquivo o nó ainda publica as imagens,
mas com `CameraInfo` vazio.

**Banda**: `image_raw` é 2,7 MB por quadro por lado; use os tópicos `/compressed` (JPEG, qualidade 80)
para ver de fora do robô. Medido em bancada: 29,5 fps de captura, ~25 fps no tópico raw e 29,8 no comprimido.

**Foxglove — "X" atravessando a imagem**: não é defeito da câmera nem da compressão. Quando o painel
recebe o `camera_info`, o Foxglove desenha a geometria da câmera por cima, e de frente esse desenho vira um
retângulo com as diagonais cruzadas. Desmarcar a calibração no painel resolve (confirmado em 2026-09-11).
Diagnóstico feito na época: 1154 quadros analisados sem nenhum JPEG inválido e sem nenhuma linha diagonal
longa, PNG cru e JPEG do tópico limpos, USB 3.0 sem erro. Ou seja, se aparecer algo estranho na imagem,
vale primeiro checar se é sobreposição do visualizador.

**TF**: `publish_tf` vem desligado porque a montagem ainda não foi medida. Ao medir, passar
`publish_tf:=true zed_xyz:="x y z" zed_rpy:="r p y"` (o frame óptico usa a convenção de visão:
z para frente, x para a direita, y para baixo).

## Mapa local voxelizado para indoor (`voxel_local_map`)

O MID360 varre de forma **não repetitiva**: um quadro sozinho cobre pouco, e é a soma de vários
que preenche a cena. O nó acumula uma janela curta **compensando o movimento pela odometria**,
voxeliza e publica a nuvem reduzida mais uma grade de ocupação pronta para um planejador local.

```bash
ros2 launch agrobot_husky_nav indoor.launch.py              # + scan 2D (scan:=false desliga)
ros2 launch agrobot_husky_nav indoor.launch.py scan:=false
```

Duas decisões que fizeram diferença, ambas medidas em bancada no laboratório:

- **Filtro do próprio robô.** Montado de cabeça para baixo, o MID360 enxerga o top plate e o arco:
  72% dos retornos vinham de menos de 1,5 m, com uma massa densa entre 0,3 e 0,6 m de altura em
  volta do robô. Um raio mínimo não resolve (o corpo não é circular), então o filtro é uma **caixa**
  em `base_link` (`self_filter_x/y/z_max`) cobrindo o A300, que tem 0,99 x 0,67 m.
- **Chão por plano ajustado, não por altura fixa.** O conjunto sensor mais piso tem cerca de 1,5° de
  inclinação; a 8 m isso vira 20 cm de erro, e um limiar fixo classificaria chão distante como obstáculo.
  O nó ajusta `z = a·x + b·y + c` aos pontos baixos a cada ciclo (com suavização) e classifica por
  **altura acima desse plano**: `obstacle_min_height` a `obstacle_max_height` é obstáculo,
  abaixo de `ground_max_height` é chão (célula livre), o resto fica desconhecido.

Validação da montagem (feita com o robô parado no laboratório): plano de chão ajustado deu altura de
**-0,163 m** sob o robô contra -0,165 nominal, inclinação de 1,1° e resíduo de 4,7 cm. A nuvem
voxelizada foi comparada setor a setor com o `scan` 2D (que usa TF e já havia sido validado com uma
pessoa na frente): diferença de **0 a 5 cm** nos setores frontais.

Números típicos de bancada: 85 mil pontos na janela de 0,6 s viram ~5,4 mil voxels de 10 cm,
resultando em ~890 células ocupadas e ~1750 livres, a 5 mapas por segundo.

Para planejar direto na grade, ligue `inflation_m` com o raio do robô (~0,45 m).

## Follow-me por marcador ArUco (`follow_me`)

O robô segue uma pessoa que carrega um **marcador ArUco** impresso. Sem rede neural e sem o SDK
da ZED: com a calibração de fábrica e o tamanho real do marcador, uma única imagem já dá a posição
3D do alvo. O lidar não entra no laço de controle, fica só como segurança.

```bash
python3 tools/make_aruco.py 0 15            # gera o PNG para imprimir em A4 (id 0, lado 15 cm)
ros2 launch agrobot_husky_nav follow_me.launch.py
ros2 service call /a300_00096/follow_me/start std_srvs/srv/Trigger   # ou Options no joystick
```

Precisa da câmera e do scan no ar: `zed.launch.py` e `indoor.launch.py` (ou `nav.launch.py`).

**Controle**: gira para manter o alvo centrado (banda morta de 4°) e avança ou recua para manter
`target_distance` (1,5 m, banda morta de 20 cm). Só avança com o alvo dentro de ±60°, para girar
primeiro e andar depois. Velocidades limitadas por `max_linear` (0,4 m/s indoor) e `max_angular`.

**Segurança**, na mesma lógica do seguidor de waypoints: segurar **L1** pausa e devolve o controle,
**Círculo** aborta, **Options** inicia ou retoma; sem joystick por 1 s, pausa. Estados: `IDLE`,
`FOLLOWING`, `LOST` (marcador sumiu por mais de 1,5 s), `BLOCKED`, `PAUSED`, `ABORTED`.

A guarda de obstáculos **ignora a pessoa que está sendo seguida**, senão o robô pararia por causa
dela. Um setor angular fixo não basta: a 1 m de distância, ±20° cobrem só 36 cm e os ombros ficam de
fora — foi o que aconteceu no primeiro teste, com o robô entrando em `BLOCKED` três vezes por causa do
próprio operador. Agora o setor é calculado pela **largura da pessoa na distância em que ela está**
(`target_width_m`) e só vale para retornos que estejam **à mesma distância** do alvo
(`target_mask_range_m`); qualquer coisa em outra distância continua bloqueando normalmente.

**Primeiro teste com movimento** (laboratório, 2026-09-11, operador andando com o marcador na mão):
2 minutos seguindo, mantendo a distância em torno de 1,5 m — avançou até 0,40 m/s quando o alvo se
afastou para 2,3 m e **recuou** a -0,18 m/s quando chegou a 1,2 m, girando até ±0,43 rad/s para manter
o alinhamento. Entrou em `BLOCKED` e voltou sozinho, e recuperou de `LOST` quando o marcador saiu de vista.

**Validado com marcador real** (laboratório, 2026-09-11, marcador de 17 cm a ~2,6 m):
231 detecções em 236 quadros, distância e rumo estáveis (2,61 m e -6,6°, variação de 1 cm entre leituras).
Conferido contra o lidar no mesmo rumo: o `scan` deu 2,56 m de mediana, ou seja **5 cm de diferença**
entre câmera e lidar — o que valida de uma vez o tamanho do marcador, a calibração e o deslocamento
`camera_xyz` estimado. Para identificar o dicionário e o id de um marcador desconhecido, use o mesmo
truque: pegue um quadro e rode `detectMarkers` em todos os `cv2.aruco.DICT_*`.

**Antes do primeiro teste de movimento**: conferir com régua o lado do marcador (`marker_size_m`) e
medir a pose da ZED em `base_link` (`camera_xyz`, `camera_yaw_deg`) — o valor atual é estimado, mas a
comparação com o lidar mostrou erro de apenas 5 cm.

**Atenção**: `follow_me` e `gps_waypoint_follower` publicam os dois em `cmd_vel`. Não rode os dois
ao mesmo tempo; `~/nav_kill.sh nav` ou `~/nav_kill.sh follow` encerram um deles.

**Armadilha resolvida**: o `cv_bridge` desta distribuição é compilado contra o OpenCV 4.6 do sistema
e, carregado junto com o OpenCV 4.10 do pip no mesmo processo, derruba o nó com falha de segmentação.
Por isso a conversão de `sensor_msgs/Image` para numpy aqui é manual, sem `cv_bridge`.

## Mapeamento global 2D (`slam.launch.py`)

Constrói o mapa do prédio com o `slam_toolbox` a partir do mesmo `scan` que já sai do MID360.
O MID360 é 3D, mas para navegar dentro de prédio o que interessa é a planta, e o scan já vem
360° em `base_link`, filtrado por altura. Dá fechamento de laço e, de brinde, cria o frame
**`map`** — que é o que faltava para o painel 3D do Foxglove ter um referencial fixo.

```bash
ros2 launch agrobot_husky_nav indoor.launch.py     # precisa do scan no ar
ros2 launch agrobot_husky_nav slam.launch.py       # sobe já ativo
# dirija o robô devagar pelo ambiente e acompanhe /a300_00096/map no Foxglove
~/save_map.sh nome_do_mapa                          # grava em /home/robot/maps
```

`save_map.sh` gera quatro arquivos: `.pgm` e `.yaml` (imagem do mapa, que o `nav2_map_server` lê)
e `.data`/`.posegraph` (grafo de poses, para **continuar o mapeamento depois** ou rodar em modo
localização). Para localizar num mapa já feito:

```bash
ros2 launch agrobot_husky_nav slam.launch.py mode:=localization map_file:=/home/robot/maps/nome_do_mapa
```

**Detalhe que custou tempo**: o `slam_toolbox` é um **nó de ciclo de vida**. Ele sobe em
`unconfigured` e não assina absolutamente nada — nem o scan — até ser levado a `active`. Sem isso
o nó parece vivo, não reclama de nada e simplesmente não faz mapa. O `slam.launch.py` já emite as
transições `configure` e `activate` (argumento `autostart`, ligado por padrão). Para conferir:
`ros2 lifecycle get /a300_00096/slam_toolbox` deve responder `active [3]`.

Parâmetros que valem ajuste em `config/slam.yaml`: `resolution` (5 cm), `max_laser_range` (20 m),
`minimum_travel_distance`/`minimum_travel_heading` (a cada 30 cm ou ~17° ele processa uma varredura)
e `loop_search_maximum_distance` (3 m).

## Exploração autônoma por fronteiras (`explore.launch.py`)

Fecha o ciclo do mapeamento: o SLAM diz o que já é conhecido, e este nó decide **para onde ir para
conhecer mais**. O mesmo MID360 serve às duas pontas — a planta escolhe o destino, o `scan` a 10 Hz
cuida do desvio imediato.

```bash
ros2 launch agrobot_husky_nav indoor.launch.py     # scan
ros2 launch agrobot_husky_nav slam.launch.py       # mapa + frame map
ros2 launch agrobot_husky_nav explore.launch.py    # não inicia sozinho
ros2 service call /a300_00096/explore/start std_srvs/srv/Trigger   # ou Options no joystick
```

**Como escolhe o destino**: fronteira é uma célula livre encostada em célula desconhecida, isto é, a
borda do que se sabe. O nó infla os obstáculos pelo raio do robô (para não mirar um ponto colado na
parede), agrupa as fronteiras restantes, descarta grupos menores que `min_frontier_cells` e escolhe o
de melhor relação entre tamanho e proximidade (`tamanho / (1 + d²)`). Quando não sobra nenhuma
fronteira, o estado vai para `DONE`: o mapa fechou.

**Como se move**: planejamento A* sobre o próprio mapa (`grid_planner.py`), seguido por
perseguição de ponto à frente (`path_follow.py`). A primeira versão era reativa (rumo ao destino
somado à repulsão do scan) e **ficou presa contra uma parede** no laboratório, oscilando: repulsão
local não contorna obstáculo côncavo. O A* entrega um caminho que já passa longe das paredes,
replanejado a cada `replan_period`. O scan continua como última barreira, para o que o mapa ainda
não viu. Suavização do caminho e controle por curvatura: ver a seção sobre `grid_planner.py` e
`path_follow.py` abaixo.

**Como escolhe entre fronteiras**: pelo **caminho real**, não pela distância em linha reta. Medido
no laboratório: a fronteira a 9,3 m em linha reta exigia 45,7 m de caminho (estava atrás de uma
parede), enquanto uma a 22,8 m precisava de 24,8 m — ordenar por linha reta mandava o robô
justamente para a pior escolha. O nó planeja para as `max_candidates` melhores candidatas e fica
com a de menor caminho.

**Antitravamento**: o progresso é medido pela **aproximação do destino** (`progress_distance` em
`progress_time`), não por deslocamento qualquer, senão girar no lugar contaria como progresso.
Destino sem caminho possível entra em lista negra por `blacklist_time`. Cada destino também tem
`goal_timeout`.

**Segurança**, igual aos demais: segurar **L1** pausa e devolve o controle, **Círculo** aborta,
**Options** inicia ou retoma; sem joystick, sem scan, sem mapa ou sem TF, ele para. A pausa por
serviço gruda até um start explícito.

**Atenção**: `explore`, `goto_point`, `follow_me` e `gps_waypoint_follower` publicam todos em
`cmd_vel`. Rode **um de cada vez**; `~/nav_kill.sh explore|goto|follow|nav` encerra o que não for
usar. Confira com `ros2 topic info /a300_00096/cmd_vel`: mais de um publicador significa dois
controladores disputando o robô.

## Ir até um marcador no mapa (`goto_point.launch.py`)

O caso mais direto do uso indoor: o operador olha o mapa do SLAM no Foxglove, clica num lugar e o
robô vai até lá. É o mesmo miolo da exploração (A* do `grid_planner.py` + perseguição de ponto à
frente + parada pelo scan), só que **quem escolhe o destino é a pessoa**.

```bash
ros2 launch agrobot_husky_nav indoor.launch.py      # scan
ros2 launch agrobot_husky_nav slam.launch.py        # mapa + frame map
ros2 launch agrobot_husky_nav goto_point.launch.py  # anda assim que receber um marcador
```

**Pelo Foxglove**: no painel 3D, ferramenta de publicar. Arrastando, sai um `PoseStamped` (ponto e
rumo); clicando, um `PointStamped` (só o ponto). O nó escuta os dois dentro do namespace
(`/a300_00096/goal_pose`, `/a300_00096/clicked_point`) **e** na raiz (`/goal_pose`,
`/clicked_point`, `/move_base_simple/goal`), porque o painel costuma publicar sem prefixo. O clique
é convertido para o frame do mapa pela TF, então funciona com o painel em qualquer frame.

**Pela linha de comando** (`~/goto.py`, roda no robô):

```bash
./goto.py 3.5 -1.2                 # vai para (3.5, -1.2) no frame do mapa
./goto.py 3.5 -1.2 --yaw 90        # chega e gira para 90 graus
./goto.py 2 0 --frame base_link    # dois metros à frente de onde o robô está agora
./goto.py 1 0 --then 2 2 --then 0 3   # rota de três pontos
./goto.py --status | --clear | --skip | --stop | --start
```

**Fila**: cada marcador entra numa fila e é visitado na ordem, então três cliques viram uma rota de
três waypoints (`goal_mode: replace` troca por "vá só para o último clique"). Os marcadores saem em
`~/markers` numerados, para aparecerem no painel 3D; o caminho planejado, em `~/path`.

**Chegada**: `goal_tolerance` de 35 cm, porque um clique não é uma medição. Se o marcador trouxe
rumo, o robô gira no lugar para ele ao chegar (`align_final_yaw`). O destino pode cair dentro de uma
parede ou da margem de segurança: nesse caso o planejador desliza para a célula livre mais próxima
em vez de recusar.

**Desistência**: sem caminho por `plan_fail_limit` tentativas, parado contra obstáculo por
`blocked_timeout`, sem se aproximar `progress_distance` em `progress_time`, ou `goal_timeout`
estourado — o marcador é descartado e a fila segue. **O tempo de pausa não conta**: quando o
operador segura L1, quando o joystick some ou quando o SLAM engasga, os prazos são adiados pelo
tempo parado e a medida de progresso recomeça na retomada. Sem isso uma pausa de 37 s bastava para
o nó acordar convencido de que não houve progresso e descartar um marcador intocado — foi o que
aconteceu no laboratório em 2026-09-11. O `~/status` mostra `paused_s` enquanto a pausa dura. Diferente do `explore`, aqui `allow_unknown` é
`false`: com mapa pronto, não se atravessa o que não foi mapeado.

**Segurança**, igual aos demais: segurar **L1/R1** pausa e devolve o controle, **Círculo** aborta;
sem joystick, sem scan, sem mapa ou sem TF, ele para. A pausa por serviço gruda até um start
explícito.

**Validado no laboratório em 2026-09-11**: marcador a 15,75 m em linha reta, caminho de 18,6 m,
percorrido em ~62 s a 0,3 m/s, incluindo passagem por vão estreito (obstáculo frontal a 0,60 m) sem
travar, seguindo direto para o marcador seguinte da fila. Planejamento em 4-7 ms sobre um mapa de
534x1083 células.

## Suavidade do movimento (`grid_planner.py` + `path_follow.py`)

Na primeira corrida o robô chegava aos marcadores, mas **serpenteava** o caminho inteiro. Duas
causas, uma no caminho e outra no controle.

**O caminho era uma escada.** Numa grade de 10 cm com 8 vizinhos, o A* só sabe andar em múltiplos
de 45 graus: um corredor diagonal vira degrau-degrau-degrau. Seguir isso é seguir um zigue-zague.
Agora o caminho passa por três etapas antes de sair do planejador:

1. **encurtamento por visada livre** — pontos que se enxergam viram um trecho reto (Bresenham sobre
   a grade de proibição). Uma escada de sete células vira dois pontos;
2. **arredondamento de quina** (Chaikin, `smooth_iterations`) — ponto que cairia em célula proibida
   é descartado, melhor um canto vivo do que raspar a parede;
3. **reamostragem uniforme** (`sample_step`) — a perseguição mede distância ao longo do caminho, e
   trecho desigual faria o ponto perseguido saltar. Vale também no trecho reto de dois pontos: sem
   ela o ponto perseguido seria o próprio destino, a metros de distância, e o desvio lateral seria
   corrigido devagar demais.

O encurtamento nunca aproxima o caminho mais da parede do que o A* já havia aceitado: a visada só
vale se o custo máximo ao longo dela não passar do custo máximo do trecho original. Sem essa
condição a suavização comeria a folga de segurança.

Medido sobre o mapa do laboratório, num destino a 4 m: **107 graus de mudança de rumo por metro no
caminho cru, 2,3 graus por metro no suavizado**, com o comprimento caindo de 4,2 para 4,0 m e o
planejamento ainda em 6 ms.

**O controle caçava o rumo.** O comando angular era `w = k · erro_de_rumo`, que zera só quando o
robô aponta exatamente para o ponto perseguido — então ele passa do ponto, corrige para o outro
lado e repete. Pure pursuit de verdade é geométrico: existe um arco que sai da pose atual e chega
no ponto perseguido, de curvatura `κ = 2·sen(α)/L`, e basta comandar `w = v·κ`. O robô entra na
curva em vez de caçá-la. Três ajustes completam:

* **distância adaptativa**: `lookahead_min + lookahead_gain · v`, limitada por `lookahead_max`.
  Parado o ponto é próximo (preciso), andando ele se afasta (suave);
* **rampa**: `max_linear_accel` e `max_angular_accel` limitam a variação por ciclo, porque degrau de
  velocidade vira solavanco e escorrega a roda. A rampa é zerada em toda parada, pausa ou abortagem;
* **histerese no giro parado**: entra acima de `turn_in_place_angle` (0,7 rad), só sai abaixo de
  `turn_resume_angle` (0,25 rad). Sem isso o robô alterna entre girar e andar bem na fronteira.

Em simulação, seguindo o mesmo caminho em escada, o desvio padrão do comando angular cai de 0,107
para 0,054 rad/s só pelo controle, e para **0,012 rad/s** com o caminho suavizado — as trocas de
sinal do comando caem de 102 para 10. O caminho é a causa dominante; o controle, a segunda.

O estado publicado em `~/status` traz `cmd` (v e w efetivamente comandados), `lookahead_m` e
`turning_in_place`, que é o que se olha quando o movimento não parecer suave.

**Medido no robô em 2026-09-11**, dois marcadores seguidos, 100 s de corrida, 850 comandos com o
robô andando: desvio padrão do comando angular **0,064 rad/s** e **11 trocas de sinal em 100 s**
(0,11/s), velocidade média 0,19 m/s com teto de 0,30. Comandos típicos em reta: `[0.28, 0.02]`.

**Nota de operação**: nessa corrida o robô passou 25 s a 0,07-0,09 m/s com um obstáculo a 0,87 m no
cone frontal. Não é travamento, é o freio de proximidade: entre `obstacle_stop_distance` (0,6 m) e
`obstacle_slow_distance` (1,5 m) a velocidade cai proporcionalmente, e a 0,87 m sobra 30% dela. Se
isso for conservador demais para o ambiente, o parâmetro a mexer é `obstacle_slow_distance`, não o
de parada.
