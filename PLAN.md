# Husky A300 — histórico e próximos passos

Estado atual (2026-09-10): **navegação por waypoints GNSS RTK validada em campo.** Ver `README.md`
para arquitetura e operação, `agrobot_husky_nav/README.md` para o pacote.

## Histórico

### 2026-09-08 — levantamento e stack
- Levantamento do robô (Jazzy, Clearpath 2.9, namespace `a300_00096`, discovery server, `robot.yaml` da SENAI).
- Fixposition estava fora da rede; passou pelo Wi-Fi (`10.0.0.199`) e no fim do dia foi para o cabo (`192.168.131.35`).
- Decisão: **stack leve, sem Nav2**. Criado `agrobot_husky_nav` (seguidor + guarda de obstáculos + joystick).
- `pointcloud_to_laserscan` instalado; MID360 caracterizado pela nuvem: invertido, girado 90°, 0,61 m do chão.
- Correções RTK via ROS 2: `ntrip_client` integrado ao `colcon_ws`, `ntrip.launch.py`, fonte do sensor → I/O,
  provado com parada/religada do cliente. Arquitetura B escolhida (sensor no cabo, robô com Wi-Fi).
- Fusão do sensor encontrada parada (autostart habilitado; start falha). Foxglove: mesh `.STL` → `.stl`.

### 2026-09-09 — campo
- Robô fixado na Agriwing 2,4 GHz (5G removida do netplan).
- Sensor sem IP no cabo → usuário fixou `192.168.131.35/24` na interface do sensor.
- Roteador de campo sem internet → **túnel SSH reverso** do laptop (`tools/ntrip_tunnel.py` + `~/ntrip_tunnel_up.sh`).
  RTK fixed nas duas antenas (base RBMC a 486 m).
- Fusão descartada por ora (câmera sem calibração) → **`pose_source: dual_gnss`** implementado e validado
  (erro de rumo < 1° contra deslocamento RTK de 3 m).
- Testes: 5 m reto a 0,3 m/s (18 s, 0,58 m); quadrado 5 m a 0,5 m/s (68 s); a 1,0 m/s (48 s, guarda com
  reduz 4 m / para 1,8 m). Chegada sempre ~0,57 m. Guarda de obstáculos bloqueou e retomou nas duas corridas.
- Scripts de campo criados: `field_up.sh`, `nav_kill.sh`, `make_forward_mission.py`, `make_relative_mission.py`, `heading_check.py`.

### 2026-09-10 — consolidação
- Auditoria robô × diretório: pacote e scripts idênticos. `deploy.py` passou a sincronizar também os scripts do robô.
- `inventory/` atualizado (robot.yaml, ins_0.yaml, netplan, tópicos), `missions/` e `robot-home/` adicionados.
- **Web UI de teste** (`webui/index.html`, servida em http://10.0.0.60:8088): posição/estado ao vivo, waypoints por
  clique ou atalhos, envio da missão (`set_mission`) e Start/Pause/Stop via Foxglove bridge (`foxglove.sdk.v1`).

### 2026-09-09 (tarde) — percurso gravado com o joystick
- Causa raiz do Fixposition sumir do cabo: a conexão `fp-navvr2-eth0-static-ip` do sensor estava com
  **autoconnect desligado**. Ligado pela API (`net/conn_set` + `net/conn_up`); após o reboot seguinte
  ele subiu sozinho e o driver conectou sem intervenção.
- `record_waypoints.py`: grava waypoints enquanto se dirige. Qualquer botão de face marca (o mapeamento
  do controle varia: neste PS4 a bolinha é o índice 1, não o 2), Share desfaz, e todo botão apertado é
  registrado no log com o índice. O YAML é reescrito a cada ponto.
- 5 pontos gravados com RTK fixed (`missions/mission_gravada_5pts_2026-09-09.yaml`): circuito de 27 m
  com giros de 46° a 112°, dentro de uma caixa de 10 x 12 m.
- Missão executada com o padrão **partida → 1 → 2 → 3 → 4 → 5 → partida** (`mission_partida_5pts_retorno_2026-09-09.yaml`),
  com o ponto de partida capturado na posição do robô no início: 92 m em 222 s a 0,5 m/s, todos os
  7 waypoints alcançados entre 0,58 e 0,59 m e **retorno a 0,53 m do ponto de partida**.
- Web UI: o servidor HTTP precisa ser iniciado sem o `pgrep` inline (o padrão casa com a própria linha
  de comando por SSH e o servidor nunca sobe). Dentro do `field_up.sh` isso não acontece.

### 2026-09-09 (noite) — telemetria no sistema Agrobot
- Levantado como o drone se conecta (`agriwing@jetson.local`): o nó `drone_real/offboard_aruco_integration.cpp`
  publica os 9 tópicos do contrato `Agrobot_v2` como tópicos ROS globais (`/agent_telemetry`, `/agent_keep_alive`,
  `/agent_details`, `/activity_status`, ...), com períodos de 1 s / 10 s / 20 s. Um conversor `rabbitros` fora do
  Jetson leva esses tópicos ao **RabbitMQ em 10.0.0.96** (usuário `agrobot`). O drone é o agente `x650-jetson`.
- O Husky não pode repetir o mesmo caminho: está atrás do FastDDS Discovery Server e não aparece no grafo
  multicast. Criado o nó **`agrobot_bridge`**, que fala AMQP direto com o broker (pika em `/home/robot/pylibs`,
  instalado a partir de uma wheel copiada do laptop porque o robô não tem internet).
- Resultado: `agent_telemetry` a 1 Hz, `agent_keep_alive` a 0,1 Hz e `agent_details` a cada 20 s, todos aceitos
  (nenhuma mensagem em DLQ) e o orquestrador criou a fila `agent_details_husky`, confirmando o reconhecimento
  do agente. A ponte entrou no `field_up.sh` (`BRIDGE=0` pula) e no `nav_kill.sh`.

### 2026-09-11 — câmera estéreo ZED 2i (laboratório)
- ZED 2i instalada no Husky, reconhecida como câmera UVC (`/dev/video0`, YUYV lado a lado).
- Nó `zed_stereo` criado: corta o quadro ao meio e publica `zed/{left,right}/image_raw` (+ `/compressed`
  e `camera_info`), mais um serviço `zed/save` que grava o par em PNG. **Sem SDK, sem CUDA, sem IA.**
- Calibração de fábrica baixada por número de série (SN 33943780) e usada para preencher `K`, `D`, `R`, `P`
  via `cv2.stereoRectify` — assim o `stereo_image_proc` (`rectify:=true`) funciona sem calibração manual.
- Verificado em bancada: 29,5 fps em HD, zero falhas de leitura, par com paralaxe correta (16 px).
- "Artefato em X" relatado no Foxglove era a sobreposição de calibração do próprio visualizador: dado de
  origem íntegro (1154 quadros sem falha), resolvido desmarcando a calibração no painel.
- Pendente: medir a montagem da câmera para ligar o `publish_tf`.

### 2026-09-11 (tarde) — mapa local voxelizado do MID360 (indoor)
- Nó `voxel_local_map` + `indoor.launch.py`: acumula a nuvem compensando odometria, voxeliza e publica
  `voxel_cloud` e `local_map` (OccupancyGrid rolante de 20x20 m a 10 cm).
- Descoberto na bancada que 72% dos retornos eram do próprio robô (o lidar invertido enxerga o top plate
  e o arco) -> filtro por caixa do corpo, não por raio.
- Chão passou a ser classificado por plano ajustado por ciclo, porque o conjunto tem ~1,5° de inclinação.
- Montagem validada: plano de chão a -0,163 m (nominal -0,165), inclinação 1,1°, resíduo 4,7 cm.
  Nuvem comparada setor a setor com o `scan` 2D: diferença de 0 a 5 cm à frente.
- **Follow-me** (`follow_me` + `follow_me.launch.py`): alvo identificado por marcador ArUco (escolha do
  usuário), pose 3D por `solvePnP` com a calibração de fábrica, controle mantendo alvo centrado a 1,5 m.
  Lidar só como segurança, com o setor do alvo mascarado. Gerador do marcador em `tools/make_aruco.py`
  e PNG pronto em `calib/aruco_id0_15cm.png`.
- Armadilha encontrada: `cv_bridge` (OpenCV 4.6 do sistema) + OpenCV 4.10 do pip no mesmo processo =
  falha de segmentação. O `follow_me` converte Image para numpy sem `cv_bridge`.
- Detecção validada em bancada com marcador real de 17 cm (id 2, DICT_4X4_50) a 2,6 m: 231 detecções
  em 236 quadros; distância da câmera bateu com o lidar no mesmo rumo com 5 cm de diferença.
- **Teste de movimento do follow-me feito**: 2 min seguindo o operador, mantendo ~1,5 m (avanço até
  0,40 m/s, recuo a -0,18 m/s, giro até ±0,43 rad/s), com BLOCKED e LOST recuperando sozinhos.
- Ajuste que saiu do teste: a guarda de obstáculos bloqueava por causa do próprio operador (setor fixo
  de ±20° cobre só 36 cm a 1 m). Agora a máscara do alvo usa a largura da pessoa na distância medida e
  só vale para retornos na mesma distância.
- Pendente: medir a pose da ZED em base_link (a estimativa atual errou 5 cm contra o lidar).

### 2026-09-11 (noite) — mesh do robô no Foxglove
- Erro "20 links have error" no painel 3D: a lista de permissões de assets da ponte Foxglove bloqueava
  todas as malhas, porque o gerador da Clearpath dobra as barras invertidas ao serializar listas
  (`[-\w%]` vira `[-\\w%]`). O arquivo modelo da Clearpath está correto; quebra na geração.
- Corrigido de forma permanente em `robot.yaml` (`platform.extras.ros_parameters.foxglove_bridge`) com
  um padrão sem barras invertidas, imune à re-serialização. 15 de 15 malhas carregam após regenerar.
- Lição de método: URIs colhidas por SSH carregam `\r` e fazem qualquer teste falhar por engano.

### 2026-09-11 — banda: robô movido para a 5 GHz no laboratório
- Atraso na câmera e na nuvem era o **enlace de subida** da 2,4 GHz: 24,3 Mbit/s negociados, com
  895 KB/s já saturando. Perfil `Agriwing_5G` restaurado no netplan: subida foi a 1200,9 Mbit/s e o
  ping caiu de 85 ms para 3 ms. Os dois perfis convivem; em campo, remover o 5G força a 2,4 GHz.
- Custo medido por tópico: `image_raw` 78,6 MB/s (nunca pela rede), `/livox/lidar` 5,25 MB/s,
  `image_raw/compressed` 1,47 MB/s, `voxel_cloud` 513 KB/s, `local_map` 202 KB/s, `scan` 30 KB/s.
- Corrigido no `follow_me`: a pausa por serviço não grudava e o nó voltava a seguir sozinho.

### 2026-09-11 — mapeamento global 2D
- `slam_toolbox` instalado e integrado (`slam.launch.py` + `config/slam.yaml`), consumindo o `scan` do
  MID360 e publicando o frame `map`, a transformada `map→odom` e o OccupancyGrid global.
- Armadilha: o `slam_toolbox` é nó de **ciclo de vida** e sobe em `unconfigured`, sem assinar nada e
  sem reclamar. O launch agora emite `configure` e `activate` sozinho (padrão do launch oficial).
- `save_map.sh` grava `.pgm`/`.yaml` (para o map_server) e `.data`/`.posegraph` (para continuar o
  mapeamento ou rodar em modo localização). Testado.
- Como efeito colateral útil, agora existe o frame `map`, que o painel 3D do Foxglove pode usar.

### 2026-09-11 — exploração autônoma por fronteiras
- Nó `explore` + `explore.launch.py`: detecta fronteiras no mapa do SLAM (livre encostado em
  desconhecido), infla obstáculos pelo raio do robô, agrupa com `connectedComponents` do OpenCV e
  escolhe pelo maior `tamanho/(1+d²)`. Controle reativo com repulsão pelo scan; antitravamento por
  falta de avanço. Termina em `DONE` quando não há mais fronteiras.
- Joystick segue como camada de segurança, mesma lógica dos outros nós.

### 2026-09-11 — planejamento A* e navegação até marcadores
- A exploração reativa **ficou presa contra uma parede** no laboratório: repulsão local não contorna
  obstáculo côncavo. Substituída por planejamento A* sobre o próprio mapa do SLAM, com perseguição de
  ponto à frente. O scan ficou como última barreira, para o que o mapa ainda não viu.
- Segundo erro descoberto na mesma corrida: as fronteiras eram ordenadas por distância em linha reta.
  Uma a 9,3 m exigia 45,7 m de caminho (atrás de uma parede) enquanto uma a 22,8 m precisava de
  24,8 m. Agora o nó planeja para as melhores candidatas e escolhe pelo **caminho real**.
- Com isso a corrida seguinte progrediu sem travar: 8 destinos, área conhecida de 137,5 para 172,5 m².
- O planejamento saiu para `grid_planner.py`, compartilhado, para não existirem dois planejadores
  divergindo (raio e folga diferentes fariam um aceitar caminho que o outro proíbe).
- Nó novo `goto_point` + `goto_point.launch.py`: o operador clica no mapa pelo Foxglove e o robô vai
  até lá; cada clique entra numa fila, virando rota. Também aceita JSON por `~/set_goal` e a linha de
  comando `~/goto.py`. Validado: marcador a 15,75 m, caminho de 18,6 m em ~62 s, passagem por vão
  estreito sem travar, e emenda automática no marcador seguinte.
- Armadilha operacional corrigida: `nav_kill.sh` casava `"agrobot_husky_nav explore"`, padrão que
  **não existe** na linha de comando do executável instalado (`.../lib/agrobot_husky_nav/explore`).
  Três instâncias sobreviveram aos restarts e publicaram juntas em `cmd_vel`. Os padrões agora casam
  o caminho do executável, e o resumo do script conta os processos de `explore` e `goto_point`.
  Verificação de campo: `ros2 topic info /a300_00096/cmd_vel` tem que mostrar **um** publicador.

### 2026-09-11 — suavidade do movimento
- O robô chegava aos marcadores mas **serpenteava**. Duas causas independentes.
- Caminho: o A* numa grade de 10 cm com 8 vizinhos só anda em múltiplos de 45 graus, então o
  caminho era uma escada. Agora passa por encurtamento por visada livre, arredondamento de quina
  (Chaikin) e reamostragem uniforme, dentro do `grid_planner.py`. A visada só encurta se não
  aproximar da parede mais do que o A* já aceitara, para não comer a folga de segurança.
  Medido no mapa do laboratório, destino a 4 m: 107 graus/m de mudança de rumo no caminho cru
  contra 2,3 graus/m no suavizado, com o comprimento caindo de 4,2 para 4,0 m.
- Controle: `w = k · erro_de_rumo` zera só quando o robô aponta exato para o alvo, então ele passa e
  corrige em ciclo. Trocado por pure pursuit geométrico (`w = v·κ`, `κ = 2·sen(α)/L`) em
  `path_follow.py`, com distância de perseguição adaptativa à velocidade, rampa de aceleração e
  histerese no giro parado. Em simulação sobre o mesmo caminho em escada, o desvio padrão do
  comando angular cai de 0,107 para 0,012 rad/s e as trocas de sinal de 102 para 10.
- Os dois nós de movimento autônomo (`goto_point` e `explore`) usam o mesmo planejador e o mesmo
  seguidor, para não divergirem.
- Verificado no robô: 100 s de corrida, dois marcadores, desvio padrão do comando angular de
  0,064 rad/s e 11 trocas de sinal (0,11/s), a 0,19 m/s de média.
- Corrigido também: os prazos (progresso, `goal_timeout`, `blocked_timeout`) corriam durante a pausa.
  O operador segurou L1 por 37 s e, na retomada, o nó descartou o marcador por "sem progresso" sem o
  robô ter tido chance de andar. Agora o tempo parado por ordem de alguém é descontado dos prazos e a
  medida de progresso recomeça na retomada, nos dois nós (`goto_point` e `explore`).
- Ajuste seguinte: a reamostragem também no trecho reto de dois pontos. Sem ela o caminho reto saía
  com dois pontos, o ponto perseguido virava o próprio destino a metros de distância e o desvio
  lateral era corrigido devagar demais.

### 2026-09-15 — organização do `/home/robot`
- Os scripts de operação eram arquivos soltos em `/home/robot/`, copiados pelo deploy: 15 deles, sem
  nada que impedisse a cópia no robô de divergir do repositório. Conferido antes de mexer: os 15
  estavam idênticos, mas por sorte, não por construção.
- Agora moram em `agrobot_husky_nav/scripts/`, dentro do pacote. O `colcon build` os instala em
  `lib/agrobot_husky_nav`, o que os deixa a um `ros2 run` de distância, e o deploy cria em
  `/home/robot/` um link para cada um. `~/nav_kill.sh all` continua valendo, e a documentação toda
  segue correta, mas o arquivo agora é um link para o fonte instalado.
- Promovidos ao repositório os cinco scripts do Livox que já viviam no robô e só tinham cópia morta
  em `robot-home/` (diretório removido).
- `goto_up.sh`, improviso da sessão anterior, virou `indoor_up.sh`: sobe nuvem, scan, mapa local e
  SLAM, aceita `goto` ou `explore` como argumento e `MAPA=<nome>` para continuar de um mapa salvo.
  Ao contrário do `field_up.sh`, não exporta o perfil do FastDDS que bloqueia transient_local, que é
  justamente o que faria o planejador não receber o `map`.
- Arrumação do resto: missões soltas para `~/missions/`, logs para `~/logs/`, cópias antigas para
  `~/attic/2026-09-15/`. Credenciais NTRIP, calibração da ZED e o perfil do FastDDS ficaram onde
  estavam, porque não são scripts.

### 2026-09-15 — o robô sumiu do painel 3D do Foxglove
- Sintoma: a descrição do robô não aparecia mais. Servidor íntegro: `robot_state_publisher` no ar, a
  ponte ouvindo na 8765, o URDF publicado com 24 KB e a allowlist de assets correta.
- Diagnosticado com um cliente WebSocket mínimo que fala o protocolo do Foxglove: a ponte **anuncia**
  o canal, mas assinar não traz mensagem nenhuma.
- Causa: o URDF é publicado **uma única vez** num tópico retido, e a assinatura da ponte estava em
  `BEST_EFFORT/VOLATILE`. Assinante volátil não recebe mensagem retida. A ponte escolhe a QoS quando
  o primeiro cliente pede o tópico; se o publicador ainda não foi descoberto, cai no volátil e fica
  assim enquanto houver cliente assinando — e a assinatura ROS é uma só para todos os clientes.
- Solução: nó `urdf_beacon`, que guarda a descrição e a repete a cada 5 s em
  `robot_description_foxglove`, tópico separado de propósito, porque o `controller_manager` também
  assina `robot_description` e reage a cada mensagem nova. Sobe junto no `field_up.sh` e no
  `indoor_up.sh`.
- Verificado pela própria ponte: no tópico repetido chegam 40 links e a primeira mesh baixa com
  3,1 MB; no tópico original, nada.

## Próximos passos
1. **Modo híbrido indoor/outdoor** (ver o README): georreferenciar o mapa do SLAM com a lat/lon e o
   rumo da origem, unificar a árvore de TF em `earth → map → odom → base_link` trocando apenas quem
   publica `map→odom`, criar um supervisor dono do `cmd_vel`, definir o critério de transição com
   histerese e um formato de missão em que cada waypoint declara o seu frame.
2. **Botão Options do joystick**: o índice 9 não inicia (o controle expõe 15 botões; Círculo=1, L1=4,
   R1=5 confirmados). Levantar o índice correto com `ros2 topic echo /a300_00096/joy_teleop/joy` e
   corrigir `joy_start_button` em `explore.yaml` e `goto_point.yaml`.
3. **Serviço systemd** (ou `platform.extras.launch`) para Livox + `bringup.launch.py` subirem com o robô, sem missão.
4. **Internet em campo sem laptop**: modem 4G no roteador ou no robô; então `ntrip.launch.py` direto no caster.
5. **Calibrar a câmera do Fixposition** e voltar a `pose_source: fusion` (rumo e pose mesmo sem RTK fixed;
   tolera perda de correção). Precisa da fusão iniciando na API (`ctrl/action {"fusion":"start"}`).
6. **Trajeto de fileiras** (ida e volta paralelas de 20 m) e teste do `obstacle_steer` (desvio lateral).
7. **Rosbag** por corrida (`gps_*/fix`, `gnsscorr`, `scan`, `cmd_vel`, `joy`, `status`) para análise.
8. Consumir `activity_dispatch`/`activity_abort` na ponte e publicar `activity_status` (missão vinda do sistema).
9. Integração com o PLAAC (`agrobot-physical-layer`): expor `load_mission`/`start`/`stop`, e agora também
   os marcadores do `goto_point`, como skills de missão (detalhes em `proximos_passos_plaac.md`).
10. Curvas suaves com raio mínimo em vez de giro parado, se a aplicação pedir. O seguidor já comanda por
    curvatura, então falta só limitar `κ` e tratar o caso em que o destino fica atrás do robô.
11. Cabo do laptop: adaptador USB-ethernet sem link em 2026-09-08 (trocar); laptop precisa de IP estático
    em `192.168.131.x`.
