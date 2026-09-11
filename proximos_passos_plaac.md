# Próximos passos — integrar o Husky A300 ao PLAAC e às camadas Agrobot

Documento de planejamento escrito em 2026-09-09, a partir de uma leitura completa dos quatro
repositórios (`services/`, `comms/`, `agrobot-physical-layer/plaac`, `agrobot-simulation`) cruzada
com o que já está **validado em campo** no robô real (ver `README.md` e `PLAN.md` deste diretório).

Nada aqui foi implementado ainda. A navegação por waypoints GNSS do Husky funciona de forma
autônoma (`agrobot_husky_nav`), e é essa capacidade que queremos expor como **skill** de um agente
PLAAC comandado pela camada de serviços.

---

## 1. Objetivo

Fechar o caminho completo:

```
UI/serviços (missão) → camada de comunicação → agent-manager (PLAAC) no robô → agrobot_husky_nav → Husky anda
                     ←        progresso, telemetria e resultado                                  ←
```

com um teste de ponta a ponta usando o agente real do Husky.

---

## 2. Estado de cada camada (2026-09-09)

### 2.1 Camada de serviços — `services/agrobot-manager` + `services/agrobot-orchestrator`
- Missão criada na UI (Laravel/Vue) → `POST /api/mission/start` no orquestrador (Node/Fastify).
- O orquestrador **gera os waypoints** a partir de polígonos PostGIS por cobertura boustrophedon
  (`services/Boustro.ts`, `models/activities/*.ts`) — não existe endpoint "suba esta lista de waypoints".
- Despacho por **RabbitMQ**, fila `activity_dispatch` (`models/activities/Activity.ts:120-145`):
  `{agentId, protocol:"ROS2", version, correlationId, timestamp, seq, payload:{missionId, activityId, taskType, waypoints:[{lat,lng,altitude}]}}`
- Filas de retorno consumidas: `activity_status`, `agent_telemetry`, `agent_keep_alive`, `agent_details`, `agent_emergency`.
- Filas declaradas **sem consumidor** (a caixa de entrada do robô): `activity_dispatch`, `activity_abort`, `agent_config`, `agent_alert`.
- Registro de agente é **manual na UI** (`agents.supported_activities` jsonb = as "skills"); `agent_details`
  e `agent_keep_alive` não persistem no banco.
- Único "robô" que consome `activity_dispatch` hoje: `.scripts/test-simulation.ts` (robô falso).
- Stack em `agrobot-docker/docker-compose.yml` (Postgres+PostGIS, RabbitMQ, Mosquitto ocioso, manager, orchestrator).
  Submódulos `services/manager` e `services/orchestrator` estão **vazios** neste clone.

### 2.2 Camada de comunicação — `comms/agrobot-json-message-lib` + `comms/agrobot-communication`
- Contrato vivo é o **`Agrobot_v2`**: envelope `{source, destination, version, correlationId?, timestamp, seq, payload}`
  (**sem campo `type`**), 9 tipos de mensagem, `activity_dispatch` é o único com waypoints lat/lng.
- Ponte **RabbitMQ ↔ ROS 2** pronta: `conversors/rabbitros` transforma cada fila em tópico
  `std_msgs/String` (`/activity_dispatch`, `/activity_status`, ...). Consome com `passive=True`,
  então as filas precisam existir antes.
- Dois defeitos conhecidos: o gateway publica protobuf e a ponte faz `decode('utf-8')`
  (`rabbitros_adapter.py:74-80` vs `rabbitros.py:133`); e o broker padrão está fixo em `192.168.0.103`
  (`rabbitros.py:17`).
- Existem **três cópias divergentes** de schemas (biblioteca v2, `gateway/schemas` v1 pontuado,
  `communication-container/schemas` v1). Não há `mission.ack`, `mission.progress` nem `mission.result` no v2.

### 2.3 PLAAC — `agrobot-physical-layer/plaac` (branch `main` é a mais completa)
- Transporte: **só tópicos ROS 2 com JSON** (`std_msgs/String`), prefixo `/agrobot/<agent_id>/...`.
  Não fala MQTT/AMQP — por decisão (D-09, a ponte é papel da camada de comunicação).
- Ciclo implementado: `mission.upload` → validação por JSON Schema → `mission.ack` →
  `mission.progress` (VALIDATED/PLANNED/RUNNING/PAUSED) → `mission.result` (COMPLETED/FAILED/CANCELLED/INTERRUPTED),
  mais `agent.announce` (catálogo de skills), `agent.heartbeat` e `robot.telemetry.state`.
- **Waypoints só em métrico** `{seq, x, y, z, yaw}` (`mission.upload-1.0.json:85-118`). Não há lat/lon,
  embora `device.frame` já aceite `WGS84` e exista um schema `robot.telemetry.gnss` (nunca publicado).
- Executores: `MissionExecutor` (simulado) e `Nav2MissionExecutor` (`nav2_msgs/NavigateToPose`).
  Seleção **fixa** por `use_nav2` (`agent_manager_node.py:273-290`). `pause`/`resume` do Nav2 são no-ops.
- Skills: `HuskyA300_UGV_Agent.SKILL_DESCRIPTORS` já declara **`follow_waypoints`** (NAVIGATION, com
  `robot_speed`, `tolerance_m`, precondições `gps_fix`/`nav_stack_up`) — exatamente a nossa capacidade real.
  O `MissionValidator` exige esse nome quando a missão tem waypoints.
- Telemetria é stub (`AgentHardwareInterface.get_state()` devolve `{"status": "IDLE"}` fixo).
- Branch `origin/4-desenvolver-hardware-interface-plaac`: tem `husky_hardware_interface.py` (odom, bateria,
  `cmd_vel`) mas com namespace errado (`a300_0000` ≠ `a300_00096`) e **não está ligada** a nenhum agente.
- Milestones: M1/M2 no código mas sem validação ponta a ponta; M3 parcial; M4 não iniciado.

### 2.4 Robô real — `husky-config/agrobot_husky_nav` (validado em campo)
- `gps_waypoint_follower`: entrada `set_mission` (String YAML/JSON) e `load_mission` (caminho),
  serviços `start`/`pause`/`stop` (`std_srvs/Trigger`), saída `status` (String JSON a 2 Hz com
  estado, lat/lon, rumo, waypoint atual, distância, saúde, fix RTK) e `cmd_vel`.
- Missões em **lat/lon** com tolerância por ponto. Guarda de obstáculos pelo MID360 e bypass por joystick.
- Testes de campo: 5 m reto; quadrado de 5 m a 0,5 e 1,0 m/s; percurso gravado de 5 pontos (2026-09-09).

---

## 3. Onde as pontas não se encontram

| # | Lacuna | Impacto |
|---|---|---|
| 1 | Serviços falam **v2 por RabbitMQ**; PLAAC fala **v1 por tópicos ROS**. Ninguém traduz | Nada chega ao robô |
| 2 | Envelope do orquestrador (`agentId`) ≠ envelope do schema v2 (`source`/`destination`); a validação só loga | Um agente que validar de verdade rejeita o despacho |
| 3 | `mission.upload` do PLAAC **não aceita lat/lon**; serviços só mandam lat/lon | Missão GPS é rejeitada com `SCHEMA_INVALID` |
| 4 | PLAAC não tem executor para o nosso seguidor (só simulado e Nav2), nem `robot_namespace` configurável | Não há como acionar `agrobot_husky_nav` |
| 5 | v2 não tem progresso por waypoint (só `activity_status`); o `mission.progress` do PLAAC não tem consumidor | Perde-se a granularidade que o robô já produz |
| 6 | `activity_abort` é declarada mas nunca enviada; PLAAC tem `mission.cancel` | Sem parada remota |
| 7 | Telemetria real (posição, bateria) não existe nem no PLAAC nem no adaptador | UI não mostra o robô no mapa |
| 8 | Registro/skills: `agent_details` não persiste; `agents.id` precisa casar com o `agentId` na mão | Alocação de agente frágil |
| 9 | Ponte `rabbitros`: bug protobuf/utf-8 e broker fixo | Mensagens vão para a DLQ |

---

## 4. Arquitetura proposta

Manter o PLAAC como agent-manager com o contrato v1 dele, e acrescentar no robô um **adaptador de
comunicação** fino entre o RabbitMQ da camada de serviços e os tópicos do PLAAC. **Nada muda nos serviços.**

```
RabbitMQ (laptop/servidor)          Adaptador (no robô)              PLAAC                      agrobot_husky_nav
──────────────────────────          ───────────────────              ─────                      ─────────────────
activity_dispatch  (lat/lng) ─────► traduz p/ mission.upload  ─────► valida, ack, executor ────► set_mission + start
activity_abort              ─────► mission.cancel             ─────► executor.cancel()    ────► stop
activity_status             ◄───── ack / progress / result    ◄───── mission.*            ◄──── status (JSON)
agent_telemetry             ◄───── robot.telemetry.state      ◄───── telemetria           ◄──── status + BMS
agent_keep_alive            ◄───── agent.heartbeat            ◄─────
agent_details               ◄───── agent.announce (skills)    ◄─────
```

Por que assim: preserva o ack/progresso que o PLAAC já implementa (mais rico que o v2), não exige
mexer em repositórios de outras pessoas, e o adaptador é substituível quando o contrato convergir.

### Mapeamento de mensagens

| v2 (serviços) | v1 (PLAAC) | Observações |
|---|---|---|
| `activity_dispatch.payload.waypoints[{lat,lng}]` | `mission.upload.payload.waypoints[{seq,lat,lon}]` com `device.frame: WGS84` | exige extensão do schema (§5.1) |
| `taskType` MAPPING/MONITORING/WEEDING | `task_type` MAPPING/NAVIGATION/WEEDING | MONITORING → NAVIGATION por ora |
| `missionId` + `activityId` | `mission_id` (string `"<missionId>:<activityId>"`) | o adaptador guarda o par para a volta |
| `agentId` | `device.id` = `agent_id` do PLAAC | decidir a identidade (§7) |
| `activity_status{status}` | `mission.ack`/`progress`/`result` | ACCEPTED→IN_PROGRESS, COMPLETED→COMPLETED, FAILED/CANCELLED→FAILED |
| `agent_telemetry{position,battery,speed,heading}` | `robot.telemetry.state` + `gnss` | posição vem do `status` do seguidor |

---

## 5. Mudanças necessárias (todas na branch `main` do PLAAC)

### 5.1 Schema: waypoints geográficos
`mission.upload-1.1.json`: `waypoints.items` vira `oneOf` — o item métrico atual `{seq,x,y,z,yaw}`
**ou** o geográfico `{seq, lat, lon, alt?, yaw?, tolerance_m?}`, válido quando `device.frame == "WGS84"`.
Manter o 1.0 aceito, para não quebrar os testes existentes.

### 5.2 Executor novo: `WaypointFollowerMissionExecutor`
Mesmo contrato pato dos outros (`mission_id`, `started_at`, `start/cancel/pause/resume/join`,
`on_progress`, `on_result`), mas conversando com o `agrobot_husky_nav`:
- publica a missão em `<robot_ns>/gps_waypoint_follower/set_mission` (YAML/JSON inline);
- chama `start` / `pause` / `stop` (`std_srvs/Trigger`) — **precisa de cliente de serviço**, que o PLAAC
  ainda não usa em lugar nenhum (`create_client` = 0 ocorrências) e exige `std_srvs` no `package.xml`;
- assina `<robot_ns>/gps_waypoint_follower/status` e traduz:
  `RUNNING→RUNNING`, `PAUSED|BLOCKED→PAUSED` (+ evento com o motivo do campo `health`),
  `DONE→COMPLETED`, `ABORTED→FAILED`, `wp_index/total→progress_pct` e `current_step{kind: WAYPOINT}`.
- Atenção ao **deadlock**: a lógica do executor roda em thread própria enquanto o nó usa
  `rclpy.spin` de thread única (`agent_manager_node.py:367`). Usar `MultiThreadedExecutor` +
  `ReentrantCallbackGroup`, ou chamar os serviços de forma assíncrona.

### 5.3 Configuração
- `executor_type: simulated | nav2 | waypoint_follower` (ou import dinâmico como o `agent_class`),
  no lugar do booleano `use_nav2`.
- `robot_namespace: a300_00096` para resolver tópicos e serviços do seguidor.
- Perfil `plaac_bringup/config/husky_a300_real.yaml`: `agent_id`, `protocol.mode: Real`,
  `device.frame: WGS84`, `use_sim_time: false`.

### 5.4 Telemetria real
Interface de hardware que lê o `status` do seguidor (lat/lon, rumo, velocidade) e
`platform/bms/state` (bateria) e alimenta `robot.telemetry.state` + `robot.telemetry.gnss`.
Aproveitar `husky_hardware_interface.py` da branch `4-...`, **corrigindo o namespace** e ligando-a ao agente.

### 5.5 Skills
Ajustar a descrição de `follow_waypoints` (hoje diz "usando Nav2") e as precondições para o que o
seguidor de fato exige: `rtk_fix` (gnss1/gnss2 ≥ 7), `scan_ok`, `joystick_present`, `estop_released`.
Avaliar publicar `available: false` quando a saúde do seguidor reprovar — é a base para o
"skill discovery" automático que está na lista de lacunas do próprio PLAAC.

### 5.6 Adaptador `plaac_comms_bridge` (pacote novo)
- Consome `activity_dispatch` e `activity_abort` do RabbitMQ; publica `activity_status`,
  `agent_telemetry`, `agent_keep_alive`, `agent_details`.
- Aceita os **dois** formatos de envelope v2 (`agentId` e `source`/`destination`).
- Guarda o par `missionId`/`activityId` e o `correlationId` para a resposta.
- Pode reusar `conversors/rabbitros` em vez de falar AMQP direto — decidir em §7.

---

## 6. Plano por etapas (testável sem o robô)

O laptop tem ROS 2 Jazzy e Docker, então dá para validar quase tudo antes de ir a campo.

1. **Seguidor simulado** (`fake_waypoint_follower.py`): mesma interface do real (`set_mission`,
   `start/pause/stop`, `status`), integrando a posição a partir de um lat/lon inicial. Fica em
   `husky-config/tools/` ou no pacote de testes do PLAAC.
2. **Schema 1.1 + executor novo + config**: testes unitários do PLAAC (rodam sem ROS) e um teste de
   integração com o seguidor simulado.
3. **Adaptador**: sobe o RabbitMQ do `agrobot-docker`, declara as 9 filas, publica um `activity_dispatch`
   à mão (o exemplo `examples/activity_dispatch.json` serve) e confirma `activity_status` de volta.
4. **Ponta a ponta simulado**: serviços (ou script) → RabbitMQ → adaptador → PLAAC → seguidor simulado.
5. **Ponta a ponta real**: trocar o seguidor simulado pelo `agrobot_husky_nav` no Husky. Primeira
   missão: 2 waypoints a 5 m, 0,3 m/s, operador com o joystick (L1 pausa, Círculo aborta).
6. **Ajustes de campo**: latência do RabbitMQ pela Wi-Fi, reconexão, comportamento quando o RTK cai
   (o seguidor pausa sozinho — o PLAAC precisa reportar isso como `PAUSED` com motivo, não como falha).

---

## 7. Decisões pendentes (precisam de acordo com o time)

1. **Contrato**: adaptador no robô preservando o v1 do PLAAC (proposta acima) **ou** reescrever o
   PLAAC para falar v2 direto no RabbitMQ? A primeira é menos invasiva e preserva ack/progresso;
   a segunda elimina uma camada e uma tradução.
2. **Onde mora o código**: executor e schema no repositório do PLAAC (natural); o adaptador junto do
   PLAAC ou em `comms/agrobot-communication`? Sugestão: começar no PLAAC e mover depois de estabilizar.
3. **Identidade do agente**: hoje o `agent_id` do PLAAC é ao mesmo tempo prefixo de tópico e
   `device.id` obrigatório. Usar `a300_00096` (namespace ROS do robô) ou o `agents.id` do banco dos
   serviços? Precisamos de um mapa explícito entre os dois.
4. **Waypoints**: os serviços só geram waypoints por cobertura de área. Para os nossos testes é
   preciso um caminho de "lista de waypoints avulsa" (endpoint novo, ou o adaptador aceitando um
   arquivo local como o que gravamos com `record_waypoints.py`).
5. **Broker em campo**: RabbitMQ roda no laptop (rede Agriwing, sem WAN). Definir endereço fixo,
   credenciais e o que acontece quando o link cai no meio de uma missão.

---

## 8. Referências rápidas

| Assunto | Onde |
|---|---|
| Contrato v2 e exemplos | `comms/agrobot-json-message-lib/SCHEMAS.md`, `examples/activity_dispatch.json` |
| Ponte RabbitMQ↔ROS | `comms/agrobot-communication/conversors/rabbitros/` |
| Despacho e filas | `services/agrobot-orchestrator/src/models/activities/Activity.ts`, `src/handlers/rabbitmq/index.ts` |
| Robô falso de referência | `services/agrobot-orchestrator/.scripts/test-simulation.ts` |
| Nó do agent-manager | `agrobot-physical-layer/plaac/plaac_manager_py/plaac_manager_py/agent_manager_node.py` |
| Schemas do PLAAC | `.../plaac_manager_py/schemas/` (13 arquivos Draft-07) |
| Skills do Husky | `.../agent_examples/husky_a300/husky_a300_ugv_agent.py` (`SKILL_DESCRIPTORS`) |
| Executor Nav2 (modelo) | `.../plaac_manager_py/nav2_mission_executor.py` |
| Interface do nosso seguidor | `husky-config/agrobot_husky_nav/README.md` |
