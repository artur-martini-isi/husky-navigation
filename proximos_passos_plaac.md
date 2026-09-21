# Integrar o Husky A300 ao PLAAC e às camadas Agrobot

**Revisão de 2026-09-21.** A versão anterior deste documento era de 2026-09-09 e ficou obsoleta:
entre uma data e outra a camada de comunicação ganhou um **gateway** e o orquestrador migrou o
envelope. Três das nove lacunas que listávamos foram resolvidas por outras pessoas, e a arquitetura
que propúnhamos (um adaptador AMQP no robô) **deixou de fazer sentido** — o gateway ocupa esse lugar.

---

## 1. Objetivo

```
UI/serviços → orquestrador → RabbitMQ → gateway (workstation) → UDP → gateway (husky)
    → tópicos ROS 2 → [ PLAAC ] → agrobot_husky_nav → Husky anda
    ←            activity_status, agent_telemetry, keep_alive, details            ←
```

---

## 2. O que mudou desde 2026-09-09

| Antes | Agora |
|---|---|
| Orquestrador mandava `agentId`/`protocol` | Manda `source`/`destination`, o contrato publicado |
| `activity_abort` declarada e nunca enviada | `Activity.abort()` publica de verdade |
| Identidade do agente combinada na mão | Coluna `destination_id` na tabela de agentes, preenchida na UI |
| Ponte `rabbitros` com bug de protobuf | Substituída pelo gateway `protocol_hub` |
| Três cópias divergentes de schemas | Biblioteca v2 única, usada como submódulo pelo gateway |

**Continua igual:** o PLAAC não fala nada disso. Ele tem dialeto próprio, com tópicos
`/agrobot/<agent_id>/...`, mensagens `mission.upload`/`ack`/`progress`/`result` e waypoints
**métricos**. Não há uma linha sequer sobre PLAAC dentro do gateway.

---

## 3. O gateway, que é a peça nova

Serviço Python em contêiner, imagem sobre `ros:jazzy-ros-base`. O **mesmo binário** roda nas duas
pontas, escolhido por papel: `service` na workstation, com borda RabbitMQ, e `agent` no robô, com
borda ROS 2. Entre as duas pontas ele serializa em protobuf e envia por UDP.

No robô ele expõe **nove tópicos planos**, todos `std_msgs/String` com o envelope JSON dentro:

| Direção | Tópicos |
|---|---|
| Gateway **publica** (serviços → robô) | `/activity_dispatch`, `/activity_abort`, `/agent_config`, `/agent_alert` |
| Gateway **assina** (robô → serviços) | `/agent_telemetry`, `/activity_status`, `/agent_details`, `/agent_keep_alive`, `/agent_emergency` |

Os nomes dos tópicos são **absolutos**, sem namespace. Roteamento por `destination`: o gateway do
robô ignora silenciosamente tudo que não for endereçado a ele.

### A identidade é a palavra `husky`

Minúscula, sem sufixo, e precisa bater em quatro lugares: variável do contêiner, mapa de pares da
topologia, campo do envelope e coluna `destination_id` no banco dos serviços. Nossa ponte já usa
`husky`, então esse ponto já está certo.

---

## 4. Três armadilhas verificadas no código

Não são suposições: conferi cada uma nos arquivos.

### 4.1 O envelope rejeita campos extras

Todos os nove esquemas têm `additionalProperties: false` **no topo e dentro do payload**. O envelope
aceita exatamente sete campos: `source`, `destination`, `version`, `correlationId`, `timestamp`,
`seq`, `payload`.

Nossa ponte manda `agentId` e `protocol` além desses, de propósito, para funcionar com as duas
leituras do envelope. **O gateway descarta essas mensagens**, e descarta no próprio robô, porque
valida na entrada e de novo depois do enlace. Quando migrarmos, esses dois campos têm que sair.

### 4.2 A QoS do gateway não casa com um assinante comum

O gateway publica e assina tudo com `BEST_EFFORT` + `VOLATILE`. O padrão de um assinante ROS 2 é
`RELIABLE`, e `RELIABLE` **não casa** com um publicador `BEST_EFFORT`. Ou seja, um
`create_subscription(String, '/activity_dispatch', cb, 10)` escrito do jeito natural **não recebe
nada, sem erro nenhum**. É a falha silenciosa mais provável de toda essa integração.

Consequência de projeto: o despacho de missão é sem garantia de entrega. Uma missão publicada
enquanto nosso nó reinicia se perde para sempre. O reconhecimento tem que ser da aplicação, por
`activity_status`.

### 4.3 O gateway não enxerga os tópicos do nosso robô

O perfil do Husky fixa `ROS_DOMAIN_ID=22`. Nosso A300 roda no domínio 0 e **atrás de um FastDDS
Discovery Server**, que é justamente por isso que a ponte atual existe. O gateway não define
`ROS_DISCOVERY_SERVER` nem `ROS_SUPER_CLIENT` em lugar nenhum, conferido por busca no repositório
inteiro.

**Este é o bloqueio de verdade.** Sem resolver, o contêiner sobe, roda e não vê tópico nenhum. O
único sinal é o contador de assinantes zerado no log dele.

Dois caminhos: acrescentar as variáveis de descoberta ao contêiner, ou rodar um nó pequeno dentro do
ambiente ROS do robô que faça a ponte entre o grafo do robô e o domínio 22.

---

## 5. O estado real do PLAAC

O `main` é a única versão funcional: 85 testes passam. Mas o que interessa para nós está incompleto.

**Falta, e é bloqueante para um robô real:**

1. **Não há como injetar uma interface de hardware.** O bootstrap chama a classe do agente sem
   argumentos e não existe chave de configuração para a interface. Toda implantação hoje roda o
   stub, cujo estado é a constante `IDLE`. A telemetria do PLAAC não tem como carregar dado real.
2. **Waypoints só em métrico.** O esquema exige `{seq, x, y, z, yaw}`. Uma missão em lat/lon é
   recusada com `SCHEMA_INVALID`, e lat/lon é o que os serviços mandam.
3. **A escolha de executor é um booleano** `use_nav2`. Acrescentar um terceiro destino, como o nosso
   seguidor, exige editar o nó.
4. **Giro de thread única.** Qualquer cliente de serviço chamado de dentro de um executor trava. O
   nosso seguidor é comandado por serviços `Trigger`, então isso nos atinge direto.
5. **Sem reconhecimento para cancelar, pausar e retomar**, embora o esquema preveja.
6. **Pausa e retomada do executor Nav2 são no-ops** que só escrevem no log.

**Ramos abertos, não integrados:**

| Ramo | Data | O que tem | Serve? |
|---|---|---|---|
| `10-desenvolver-o-message-handler` | 17/09 | Migração para a biblioteca v2 | **Não**, está com 298 de 373 linhas do nó comentadas |
| `8-adicionar-submódulo-protocol-json-msg-lib` | 17/09 | Só o submódulo e o invólucro | Parcialmente |
| `4-desenvolver-hardware-interface-plaac` | 28/08 | 486 linhas de interface do Husky | **Em parte**, ver abaixo |

A interface do ramo 4 lê odometria e bateria e preenche o estado, o que resolveria o item 1. Mas ela
é de teleoperação, com métodos de andar para frente e girar, e usa namespace `a300_0000`, tópico de
bateria e de odometria que não são os do nosso robô. Aproveitável com correções.

O ramo 10 merece atenção: alguém começou a migrar o PLAAC para o protocolo v2. Se isso for em frente,
a tradução entre dialetos encolhe muito. Vale alinhar antes de escrever código que o trabalho dessa
pessoa tornaria desnecessário.

---

## 6. Duas arquiteturas possíveis

### A — Robô fala direto com o gateway, sem PLAAC

`agrobot_husky_nav` ganha um nó que assina os quatro tópicos de comando e publica os cinco de
relatório. Traduz `activity_dispatch` em missão do seguidor e o estado do seguidor em
`activity_status`.

Prós: caminho mais curto para um teste de ponta a ponta, e os construtores de payload que já temos
na ponte são reaproveitados quase inteiros. Contra: o PLAAC fica de fora, e ele é o objetivo.

### B — PLAAC no meio

O mesmo nó acima, mas em vez de falar com o seguidor, fala com o PLAAC no dialeto dele, e o PLAAC
ganha um executor que comanda o seguidor.

Prós: é o objetivo declarado, e preserva o reconhecimento e o progresso por waypoint que o PLAAC já
sabe produzir, mais ricos que o `activity_status`. Contra: exige resolver os seis itens da seção 5.

**Recomendação:** fazer A primeiro, como andaime. Ele destrava o teste de ponta a ponta, prova o
enlace do gateway e a descoberta DDS, e o trabalho não se perde: o tradutor de mensagens é o mesmo
nos dois desenhos, muda só com quem ele conversa do lado de dentro.

---

## 7. Ordem de ataque

1. **Descoberta DDS** (seção 4.3). Sem isso nada mais importa. Testável no laboratório, hoje.
2. **Enlace do gateway ponta a ponta**, com os scripts de demonstração que ele traz, antes de
   escrever qualquer código nosso.
3. **Nó tradutor no robô**, com a QoS certa. Primeiro só os relatórios, que é o que já sabemos
   montar, depois o consumo de comandos.
4. **Aposentar a ponte AMQP atual**, sem rodar as duas ao mesmo tempo para comandos.
5. **PLAAC**: injeção da interface de hardware, esquema de waypoint geográfico, executor do
   seguidor, giro multi-thread.
6. **Ponta a ponta com PLAAC no meio.**

---

## 8. Decisões pendentes

- **O ramo 10 vai em frente?** Se o PLAAC migrar para o v2, a tradução de dialeto some quase toda.
- **Onde mora o tradutor?** Pacote novo no `husky-config`, ou dentro do PLAAC. Afeta quem mantém.
- **A descoberta DDS se resolve no contêiner ou com um nó relé?** A primeira é mais limpa, a segunda
  não depende de mexer no repositório de outra pessoa.
- **O gateway roda no próprio Husky ou num computador de bordo à parte?** O perfil pressupõe rede
  no modo host e porta UDP 15550 aberta nos dois sentidos.
