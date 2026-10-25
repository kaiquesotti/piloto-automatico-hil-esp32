# Protocolo UART2 e API web

Especificação do enlace entre os dois ESP32 e da interface HTTP da planta, conforme implementado em [`firmware/controle/controle.ino`](../firmware/controle/controle.ino) e [`firmware/planta/planta.ino`](../firmware/planta/planta.ino).

---

## 1. Camada física

| Item | Valor |
|---|---|
| Interface | UART2 dos dois ESP32 |
| Pinos | GPIO16 (RX2) e GPIO17 (TX2), cruzados |
| Velocidade | 460 800 bit/s |
| Formato | 8N1 (8 bits de dados, sem paridade, 1 bit de parada) |
| Referência | GND comum entre as placas |
| Alimentação | cada placa pelo seu próprio cabo USB; **não interligar os barramentos de 3,3 V** |

Ligação:

```text
PLANTA  GPIO17 (TX2) ───────────────► CONTROLE GPIO16 (RX2)
PLANTA  GPIO16 (RX2) ◄─────────────── CONTROLE GPIO17 (TX2)
PLANTA  GND          ◄───────────────► CONTROLE GND
```

---

## 2. Formato da trama

```text
+--------+--------+-----------+---------------------+--------+
| 0xAA   | tipo   | tamanho   | payload (0..64 B)   | XOR    |
+--------+--------+-----------+---------------------+--------+
  1 byte   1 byte   1 byte      "tamanho" bytes       1 byte
```

- **0xAA** — byte de sincronização.
- **tipo** — identifica a mensagem (ver tabela abaixo).
- **tamanho** — número de bytes do payload.
- **XOR** — `tipo XOR tamanho XOR payload[0] XOR ... XOR payload[n-1]`. O byte de início **não** entra no cálculo.

O receptor aceita payloads de até 64 bytes (`gRxBuffer`). Tramas com tamanho maior são descartadas. Se o XOR não confere, a trama é descartada e a máquina de estados volta a `WAIT_START`.

Os campos numéricos são `float32` em **little-endian**, gravados por `memcpy` a partir do `float` (a arquitetura do ESP32 é little-endian).

---

## 3. Mensagens

| Tipo | Nome | Direção | Tamanho | Conteúdo |
|---|---|---|---|---|
| `0x01` | `CONFIG` | planta → controle | 33 B | 8 `float32` + 1 byte `run` |
| `0x02` | `u_pid` | controle → planta | 4 B | `float32` Fx comandado [N] |
| `0x03` | `Y` | planta → controle | 4 B | `float32` velocidade medida [m/s] |

### 3.1 `CONFIG` (0x01) — 33 bytes

| Offset | Tipo | Campo | Unidade |
|---:|---|---|---|
| 0 | `float32` | `Kp` | — |
| 4 | `float32` | `Ki` | — |
| 8 | `float32` | `Kd` | — |
| 12 | `float32` | `N` (filtro derivativo) | 1/s |
| 16 | `float32` | `Ts` (período de amostragem) | s |
| 20 | `float32` | referência inicial | m/s |
| 24 | `float32` | referência final | m/s |
| 28 | `float32` | instante do degrau | s |
| 32 | `uint8` | `run` (0 = parado, 1 = rodando) | — |

A planta envia `CONFIG` quando algum parâmetro muda (`gConfigDirty`) e, como *heartbeat*, a cada 50 ciclos da tarefa da planta — **0,5 s** com Ts = 10 ms.

### 3.2 `u_pid` (0x02) — 4 bytes

| Offset | Tipo | Campo | Unidade |
|---:|---|---|---|
| 0 | `float32` | `u_pid` (força comandada Fx) | N |

Enviada pelo ESP32 do controle a cada período de amostragem, após o primeiro `Y`. A planta aplica esse valor à entrada do atuador de borboleta.

### 3.3 `Y` (0x03) — 4 bytes

| Offset | Tipo | Campo | Unidade |
|---:|---|---|---|
| 0 | `float32` | velocidade medida `u_medida` | m/s |

A velocidade medida inclui o ruído gaussiano configurado. O estado real da planta (sem ruído) é registrado separadamente na coluna `speed_true_mps` do CSV.

---

## 4. Máquina de estados do receptor

```text
WAIT_START ──0xAA──► READ_TYPE ──► READ_LEN ──► READ_PAYLOAD ──► READ_CHECKSUM ──► DISPATCH
     ▲                    │            │              │                 │
     └────────────────────┴────────────┴──────────────┴─────────────────┘
                         XOR inválido ou tamanho > 64: descarta e volta
```

- `READ_LEN`: se `tamanho > 64`, a trama é descartada imediatamente.
- `READ_PAYLOAD`: lê exatamente `tamanho` bytes.
- `READ_CHECKSUM`: compara o byte recebido com o XOR acumulado; se conferir, entrega o payload ao `handleFrame`.
- O parser é não bloqueante (`pollLink`) e é chamado a cada ciclo das tarefas.

---

## 5. Temporização e semântica de controle

| Item | Valor |
|---|---|
| Período de amostragem padrão | `Ts = 10 ms` (100 Hz) |
| Tarefa da planta | núcleo 1, prioridade 3 |
| Tarefa do controle | núcleo 1, prioridade 3 |
| Tarefa web | núcleo 0, prioridade 1 |
| Subpassos da integração da planta | 10 por `Ts` (1 kHz efetivo) |
| Heartbeat do `CONFIG` | 0,5 s |
| Buffer de recepção | 64 bytes |

**Atuador da borboleta** — função de transferência de primeira ordem `Ka/(τa s + 1)`, discretizada por ZOH exato:

```text
a = exp(-Ts/τa)
b = Ka · (1 - a)
Fx[k+1] = a · Fx[k] + b · u_pid[k]
```

**Planta** — integração de Euler à frente com 10 subpassos por `Ts`:

```text
du/dt = Fx/m − g·sin(θ) − f·g·cos(θ) − 0,5·ρ·A·Cd·(u + uw)²/m
```

**Controlador** — PID paralelo com filtro derivativo, Euler à frente:

```text
e = r − u
u_pid = Kp·e + I + Kd·N·(e − f)
I ← I + Ki·Ts·e
f ← f + Ts·N·(e − f)
```

Na borda de subida de `run`, o controlador zera `I`, `f`, `Fx` e `t` e passa a aguardar a primeira mensagem `Y` antes de atuar. Isso evita um pico artificial no arranque.

**Ruído de medição** — gaussiano, proporcional ao módulo da velocidade:

```text
u_medida[k] = u[k] + n[k],   n[k] ~ N(0, (p/100 · |u[k]|)²)
```

`p` é o percentual configurado na página (`noise`), de 0 a 100. O ruído é somado apenas ao sinal enviado ao controlador e à coluna `speed_mps`; `speed_true_mps` permanece sem ruído.

**Perfil θ(t)** — editor de pontos com interpolação ZOH: o valor de um ponto é mantido até o próximo. O perfil padrão é 0° em t = 0, 40° em 50 s, 20° em 100 s, 0° em 150 s e 0° em 200 s.

---

## 6. Registro e decimação

O histórico da planta tem capacidade para **3000 amostras** (`kMaxSamples`). A gravação é feita em uma grade temporal uniforme de espaçamento `gLogDt` (inicialmente `Ts`); amostras fora da grade são descartadas.

Quando o buffer enche:

1. Se `windowS > 0`, as amostras mais antigas que a janela são descartadas.
2. Se ainda estiver cheio, o firmware mantém uma amostra a cada duas, dobra `gLogStride` e `gLogDt` e realinha a grade.
3. Com Ts = 10 ms, isso produz grades de 80 ms (após três decimações) ou 160 ms (após quatro), preservando a escala de tempo.

O CSV exportado contém **todas** as amostras armazenadas; a rota `/data` (usada pelos gráficos) reduz para no máximo ~700 pontos, mantendo também a última amostra.

---

## 7. API web da planta

A planta sobe um servidor HTTP na porta 80. O endereço IP é impresso no Monitor Serial ao conectar à rede.

| Rota | Método | Descrição |
|---|---|---|
| `/` | GET | página HTML embutida (`index_html.h`) |
| `/params` | GET | JSON com todos os parâmetros atuais |
| `/status` | GET | motivo do último reset, código, uptime, heap livre e estado `running` |
| `/apply` | POST | aplica parâmetros enviados como formulário |
| `/run?state=0\|1` | GET | para (`0`) ou inicia (`1`) a simulação |
| `/clear` | GET | para e limpa o histórico |
| `/data` | GET | telemetria binária para os gráficos |
| `/csv` | GET | download do CSV com todas as amostras |

### 7.1 Formato de `/data`

Cabeçalho de **32 bytes**:

| Offset | Tipo | Campo | Observação |
|---:|---|---|---|
| 0 | `uint32` | magic | `0x31444950` — bytes `"PID1"` em little-endian |
| 4 | `uint32` | epoch | incrementa a cada `resetRun()` |
| 8 | `uint32` | count | número de amostras armazenadas |
| 12 | `uint32` | stride | fator de decimação |
| 16 | `uint32` | nPoints | número de registros enviados |
| 20 | `float32` | Ts | período de amostragem [s] |
| 24 | `float32` | t | tempo atual da planta [s] |
| 28 | `uint8` | running | 1 = rodando |
| 29–31 | `uint8[3]` | reservado | zeros |

Cada registro tem **20 bytes** (5 `float32` little-endian):

| Offset | Campo | Unidade |
|---:|---|---|
| 0 | `t` | s |
| 4 | `r` | m/s |
| 8 | `u` (medida, com ruído) | m/s |
| 12 | `Fx` (força efetiva após o atuador) | N |
| 16 | `θ` | graus |

O tempo de cada registro é `t[i] = gLogT0 + i·gLogDt`, reconstruído a partir da grade uniforme — isso evita acumular erro de ponto flutuante.

### 7.2 Formato de `/csv`

Cabeçalho exato:

```csv
time_s,reference_mps,speed_mps,fx_N,theta_deg,error_mps,speed_true_mps
```

- `time_s` — tempo [s]
- `reference_mps` — referência r [m/s]
- `speed_mps` — velocidade medida (com ruído) [m/s]
- `fx_N` — força efetiva após o atuador [N]
- `theta_deg` — inclinação [°]
- `error_mps` — `r − speed_mps` [m/s]
- `speed_true_mps` — estado real da planta, sem ruído [m/s]

O arquivo é transmitido em streaming, com `Content-Disposition: attachment; filename=piloto_pi_nao_linear.csv`.

---

## 8. Diagrama resumido

```mermaid
sequenceDiagram
    participant P as ESP32 planta
    participant C as ESP32 controle
    P->>C: CONFIG (Kp, Ki, Kd, N, Ts, r0, r1, t_step, run) — 0,5 s
    Note over C: borda de subida de run:<br/>zera I, f, Fx e t
    P->>C: Y (u medida) — a cada Ts
    C->>P: u_pid (Fx) — a cada Ts
    Note over P: atuador ZOH + 10 subpassos<br/>integra u; aplica ruído na medição
    P->>P: registra t, r, u, Fx, θ
```
