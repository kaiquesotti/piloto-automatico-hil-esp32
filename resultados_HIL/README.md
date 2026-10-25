# Resultados experimentais

Este diretório reúne os dados brutos e as métricas consolidadas da comparação MIL × HIL descrita no artigo.

| Arquivo | Origem | Grade temporal | Colunas |
|---|---|---|---|
| `resultado_MIL.csv` | Simulink R2024a, `matlab/run_mil_export.m` | 10 ms | 5 |
| `resultado1_0_ruido.csv` | HIL, ruído de 0 % | 80 ms | 7 |
| `resultado1_0.5_ruido.csv` | HIL, ruído de 0,5 % | 80 ms | 7 |
| `resultado1_1.0_ruido.csv` | HIL, ruído de 1 % | 160 ms | 7 |
| `metricas_mil_hil.csv` | Métricas calculadas sobre os CSVs acima | — | 18 |

Os arquivos HIL foram exportados pela rota `/csv` da página web da planta após cada corrida. A decimação é automática: o histórico tem capacidade para 3000 amostras e, quando enche, o firmware mantém uma amostra a cada duas e realinha a grade (ver [`docs/PROTOCOLO_UART.md`](../docs/PROTOCOLO_UART.md#6-registro-e-decimação)).

As corridas duraram mais de 200 s (210,4 s para 0 %, 205,2 s para 0,5 % e 255,84 s para 1 %). As métricas do artigo foram calculadas no intervalo de **0 a 200 s**, por interpolação das séries em uma grade comum de 10 ms.

---

## 1. `resultado_MIL.csv`

Exportado por `run_mil_export.m`. Colunas:

| Coluna | Unidade | Descrição |
|---|---|---|
| `time_s` | s | tempo de simulação |
| `reference_kmh` | km/h | referência r (degrau de 90 km/h em t = 0) |
| `speed_mps` | m/s | velocidade u do modelo (sem ruído) |
| `fx_N` | N | força efetiva após o atuador |
| `theta_deg` | ° | inclinação da pista |

> [!NOTE]
> A referência neste arquivo está em **km/h** (`reference_kmh`), enquanto a velocidade está em **m/s** (`speed_mps`). Nos CSVs do HIL, a referência está em m/s (`reference_mps`). A conversão é `km/h = 3,6 × m/s`.
>
> O arquivo contém **uma amostra duplicada em t = 0**, decorrente do registro da condição inicial pelo bloco *To Workspace*. Ela não afeta as métricas, calculadas por interpolação.

---

## 2. CSVs do HIL

Todos os três arquivos têm o mesmo cabeçalho:

```csv
time_s,reference_mps,speed_mps,fx_N,theta_deg,error_mps,speed_true_mps
```

| Coluna | Unidade | Descrição |
|---|---|---|
| `time_s` | s | tempo da planta |
| `reference_mps` | m/s | referência r(t) |
| `speed_mps` | m/s | velocidade **medida**, com ruído gaussiano |
| `fx_N` | N | força efetiva após o atuador (não é o comando bruto do PID) |
| `theta_deg` | ° | inclinação da pista |
| `error_mps` | m/s | erro `r − speed_mps` (usa a velocidade medida) |
| `speed_true_mps` | m/s | estado real da planta, **sem ruído** |

O ruído é aplicado apenas ao sinal enviado ao controlador e à coluna `speed_mps`; `speed_true_mps` é a velocidade do integrador. A diferença entre as duas colunas é a parcela de ruído de medição.

---

## 3. `metricas_mil_hil.csv`

Uma linha por ensaio (`MIL`, `HIL 0%`, `HIL 0.5%`, `HIL 1%`). As métricas foram calculadas por interpolação das duas séries em uma grade comum de 10 ms, no intervalo de 0 a 200 s.

| Coluna | Unidade | Descrição |
|---|---|---|
| `ensaio` | — | identificação do ensaio |
| `rise90_s` | s | tempo de subida 10–90 % |
| `overshoot_pct` | % | sobressinal máximo |
| `settle2_s` | s | tempo de assentamento com banda de ±2 % |
| `erro_ss_kmh` | km/h | erro de regime permanente |
| `u_min_40_kmh` | km/h | velocidade mínima durante a rampa de 40° |
| `u_max_20_kmh` | km/h | velocidade máxima durante a rampa de 20° |
| `u_max_0_kmh` | km/h | velocidade máxima após o retorno a 0° |
| `fx_max_kN` | kN | pico da força de controle |
| `fx_rms_ss_N` | N | valor RMS da força em regime permanente |
| `noise_sigma_mps` | m/s | desvio-padrão do ruído de medição |
| `rmse_true_kmh` | km/h | RMSE da velocidade real em relação ao MIL |
| `rmse_meas_kmh` | km/h | RMSE da velocidade medida em relação ao MIL |
| `mae_true_kmh` | km/h | erro absoluto médio da velocidade real |
| `r_true` | — | coeficiente de correlação de Pearson — velocidade real × MIL |
| `r_meas` | — | coeficiente de correlação de Pearson — velocidade medida × MIL |
| `r2_true` | — | coeficiente de determinação R² — velocidade real × MIL |
| `fx_rmse_N` | N | RMSE da força em relação ao MIL |

Observações:

- A linha `MIL` é a própria referência; por isso suas métricas de erro e correlação são triviais (`rmse_* = 0`, `r = 1`, `fx_rmse = 0`).
- A Tabela II do artigo também reporta a velocidade de pico `u_max` e os tempos de recuperação após cada rampa de θ. Essas métricas não estão presentes neste CSV; os valores completos constam no artigo e são reproduzidos no [README principal](../README.md#resultados).
- A Tabela III do artigo apresenta apenas as três linhas HIL; a linha MIL do CSV foi incluída por completude.
