/*
 * =============================================================================
 *  planta.ino  --  ESP32 da PLANTA (Piloto automatico - MODELO NAO LINEAR)
 * =============================================================================
 *
 *  Reproduz a PLANTA do modelo Simulink "nao_linear_piloto_automatico":
 *
 *      u_pid --[ Ka/(TauA*s + 1) ]--> Fx
 *      du/dt = Fx/m - g*sin(Theta) - f*g*cos(Theta) - 0.5*rho*A*Cd*(u+uw)^2/m
 *
 *  com:
 *    u  = velocidade longitudinal [m/s]  (estado / saida do integrador)
 *    Fx = forca de tracao/frenagem [N]    (saida do atuador da borboleta)
 *    u_pid = comando do PID recebido do controle [N]
 *    Theta = inclinacao da pista [rad]    (editor de sinal, em graus na pagina)
 *
 *  RUIDO DE MEDICAO: a velocidade enviada ao controle (e registrada na coluna
 *  speed_mps) recebe ruido gaussiano proporcional a |u|; o percentual e
 *  ajustavel na pagina (0 = sem ruido).
 *
 *  CONDICAO INICIAL do integrador: campo "u0" na pagina (padrao 50 km/h).
 *  O modelo roda em TEMPO REAL, sem "stop time": so para quando voce manda.
 *
 *  Este ESP32 tambem:
 *    - entra na rede Wi-Fi definida em secrets.h (STA) e hospeda a PAGINA WEB;
 *    - ajusta parametros, condicao inicial, referencia e o SINAL Theta
 *      (editor de pontos + gerador de rampa);
 *    - inicia/para/limpa a simulacao;
 *    - envia/recebe Fx/u por FIOS (UART2) com o outro ESP32.
 *
 *  PINOS DO ENLACE (UART2) -- cruzar com o controle:
 *    PLANTA GPIO17 (TX2) ----> CONTROLE GPIO16 (RX2)
 *    PLANTA GPIO16 (RX2) <---- CONTROLE GPIO17 (TX2)
 *    PLANTA GND          <--> CONTROLE GND
 *    (NAO interligar os barramentos de 3,3 V)
 *
 *  IDIOMA: comentarios/documentacao em portugues; identificadores em ingles.
 * =============================================================================
 */

#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_system.h>

#include <cmath>
#include <cstdint>
#include <cstring>

#include "index_html.h"
#include "secrets.h"  // arquivo local (não versionado); copie secrets.example.h para secrets.h

/* ===========================================================================
 *  Configuracao
 * ===========================================================================
 */
namespace config {
const char* const kWifiSsid = WIFI_SSID;
const char* const kWifiPassword = WIFI_PASSWORD;

constexpr int kLinkRxPin = 16;
constexpr int kLinkTxPin = 17;
constexpr uint32_t kLinkBaud = 460800;

constexpr uint8_t kStartByte = 0xAA;
constexpr uint8_t kMsgConfig = 0x01;  // planta -> controle
constexpr uint8_t kMsgU = 0x02;       // controle -> planta (Fx)
constexpr uint8_t kMsgY = 0x03;       // planta -> controle (u)

constexpr double kDefaultSampleTimeS = 0.01;
constexpr int kPlantSubsteps = 10;
constexpr int kMaxThetaPoints = 32;

constexpr int kMaxSamples = 3000;  // 30 s a Ts=10 ms (limite de DRAM do ESP32)
constexpr int kPlotPoints = 700;
constexpr int kChunkPoints = 48;
constexpr int kRecordBytes = 20;  // t, r, u, Fx, theta

constexpr uint32_t kDebugBaud = 115200;
}  // namespace config

/* ===========================================================================
 *  Tipos
 * ===========================================================================
 */
struct Params {
  // Planta nao linear
  double m = 1000.0;
  double g = 9.81;
  double rho = 1.202;
  double area = 1.0;
  double cd = 0.5;
  double uw = 2.0;
  double f = 0.015;

  // Atuador da borboleta: mesma Transfer Fcn do Simulink (Ka/(TauA*s + 1)).
  double ka = 10.0;
  double tauA = 0.2;

  // Ruido de medicao da velocidade (% de |u|, sigma gaussiano).
  double noisePct = 0.5;

  // Condicao inicial do integrador (km/h)
  double speed0Kmh = 50.0;

  // PID (repassado ao controle)
  double kp = 477.204600781084;
  double ki = 42.1664078975366;
  double kd = 178.763139862948;
  double nf = 1.8234120333855;

  double ts = config::kDefaultSampleTimeS;

  // Referencia (Step), em km/h
  double refInitialKmh = 0.0;
  double refFinalKmh = 90.0;
  double refTime = 0.0;

  // Visualizacao
  double windowS = 0.0;

  // Editor do sinal Theta (graus). t em segundos. Sempre ZOH (degraus).
  double thetaT[config::kMaxThetaPoints];
  double thetaV[config::kMaxThetaPoints];
  int thetaCount = 0;
};

enum RxState : uint8_t {
  WAIT_START,
  READ_TYPE,
  READ_LEN,
  READ_PAYLOAD,
  READ_CHECKSUM,
};

/* ===========================================================================
 *  Estado global
 * ===========================================================================
 */
static WebServer gServer(80);
static Params gParams;

static volatile double gFx = 0.0;  // comando u_pid recebido do controle
static double gFxAct = 0.0;        // forca efetiva apos o atuador [N]
static volatile bool gConfigDirty = true;

// Gerador gaussiano (xorshift32 + Box-Muller) para o ruido de medicao.
static uint32_t gRngState = 0x6D2B79F5u;
static double gGaussSpare = 0.0;
static bool gGaussHasSpare = false;

static bool gRunning = false;
static double gTime = 0.0;
static double gSpeed = 0.0;    // u [m/s]
static double gRef = 0.0;      // r [m/s]
static double gThetaDeg = 0.0; // Theta atual [graus]

// Historico (t implicito: t = t0 + i*dt)
static float gLogR[config::kMaxSamples];
static float gLogU[config::kMaxSamples];      // velocidade medida (ruidosa)
static float gLogUTrue[config::kMaxSamples];  // velocidade real do integrador
static float gLogFx[config::kMaxSamples];
static float gLogTheta[config::kMaxSamples];
static int gLogCount = 0;
static int gLogStride = 1;
static double gLogT0 = 0.0;
static double gLogDt = config::kDefaultSampleTimeS;
static double gNextStoreTime = -1.0;  // proxima amostra da grade uniforme
static uint32_t gEpoch = 1;

// Parser UART
static RxState gRxState = WAIT_START;
static uint8_t gRxType = 0;
static uint8_t gRxLen = 0;
static uint8_t gRxIndex = 0;
static uint8_t gRxChecksum = 0;
static uint8_t gRxBuffer[64];

/* ===========================================================================
 *  Funcoes auxiliares
 * ===========================================================================
 */
static double clampValue(double value, double lower, double upper) {
  if (value < lower) return lower;
  if (value > upper) return upper;
  return value;
}

// Ruido gaussiano (media 0, desvio 1): xorshift32 + Box-Muller polar.
static double randomUniform() {
  gRngState ^= gRngState << 13;
  gRngState ^= gRngState >> 17;
  gRngState ^= gRngState << 5;
  return static_cast<double>(gRngState >> 8) * (1.0 / 16777216.0);
}

static double randomGaussian() {
  if (gGaussHasSpare) {
    gGaussHasSpare = false;
    return gGaussSpare;
  }
  double u = 0.0;
  double v = 0.0;
  double s = 0.0;
  do {
    u = 2.0 * randomUniform() - 1.0;
    v = 2.0 * randomUniform() - 1.0;
    s = u * u + v * v;
  } while (s >= 1.0 || s <= 1e-12);
  const double factor = std::sqrt(-2.0 * std::log(s) / s);
  gGaussSpare = v * factor;
  gGaussHasSpare = true;
  return u * factor;
}

static double stepValue(double initial, double finalValue, double timeS,
                        double t) {
  return (t >= timeS) ? finalValue : initial;
}

// Traduz o motivo do ultimo reset (para diagnostico de reinicios).
static const char* resetReasonText(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:
      return "POWERON";
    case ESP_RST_EXT:
      return "EXTERNAL";
    case ESP_RST_SW:
      return "SOFTWARE";
    case ESP_RST_PANIC:
      return "PANIC";
    case ESP_RST_INT_WDT:
      return "INT_WDT";
    case ESP_RST_TASK_WDT:
      return "TASK_WDT";
    case ESP_RST_WDT:
      return "WDT";
    case ESP_RST_DEEPSLEEP:
      return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:
      return "BROWNOUT";
    case ESP_RST_SDIO:
      return "SDIO";
    default:
      return "UNKNOWN";
  }
}

// Avalia o sinal Theta (graus) com ZOH (degrau): mantem o valor ate o proximo
// ponto. Sem interpolacao (o sinal da imagem e uma sequencia de degraus).
static double evaluateThetaDeg(double t) {
  const int n = gParams.thetaCount;
  if (n <= 0) {
    return 0.0;
  }
  if (t <= gParams.thetaT[0]) {
    return gParams.thetaV[0];
  }
  for (int i = 1; i < n; ++i) {
    if (t < gParams.thetaT[i]) {
      return gParams.thetaV[i - 1];
    }
  }
  return gParams.thetaV[n - 1];
}

// Sinal padrao: degraus iguais ao SINAL_RAMPA_1 (0 -> 40 -> 20 -> 0 graus).
// ZOH (sem interpolacao), reproduzindo a imagem do Signal Editor.
static void loadDefaultSignal() {
  if (gParams.thetaCount > 0) {
    return;
  }
  gParams.thetaT[0] = 0.0;
  gParams.thetaV[0] = 0.0;
  gParams.thetaT[1] = 50.0;
  gParams.thetaV[1] = 40.0;
  gParams.thetaT[2] = 100.0;
  gParams.thetaV[2] = 20.0;
  gParams.thetaT[3] = 150.0;
  gParams.thetaV[3] = 0.0;
  gParams.thetaT[4] = 200.0;
  gParams.thetaV[4] = 0.0;
  gParams.thetaCount = 5;
}

static void packFloat(uint8_t* buffer, int index, float value) {
  memcpy(&buffer[index * 4], &value, sizeof(float));
}

static float unpackFloat(const uint8_t* buffer, int index) {
  float value = 0.0F;
  memcpy(&value, &buffer[index * 4], sizeof(float));
  return value;
}

static void sendFrame(uint8_t type, const uint8_t* payload, uint8_t length) {
  uint8_t checksum = static_cast<uint8_t>(type ^ length);
  Serial2.write(config::kStartByte);
  Serial2.write(type);
  Serial2.write(length);
  for (uint8_t i = 0; i < length; ++i) {
    Serial2.write(payload[i]);
    checksum ^= payload[i];
  }
  Serial2.write(checksum);
}

// Kp, Ki, Kd, N, Ts, ref(m/s), ref tempo, run  -> 8 floats + 1 byte = 33.
static void sendConfig() {
  uint8_t payload[33];
  packFloat(payload, 0, static_cast<float>(gParams.kp));
  packFloat(payload, 1, static_cast<float>(gParams.ki));
  packFloat(payload, 2, static_cast<float>(gParams.kd));
  packFloat(payload, 3, static_cast<float>(gParams.nf));
  packFloat(payload, 4, static_cast<float>(gParams.ts));
  packFloat(payload, 5, static_cast<float>(gParams.refInitialKmh / 3.6));
  packFloat(payload, 6, static_cast<float>(gParams.refFinalKmh / 3.6));
  packFloat(payload, 7, static_cast<float>(gParams.refTime));
  payload[32] = gRunning ? 1U : 0U;
  sendFrame(config::kMsgConfig, payload, sizeof(payload));
}

static void resetRun() {
  gTime = 0.0;
  // Condicao inicial do integrador (estado u).
  gSpeed = gParams.speed0Kmh / 3.6;
  gRef = stepValue(gParams.refInitialKmh / 3.6, gParams.refFinalKmh / 3.6,
                   gParams.refTime, 0.0);
  gThetaDeg = evaluateThetaDeg(0.0);
  gFx = 0.0;
  gFxAct = 0.0;
  gLogCount = 0;
  gLogStride = 1;
  gLogDt = (gParams.ts > 1e-4) ? gParams.ts : config::kDefaultSampleTimeS;
  gLogT0 = 0.0;
  gNextStoreTime = -1.0;
  ++gEpoch;
}

static void handleFrame(uint8_t type, const uint8_t* payload, uint8_t length) {
  if (type == config::kMsgU && length >= 4) {
    gFx = unpackFloat(payload, 0);
  }
}

static void pollLink() {
  while (Serial2.available() > 0) {
    const uint8_t byteValue = static_cast<uint8_t>(Serial2.read());
    switch (gRxState) {
      case WAIT_START:
        if (byteValue == config::kStartByte) gRxState = READ_TYPE;
        break;
      case READ_TYPE:
        gRxType = byteValue;
        gRxChecksum = byteValue;
        gRxState = READ_LEN;
        break;
      case READ_LEN:
        gRxLen = byteValue;
        if (gRxLen > sizeof(gRxBuffer)) {
          gRxState = WAIT_START;
          break;
        }
        gRxChecksum ^= byteValue;
        gRxIndex = 0;
        gRxState = (gRxLen > 0) ? READ_PAYLOAD : READ_CHECKSUM;
        break;
      case READ_PAYLOAD:
        gRxBuffer[gRxIndex++] = byteValue;
        gRxChecksum ^= byteValue;
        if (gRxIndex >= gRxLen) gRxState = READ_CHECKSUM;
        break;
      case READ_CHECKSUM:
        if (byteValue == gRxChecksum) {
          handleFrame(gRxType, gRxBuffer, gRxLen);
        }
        gRxState = WAIT_START;
        break;
    }
  }
}

/* ===========================================================================
 *  Historico e serie binaria
 * ===========================================================================
 */
// Armazena em uma GRADE UNIFORME de espacamento gLogDt. Amostras que nao caem
// na grade sao descartadas; ao decimar, gLogDt dobra e a grade e realinhada.
// Assim t[i] = gLogT0 + i*gLogDt permanece exato (sem distorcer a escala).
static void storeSample(double t, double r, double u, double uTrue, double fx,
                        double thetaDeg) {
  if (t + 1e-9 < gNextStoreTime) {
    return;  // ainda nao e hora da proxima amostra da grade
  }
  if (gLogCount >= config::kMaxSamples) {
    if (gParams.windowS > 0.0) {
      const int drop =
          static_cast<int>((t - gParams.windowS - gLogT0) / gLogDt);
      if (drop >= gLogCount) {
        gLogCount = 0;
      } else if (drop > 0) {
        const int remaining = gLogCount - drop;
        memmove(gLogR, gLogR + drop, remaining * sizeof(float));
        memmove(gLogU, gLogU + drop, remaining * sizeof(float));
        memmove(gLogUTrue, gLogUTrue + drop, remaining * sizeof(float));
        memmove(gLogFx, gLogFx + drop, remaining * sizeof(float));
        memmove(gLogTheta, gLogTheta + drop, remaining * sizeof(float));
        gLogCount = remaining;
        gLogT0 += drop * gLogDt;
      }
    }
    if (gLogCount >= config::kMaxSamples) {
      int kept = 0;
      for (int i = 0; i < gLogCount; i += 2) {
        gLogR[kept] = gLogR[i];
        gLogU[kept] = gLogU[i];
        gLogUTrue[kept] = gLogUTrue[i];
        gLogFx[kept] = gLogFx[i];
        gLogTheta[kept] = gLogTheta[i];
        ++kept;
      }
      gLogCount = kept;
      gLogStride *= 2;
      gLogDt *= 2.0;
      // realinha a grade apos a decimacao (pode descartar esta amostra)
      gNextStoreTime = gLogT0 + static_cast<double>(gLogCount) * gLogDt;
    }
  }
  if (t + 1e-9 < gNextStoreTime) {
    return;  // alinha na nova grade
  }
  if (gLogCount == 0) {
    gLogT0 = t;
  }
  gLogR[gLogCount] = static_cast<float>(r);
  gLogU[gLogCount] = static_cast<float>(u);
  gLogUTrue[gLogCount] = static_cast<float>(uTrue);
  gLogFx[gLogCount] = static_cast<float>(fx);
  gLogTheta[gLogCount] = static_cast<float>(thetaDeg);
  ++gLogCount;
  gNextStoreTime = t + gLogDt;
}

static void putU32(uint8_t*& cursor, uint32_t value) {
  memcpy(cursor, &value, 4);
  cursor += 4;
}

static void putF32(uint8_t*& cursor, float value) {
  memcpy(cursor, &value, 4);
  cursor += 4;
}

// /data binario: header 32 B + nPoints*(t,r,u,Fx,theta) float32.
static void sendDataBinary() {
  const int count = gLogCount;
  int step = 1;
  if (count > config::kPlotPoints) {
    step = count / config::kPlotPoints;
    if (step < 1) step = 1;
  }
  int nPoints = (count + step - 1) / step;
  const bool addLast = (count > 0) && (((count - 1) % step) != 0);
  if (addLast) ++nPoints;
  const size_t totalBytes = 32 + static_cast<size_t>(nPoints) * config::kRecordBytes;

  uint8_t header[32];
  uint8_t* cursor = header;
  putU32(cursor, 0x31444950u);  // "PID1"
  putU32(cursor, gEpoch);
  putU32(cursor, static_cast<uint32_t>(count));
  putU32(cursor, static_cast<uint32_t>(gLogStride));
  putU32(cursor, static_cast<uint32_t>(nPoints));
  putF32(cursor, static_cast<float>(gParams.ts));
  putF32(cursor, static_cast<float>(gTime));
  *cursor++ = gRunning ? 1U : 0U;
  *cursor++ = 0U;
  *cursor++ = 0U;
  *cursor++ = 0U;

  gServer.setContentLength(totalBytes);
  gServer.send(200, "application/octet-stream", "");
  gServer.sendContent(reinterpret_cast<const char*>(header), sizeof(header));

  uint8_t chunk[config::kChunkPoints * config::kRecordBytes];
  uint8_t* chunkCursor = chunk;
  int inChunk = 0;
  for (int i = 0; i < count; i += step) {
    putF32(chunkCursor, static_cast<float>(gLogT0 + static_cast<double>(i) * gLogDt));
    putF32(chunkCursor, gLogR[i]);
    putF32(chunkCursor, gLogU[i]);
    putF32(chunkCursor, gLogFx[i]);
    putF32(chunkCursor, gLogTheta[i]);
    if (++inChunk == config::kChunkPoints) {
      gServer.sendContent(reinterpret_cast<const char*>(chunk),
                          static_cast<size_t>(inChunk) * config::kRecordBytes);
      chunkCursor = chunk;
      inChunk = 0;
    }
  }
  if (addLast && count > 0) {
    const int i = count - 1;
    putF32(chunkCursor, static_cast<float>(gLogT0 + static_cast<double>(i) * gLogDt));
    putF32(chunkCursor, gLogR[i]);
    putF32(chunkCursor, gLogU[i]);
    putF32(chunkCursor, gLogFx[i]);
    putF32(chunkCursor, gLogTheta[i]);
    ++inChunk;
  }
  if (inChunk > 0) {
    gServer.sendContent(reinterpret_cast<const char*>(chunk),
                        static_cast<size_t>(inChunk) * config::kRecordBytes);
  }
}

/* ===========================================================================
 *  JSON de parametros e parser do editor de sinal
 * ===========================================================================
 */
static String thetaText() {
  String text = "";
  for (int i = 0; i < gParams.thetaCount; ++i) {
    if (i > 0) text += "\\n";  // escape JSON de quebra de linha
    text += String(gParams.thetaT[i], 3) + "," + String(gParams.thetaV[i], 3);
  }
  return text;
}

static String paramsJson() {
  String json = "{";
  json += "\"m\":" + String(gParams.m, 2);
  json += ",\"g\":" + String(gParams.g, 4);
  json += ",\"rho\":" + String(gParams.rho, 4);
  json += ",\"A\":" + String(gParams.area, 3);
  json += ",\"Cd\":" + String(gParams.cd, 3);
  json += ",\"uw\":" + String(gParams.uw, 3);
  json += ",\"f\":" + String(gParams.f, 4);
  json += ",\"ka\":" + String(gParams.ka, 3);
  json += ",\"tauA\":" + String(gParams.tauA, 4);
  json += ",\"noise\":" + String(gParams.noisePct, 3);
  json += ",\"speed0\":" + String(gParams.speed0Kmh, 2);
  json += ",\"kp\":" + String(gParams.kp, 6);
  json += ",\"ki\":" + String(gParams.ki, 6);
  json += ",\"kd\":" + String(gParams.kd, 6);
  json += ",\"nf\":" + String(gParams.nf, 6);
  json += ",\"ts\":" + String(gParams.ts, 5);
  json += ",\"refInit\":" + String(gParams.refInitialKmh, 2);
  json += ",\"refFinal\":" + String(gParams.refFinalKmh, 2);
  json += ",\"refTime\":" + String(gParams.refTime, 3);
  json += ",\"window\":" + String(gParams.windowS, 1);
  json += ",\"theta\":\"" + thetaText() + "\"";
  json += ",\"running\":" + String(gRunning ? 1 : 0);
  json += "}";
  return json;
}

static void parseTheta(const String& text) {
  double times[config::kMaxThetaPoints];
  double values[config::kMaxThetaPoints];
  int count = 0;
  int start = 0;
  const int length = static_cast<int>(text.length());
  while (start < length && count < config::kMaxThetaPoints) {
    int end = text.indexOf('\n', start);
    if (end < 0) end = length;
    String line = text.substring(start, end);
    line.trim();
    line.replace('\r', ' ');
    line.trim();
    if (line.length() > 0) {
      int sep = line.indexOf(',');
      if (sep < 0) sep = line.indexOf(';');
      if (sep > 0) {
        times[count] = line.substring(0, sep).toFloat();
        values[count] = line.substring(sep + 1).toFloat();
        ++count;
      }
    }
    start = end + 1;
  }
  if (count <= 0) {
    return;
  }
  // Ordena por tempo (insercao).
  for (int i = 1; i < count; ++i) {
    const double tk = times[i];
    const double vk = values[i];
    int j = i - 1;
    while (j >= 0 && times[j] > tk) {
      times[j + 1] = times[j];
      values[j + 1] = values[j];
      --j;
    }
    times[j + 1] = tk;
    values[j + 1] = vk;
  }
  for (int i = 0; i < count; ++i) {
    gParams.thetaT[i] = times[i];
    gParams.thetaV[i] = values[i];
  }
  gParams.thetaCount = count;
}

/* ===========================================================================
 *  Tarefas
 * ===========================================================================
 */
static void plantTask(void*) {
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t heartbeat = 0;
  for (;;) {
    const double ts =
        (gParams.ts > 1e-4) ? gParams.ts : config::kDefaultSampleTimeS;
    const TickType_t periodTicks =
        pdMS_TO_TICKS(static_cast<uint32_t>(ts * 1000.0));
    vTaskDelayUntil(&lastWake, (periodTicks > 0) ? periodTicks : 1);

    pollLink();

    if (gConfigDirty || (heartbeat % 50U) == 0U) {
      gConfigDirty = false;
      sendConfig();
    }
    ++heartbeat;

    if (!gRunning) {
      continue;
    }

    gRef = stepValue(gParams.refInitialKmh / 3.6, gParams.refFinalKmh / 3.6,
                     gParams.refTime, gTime);
    gThetaDeg = evaluateThetaDeg(gTime);
    const double thetaRad = gThetaDeg * 3.14159265358979323846 / 180.0;

    // Atuador da borboleta: Ka/(TauA*s + 1), discretizado por ZOH exato
    // (mesmo bloco Transfer Fcn do modelo Simulink).
    const double tauA = (gParams.tauA > 1e-4) ? gParams.tauA : 1e-4;
    const double aAct = std::exp(-ts / tauA);
    const double bAct = gParams.ka * (1.0 - aAct);
    gFxAct = aAct * gFxAct + bAct * gFx;

    // Integra a dinamica nao linear com subpassos (reavalia o arrasto em u).
    const double dt = ts / config::kPlantSubsteps;
    for (int s = 0; s < config::kPlantSubsteps; ++s) {
      const double dragAccel =
          0.5 * gParams.rho * gParams.area * gParams.cd *
          (gSpeed + gParams.uw) * (gSpeed + gParams.uw) / gParams.m;
      const double du = gFxAct / gParams.m - gParams.g * std::sin(thetaRad) -
                        gParams.f * gParams.g * std::cos(thetaRad) - dragAccel;
      gSpeed += dt * du;
    }

    // Ruido de medicao proporcional a |u| (sigma em % da velocidade).
    double measured = gSpeed;
    if (gParams.noisePct > 0.0 && std::fabs(gSpeed) > 1e-9) {
      measured += randomGaussian() * (gParams.noisePct / 100.0) *
                  std::fabs(gSpeed);
    }

    // Envia a velocidade medida (ruidosa) ao controlador.
    uint8_t payload[4];
    packFloat(payload, 0, static_cast<float>(measured));
    sendFrame(config::kMsgY, payload, sizeof(payload));

    storeSample(gTime, gRef, measured, gSpeed, gFxAct, gThetaDeg);
    gTime += ts;
  }
}

static void webTask(void*) {
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.reconnect();
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    gServer.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

/* ===========================================================================
 *  setup() / loop()
 * ===========================================================================
 */
void setup() {
  Serial.begin(config::kDebugBaud);
  delay(200);
  const esp_reset_reason_t resetReason = esp_reset_reason();
  Serial.printf("Reset reason: %s (%d), free heap: %u B\n",
                resetReasonText(resetReason), static_cast<int>(resetReason),
                static_cast<unsigned>(ESP.getFreeHeap()));
  loadDefaultSignal();

  // Semente do ruido: varia com o instante de boot (nao precisa ser repetivel).
  gRngState = static_cast<uint32_t>(micros()) ^ 0x9E3779B9u;
  if (gRngState == 0u) gRngState = 0x6D2B79F5u;

  Serial2.setRxBufferSize(1024);
  Serial2.begin(config::kLinkBaud, SERIAL_8N1, config::kLinkRxPin,
                config::kLinkTxPin);

  WiFi.mode(WIFI_STA);
  // Potencia de TX menor -> menores picos de corrente (evita brownout em USB fraco).
  WiFi.setTxPower(WIFI_POWER_11dBm);
  WiFi.begin(config::kWifiSsid, config::kWifiPassword);
  Serial.print("Conectando a Wi-Fi ");
  Serial.println(config::kWifiSsid);
  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(400);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("PAGINA WEB: http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Wi-Fi nao conectou; a pagina web ficara indisponivel.");
  }

  gServer.on("/", HTTP_GET, []() {
    gServer.send_P(200, "text/html", kIndexHtml);
  });
  gServer.on("/params", HTTP_GET, []() {
    gServer.send(200, "application/json", paramsJson());
  });
  gServer.on("/status", HTTP_GET, []() {
    const esp_reset_reason_t reason = esp_reset_reason();
    String json = "{";
    json += "\"reason\":\"" + String(resetReasonText(reason)) + "\"";
    json += ",\"reason_code\":" + String(static_cast<int>(reason));
    json += ",\"uptime_ms\":" + String(static_cast<uint32_t>(millis()));
    json += ",\"heap\":" + String(ESP.getFreeHeap());
    json += ",\"running\":" + String(gRunning ? 1 : 0);
    json += "}";
    gServer.send(200, "application/json", json);
  });
  gServer.on("/apply", HTTP_POST, []() {
    if (gServer.hasArg("m")) gParams.m = gServer.arg("m").toFloat();
    if (gServer.hasArg("g")) gParams.g = gServer.arg("g").toFloat();
    if (gServer.hasArg("rho")) gParams.rho = gServer.arg("rho").toFloat();
    if (gServer.hasArg("A")) gParams.area = gServer.arg("A").toFloat();
    if (gServer.hasArg("Cd")) gParams.cd = gServer.arg("Cd").toFloat();
    if (gServer.hasArg("uw")) gParams.uw = gServer.arg("uw").toFloat();
    if (gServer.hasArg("f")) gParams.f = gServer.arg("f").toFloat();
    if (gServer.hasArg("ka")) gParams.ka = gServer.arg("ka").toFloat();
    if (gServer.hasArg("tauA")) {
      const double tauA = gServer.arg("tauA").toFloat();
      if (tauA > 1e-4) gParams.tauA = tauA;
    }
    if (gServer.hasArg("noise")) {
      const double noise = gServer.arg("noise").toFloat();
      gParams.noisePct = (noise > 0.0) ? noise : 0.0;
    }
    if (gServer.hasArg("speed0")) gParams.speed0Kmh = gServer.arg("speed0").toFloat();
    if (gServer.hasArg("kp")) gParams.kp = gServer.arg("kp").toFloat();
    if (gServer.hasArg("ki")) gParams.ki = gServer.arg("ki").toFloat();
    if (gServer.hasArg("kd")) gParams.kd = gServer.arg("kd").toFloat();
    if (gServer.hasArg("nf")) gParams.nf = gServer.arg("nf").toFloat();
    if (gServer.hasArg("ts")) {
      const double ts = gServer.arg("ts").toFloat();
      if (ts > 1e-4) gParams.ts = ts;
    }
    if (gServer.hasArg("refInit")) gParams.refInitialKmh = gServer.arg("refInit").toFloat();
    if (gServer.hasArg("refFinal")) gParams.refFinalKmh = gServer.arg("refFinal").toFloat();
    if (gServer.hasArg("refTime")) gParams.refTime = gServer.arg("refTime").toFloat();
    if (gServer.hasArg("window")) {
      const double w = gServer.arg("window").toFloat();
      gParams.windowS = (w > 0.0) ? w : 0.0;
    }
    if (gServer.hasArg("theta")) parseTheta(gServer.arg("theta"));
    gConfigDirty = true;
    gServer.send(200, "application/json", paramsJson());
  });
  gServer.on("/run", HTTP_GET, []() {
    const int state = gServer.hasArg("state") ? gServer.arg("state").toInt() : 0;
    const bool wantRun = (state != 0);
    if (wantRun && !gRunning) {
      resetRun();
    }
    gRunning = wantRun;
    gConfigDirty = true;
    gServer.send(200, "application/json", paramsJson());
  });
  gServer.on("/clear", HTTP_GET, []() {
    gRunning = false;
    resetRun();
    gConfigDirty = true;
    gServer.send(200, "application/json", paramsJson());
  });
  gServer.on("/data", HTTP_GET, []() {
    sendDataBinary();
  });
  gServer.on("/csv", HTTP_GET, []() {
    gServer.sendHeader("Content-Disposition",
                       "attachment; filename=piloto_pi_nao_linear.csv");
#if defined(CONTENT_LENGTH_UNKNOWN)
    gServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
#else
    gServer.setContentLength((size_t)-1);
#endif
    gServer.send(200, "text/csv", "");
    gServer.sendContent(
        "time_s,reference_mps,speed_mps,fx_N,theta_deg,error_mps,"
        "speed_true_mps\n");
    char line[144];
    for (int i = 0; i < gLogCount; ++i) {
      const double t = gLogT0 + static_cast<double>(i) * gLogDt;
      const float e = gLogR[i] - gLogU[i];
      const int n = snprintf(line, sizeof(line),
                             "%.4f,%.4f,%.4f,%.2f,%.4f,%.4f,%.4f\n", t,
                             gLogR[i], gLogU[i], gLogFx[i], gLogTheta[i], e,
                             gLogUTrue[i]);
      if (n > 0) gServer.sendContent(line, static_cast<size_t>(n));
    }
  });
  gServer.begin();

  xTaskCreatePinnedToCore(plantTask, "plant", 8192, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(webTask, "web", 8192, nullptr, 1, nullptr, 0);

  Serial.println("PLANTA (nao linear) pronta. Fx/u via UART2 (GPIO16/17).");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
