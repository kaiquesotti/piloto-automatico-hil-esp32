/*
 * =============================================================================
 *  controle.ino  --  ESP32 do CONTROLE (Piloto automatico - MODELO NAO LINEAR)
 * =============================================================================
 *
 *  Reproduz o CONTROLADOR do modelo Simulink "nao_linear_piloto_automatico":
 *
 *      e = r - u
 *      u_pid = Kp*e + I + D*N*(e - f)      (PID paralelo com filtro derivativo)
 *      I <- I + Ki*Ts*e
 *      f <- f + Ts*N*(e - f)
 *
 *  A saida do PID e o comando u_pid da valvula borboleta, enviado a planta.
 *  A planta aplica o atuador Ka/(TauA*s + 1) (mesmo Transfer Fcn do Simulink),
 *  integra a dinamica nao linear e devolve a velocidade medida u [m/s].
 *
 *  NAO tem Wi-Fi nem pagina web: recebe configuracao e velocidade por FIOS
 *  (UART2) e devolve Fx. Tempo real, taxa Ts.
 *
 *  PINOS DO ENLACE (UART2) -- cruzar com a planta:
 *    CONTROLE GPIO17 (TX2) ----> PLANTA   GPIO16 (RX2)
 *    CONTROLE GPIO16 (RX2) <---- PLANTA   GPIO17 (TX2)
 *    CONTROLE GND          <--> PLANTA   GND
 *    (NAO interligar os barramentos de 3,3 V)
 *
 *  IDIOMA: comentarios/documentacao em portugues; identificadores em ingles.
 * =============================================================================
 */

#include <Arduino.h>

#include <cmath>
#include <cstdint>
#include <cstring>

/* ===========================================================================
 *  Enlace e protocolo (identico ao planta.ino)
 * ===========================================================================
 */
namespace config {
constexpr int kLinkRxPin = 16;
constexpr int kLinkTxPin = 17;
constexpr uint32_t kLinkBaud = 460800;

constexpr uint8_t kStartByte = 0xAA;
constexpr uint8_t kMsgConfig = 0x01;  // planta -> controle
constexpr uint8_t kMsgU = 0x02;       // controle -> planta (Fx)
constexpr uint8_t kMsgY = 0x03;       // planta -> controle (velocidade u)

constexpr uint32_t kDebugBaud = 115200;
}  // namespace config

/* ===========================================================================
 *  Tipos
 * ===========================================================================
 */
enum RxState : uint8_t {
  WAIT_START,
  READ_TYPE,
  READ_LEN,
  READ_PAYLOAD,
  READ_CHECKSUM,
};

// Configuracao recebida da planta (PID + referencia + taxa + run).
struct ControllerParams {
  double kp = 477.204600781084;
  double ki = 42.1664078975366;
  double kd = 178.763139862948;
  double nf = 1.8234120333855;  // coeficiente do filtro derivativo
  double ts = 0.01;
  double refInitial = 0.0;   // m/s
  double refFinal = 25.0;    // m/s (90 km/h)
  double refTime = 0.0;      // s
  bool run = false;
};

/* ===========================================================================
 *  Estado global
 * ===========================================================================
 */
static ControllerParams gParams;

static bool gPrevRun = false;
static bool gRun = false;
static bool gYValid = false;   // so controla depois de receber a 1a velocidade
static volatile bool gNewY = false;

static double gTime = 0.0;
static double gY = 0.0;        // velocidade u recebida da planta [m/s]
static double gIntegral = 0.0; // estado I do PID
static double gFilteredError = 0.0;  // estado f do filtro derivativo
static double gFx = 0.0;       // comando u_pid da valvula [N]

// Parser do enlace
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
static double stepValue(double initial, double finalValue, double timeS,
                        double t) {
  return (t >= timeS) ? finalValue : initial;
}

static float unpackFloat(const uint8_t* buffer, int index) {
  float value = 0.0F;
  memcpy(&value, &buffer[index * 4], sizeof(float));
  return value;
}

static void packFloat(uint8_t* buffer, int index, float value) {
  memcpy(&buffer[index * 4], &value, sizeof(float));
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

static void handleFrame(uint8_t type, const uint8_t* payload, uint8_t length) {
  if (type == config::kMsgConfig && length >= 33) {
    gParams.kp = unpackFloat(payload, 0);
    gParams.ki = unpackFloat(payload, 1);
    gParams.kd = unpackFloat(payload, 2);
    gParams.nf = unpackFloat(payload, 3);
    const float ts = unpackFloat(payload, 4);
    if (ts > 1e-4F) {
      gParams.ts = ts;
    }
    gParams.refInitial = unpackFloat(payload, 5);
    gParams.refFinal = unpackFloat(payload, 6);
    gParams.refTime = unpackFloat(payload, 7);
    gParams.run = (payload[32] != 0);
  } else if (type == config::kMsgY && length >= 4) {
    gY = unpackFloat(payload, 0);
    gNewY = true;
  }
}

static void pollLink() {
  while (Serial2.available() > 0) {
    const uint8_t byteValue = static_cast<uint8_t>(Serial2.read());
    switch (gRxState) {
      case WAIT_START:
        if (byteValue == config::kStartByte) {
          gRxState = READ_TYPE;
        }
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
        if (gRxIndex >= gRxLen) {
          gRxState = READ_CHECKSUM;
        }
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
 *  Tarefa de controle (tempo real): nucleo 1
 * ===========================================================================
 */
static void controlTask(void*) {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    const double ts = (gParams.ts > 1e-4) ? gParams.ts : 0.01;
    const TickType_t periodTicks =
        pdMS_TO_TICKS(static_cast<uint32_t>(ts * 1000.0));
    vTaskDelayUntil(&lastWake, (periodTicks > 0) ? periodTicks : 1);

    pollLink();
    if (gNewY) {
      gNewY = false;
      gYValid = true;
    }

    // Borda de subida do run: reinicia estados do PID.
    if (gParams.run && !gPrevRun) {
      gIntegral = 0.0;
      gFilteredError = 0.0;
      gFx = 0.0;
      gTime = 0.0;
      gYValid = false;  // espera a primeira velocidade da planta
    }
    gPrevRun = gParams.run;
    gRun = gParams.run;

    if (!gRun || !gYValid) {
      continue;
    }

    const double r = stepValue(gParams.refInitial, gParams.refFinal,
                               gParams.refTime, gTime);
    const double error = r - gY;

    // PID paralelo com filtro derivativo (Forward Euler), como no Simulink.
    const double pTerm = gParams.kp * error;
    const double dTerm =
        gParams.kd * gParams.nf * (error - gFilteredError);
    gFx = pTerm + gIntegral + dTerm;
    gIntegral += gParams.ki * ts * error;
    gFilteredError += ts * gParams.nf * (error - gFilteredError);

    uint8_t payload[4];
    packFloat(payload, 0, static_cast<float>(gFx));
    sendFrame(config::kMsgU, payload, sizeof(payload));

    // Telemetria USB de backup: t, ref(m/s), u(m/s), comando u_pid(N), e(m/s)
    Serial.printf("%.4f,%.4f,%.4f,%.2f,%.4f\n", gTime, r, gY, gFx, error);

    gTime += ts;
  }
}

/* ===========================================================================
 *  setup() / loop()
 * ===========================================================================
 */
void setup() {
  Serial.begin(config::kDebugBaud);
  delay(200);
  Serial.println("t_s,reference_mps,speed_mps,fx_cmd_N,error_mps");

  Serial2.setRxBufferSize(1024);
  Serial2.begin(config::kLinkBaud, SERIAL_8N1, config::kLinkRxPin,
                config::kLinkTxPin);

  xTaskCreatePinnedToCore(controlTask, "control", 8192, nullptr, 3, nullptr, 1);

  Serial.println("CONTROLE (nao linear) pronto. Fx/u via UART2 (GPIO16/17).");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
