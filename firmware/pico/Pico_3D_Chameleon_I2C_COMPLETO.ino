/* ═══════════════════════════════════════════════════════════════════════════
   Firmware 3DChameleon Mk4 — Raspberry Pi Pico + 2x TMC2209 Standalone
   Comunicação K1 Max : USB Serial nativo — 115200 baud
   Comunicação ESP32  : I2C Slave — GPIO 4/5

   Pinout:
   ── Sensores Entrada de Filamento (NC) ──
     T0 entrada : GPIO 0  (Pino 1)
     T1 entrada : GPIO 1  (Pino 2)
     T2 entrada : GPIO 2  (Pino 4)
     T3 entrada : GPIO 3  (Pino 5)

   ── Sensor Saida Unico (NC) — compartilhado T0-T3 ──
     Saida      : GPIO 12 (Pino 16)

   ── Endstop Seletor (NC) ──
     Endstop    : GPIO 8  (Pino 11)

   ── Sensor Hotend (NC) ──
     Hotend     : GPIO 9  (Pino 12)

   ── Sensores Buffer (NC) ──
     Buffer Vazio : GPIO 10 (Pino 14)
     Buffer Cheio : GPIO 11 (Pino 15)

   ── Motor Seletor (TMC2209 standalone) ──
     STEP : GPIO 16 (Pino 21)
     DIR  : GPIO 17 (Pino 22)
     EN   : GPIO 18 (Pino 24)

   ── Motor Alimentador (TMC2209 standalone) ──
     STEP : GPIO 19 (Pino 25)
     DIR  : GPIO 20 (Pino 26)
     EN   : GPIO 21 (Pino 27)

   ── I2C Slave — ESP32 ──
     SDA : GPIO 4  (Pino 6)
     SCL : GPIO 5  (Pino 7)
     Endereço: 0x12

   Protocolo I2C:
     ESP → Pico : pacote 4 bytes [0xAA][CMD][ARG][CHK]
     Pico -> ESP : pacote 16 bytes [0xBB][ST][T][FILS][BUF][HE][FLAGS][DIST][FAST][RAMP][RAMP_MM][CHK]

   Comandos Serial USB (115200 baud) — K1 Max:
     T0-T3 HOME IDLE LOAD LOAD<mm> UNLOAD UNLOAD_RETRACAO<mm>
     BUFFER_DRAIN START_PRINT STOP_PRINT STATUS
═══════════════════════════════════════════════════════════════════════════ */

#include <EEPROM.h>
#include <Wire.h>

// ── I2C Slave — ESP32 ─────────────────────────────────
#define I2C_SDA   4
#define I2C_SCL   5
#define I2C_ADDR  0x12

#define CMD_PKT_SIZE  4
#define ST_PKT_SIZE   16

#define PKT_CMD   0xAA  // ESP → Pico
#define PKT_ST    0xBB  // Pico → ESP

#define I2C_CMD_T0          0x01
#define I2C_CMD_T1          0x02
#define I2C_CMD_T2          0x03
#define I2C_CMD_T3          0x04
#define I2C_CMD_LOAD        0x05
#define I2C_CMD_UNLOAD      0x06
#define I2C_CMD_HOME        0x07
#define I2C_CMD_START       0x08
#define I2C_CMD_STOP        0x09
#define I2C_CMD_STATUS      0x0A
#define I2C_CMD_CALIBRAR    0x70
#define I2C_CMD_SPD_FAST_UP 0x80
#define I2C_CMD_SPD_FAST_DN 0x81
#define I2C_CMD_SPD_RAMP_UP 0x82
#define I2C_CMD_SPD_RAMP_DN 0x83
#define I2C_CMD_RAMP_MM_UP  0x84
#define I2C_CMD_RAMP_MM_DN  0x85
#define I2C_CMD_DIST_UP     0x86
#define I2C_CMD_DIST_DN     0x87
#define I2C_CMD_RESET_CONFIG 0x88

#define STA_IDLE     0x00
#define STA_BUSY     0x01
#define STA_PRINTING 0x02
#define STA_ERROR    0x03

volatile bool    i2cCmdReady = false;
volatile uint8_t i2cLastCmd  = 0;
volatile uint8_t i2cLastArg  = 0;
volatile uint8_t i2cStatus   = STA_IDLE;
bool             lastOperationOk = true;
bool             unloadSensorError = false;

// ── Sensores Entrada de Filamento (NC) ────────────────
#define SENSOR_ENTRADA_T0   0
#define SENSOR_ENTRADA_T1   1
#define SENSOR_ENTRADA_T2   2
#define SENSOR_ENTRADA_T3   3

// ── Sensor Saida Unico (NC) ───────────────────────────
#define SENSOR_SAIDA        12

// ── Endstop Seletor ───────────────────────────────────
#define ENDSTOP_SELETOR     8

// ── Sensor Hotend (NC) ────────────────────────────────
#define SENSOR_HOTEND       9

// ── Buffer (NC) ───────────────────────────────────────
#define BUFFER_VAZIO_PIN    10
#define BUFFER_CHEIO_PIN    11

// ── Motor Seletor ─────────────────────────────────────
#define SEL_STEP            16
#define SEL_DIR             17
#define SEL_EN              18

// ── Motor Alimentador ─────────────────────────────────
#define EXT_STEP            19
#define EXT_DIR             20
#define EXT_EN              21

// ── Configuração motores ──────────────────────────────
const int   MICROSTEPS       = 16;
const int   STEPS_PER_REV    = 200;
const float STEPS_PER_MM     = 428.0;
const int   SPEED_DELAY      = 60;
const int   SPEED_DELAY_FAST = 28;
const int   SPEED_DELAY_RAMP = 23;
const long  RAMP_STEPS       = (long)(60.0 * STEPS_PER_MM);
const int   SEL_SPEED_DELAY  = 60;
const int   DEFAULT_BACKOFF  = 5;
const float LOAD_LIMIT_MARGIN_MM = 50.0;
const float UNLOAD_AFTER_HUB_MM = 40.0;
const float PRELOAD_RETRACT_MM = 40.0;
const float PRELOAD_MAX_MM = 300.0;
const int   RAMP_SPEED_MIN_US  = 10;
const int   RAMP_SPEED_MAX_US  = 5000;
const float CONFIG_SPEED_MAX_MMS = 70.0;
const int   CONFIG_SPEED_MIN_PERCENT = 1;
const int   CONFIG_SPEED_MAX_PERCENT = 100;

const bool  CW  = HIGH;
const bool  CCW = LOW;

// ── Estado global ─────────────────────────────────────
int  currentExtruder  = 0;
int  lastExtruder     = 0;
bool imprimindo       = false;
bool bufferDrainMode  = false;

// ── Buffer ────────────────────────────────────────────
unsigned long lastBufferCheck = 0;
const int     BUFFER_CHECK_MS = 100;
bool          lastBufferVazio = false;
bool          lastBufferCheio = false;
bool          lastEntradaPreload[4] = {false, false, false, false};
unsigned long lastPreloadCheck = 0;
const int     PRELOAD_CHECK_MS = 50;

// ── Serial USB ────────────────────────────────────────
String serialBuffer    = "";
bool   commandReceived = false;

// ── EEPROM ────────────────────────────────────────────
#define EEPROM_TOOL_ADDR      0   // 1 byte  — tool ativa
#define EEPROM_DIST_ADDR      1   // 4 bytes — distancia calibrada (float mm)
#define EEPROM_SPD_FAST_ADDR  5   // 4 bytes - velocidade load rapido (int us)
#define EEPROM_SPD_RAMP_ADDR  9   // 4 bytes - velocidade load rampa (int us)
#define EEPROM_RAMP_MM_ADDR  13   // 4 bytes - mm antes do sensor que inicia rampa
#define EEPROM_MAGIC_ADDR    17   // 1 byte  - 0xCF = configuracao valida
#define EEPROM_LOADED_ADDR   18   // 1 byte  - tool carregada + 1; zero = nenhuma

#define EEPROM_MAGIC_VAL   0xCF
#define EEPROM_SIZE        64

// Configuracoes carregadas da EEPROM
float configDistMax   = 4000.0;  // mm maximo de load — padrao 4000mm
int   configSpdFast   = 28;      // us — velocidade rapida load
int   configSpdRamp   = 1325;    // us — velocidade rampa final (F300)
int   configSpdFastPercent = 0;
int   configSpdRampPercent = 0;
float configRampMM    = 60.0;    // mm antes do sensor que inicia rampa
bool  configCalibrado = false;   // true se ja foi calibrado

float distanciaLimiteLoadMM() {
  return configDistMax + LOAD_LIMIT_MARGIN_MM;
}

long distanciaLimiteLoadSteps() {
  return (long)(distanciaLimiteLoadMM() * STEPS_PER_MM);
}

// ── I2C: comunicação opcional com ESP32/display ──────

// ═══════════════════════════════════════════════════════
// UTILITÁRIOS
// ═══════════════════════════════════════════════════════

int sensorEntradaPin(int tool) {
  switch (tool) {
    case 0: return SENSOR_ENTRADA_T0;
    case 1: return SENSOR_ENTRADA_T1;
    case 2: return SENSOR_ENTRADA_T2;
    case 3: return SENSOR_ENTRADA_T3;
    default: return -1;
  }
}

int sensorSaidaPin(int tool) { return SENSOR_SAIDA; }

bool filamentoPresente(int pin) { return digitalRead(pin) == HIGH; }
bool bufferVazio()  { return digitalRead(BUFFER_VAZIO_PIN) == HIGH; }
bool bufferCheio()  { return digitalRead(BUFFER_CHEIO_PIN) == HIGH; }

bool direcaoLoad(int tool)   { return (tool < 2) ? CCW : CW;  }
bool direcaoUnload(int tool) { return (tool < 2) ? CW  : CCW; }

enum {
  LOAD_PODE_INICIAR,
  LOAD_JA_CARREGADO,
  LOAD_BLOQUEADO
};

int verificarInicioLoad(int tool) {
  int loadedTool = carregarToolCarregada();
  bool hub = filamentoPresente(SENSOR_SAIDA);
  bool hotend = filamentoPresente(SENSOR_HOTEND);

  if (loadedTool >= 0 && loadedTool != tool) {
    Serial.print("ERRO: LOAD T"); Serial.print(tool);
    Serial.print(" bloqueado: T"); Serial.print(loadedTool);
    Serial.println(" ainda esta carregado.");
    lastOperationOk = false;
    return LOAD_BLOQUEADO;
  }

  if (loadedTool == tool && hub && hotend) {
    lastOperationOk = true;
    return LOAD_JA_CARREGADO;
  }

  if (loadedTool == tool) {
    Serial.print("ERRO: LOAD T"); Serial.print(tool);
    Serial.println(" bloqueado: EEPROM indica carregado, mas sensores nao confirmam.");
    lastOperationOk = false;
    return LOAD_BLOQUEADO;
  }

  if (hub) {
    Serial.println("ERRO: LOAD bloqueado: filamento no Hub, mas nao no Hotend.");
    lastOperationOk = false;
    return LOAD_BLOQUEADO;
  }

  if (hotend) {
    Serial.println("ERRO: LOAD bloqueado: Hotend detecta filamento, mas Hub nao.");
    lastOperationOk = false;
    return LOAD_BLOQUEADO;
  }

  return LOAD_PODE_INICIAR;
}

void salvarTool(int tool) {
  EEPROM.write(EEPROM_TOOL_ADDR, tool + 1);
  EEPROM.commit();
}
int  carregarTool()       { return EEPROM.read(EEPROM_TOOL_ADDR) - 1; }

void salvarToolCarregada(int tool) {
  EEPROM.write(EEPROM_LOADED_ADDR, (tool >= 0 && tool <= 3) ? tool + 1 : 0);
  EEPROM.commit();
}

int carregarToolCarregada() {
  int tool = EEPROM.read(EEPROM_LOADED_ADDR) - 1;
  return (tool >= 0 && tool <= 3) ? tool : -1;
}

float velocidadeMMS(int delayUs) {
  if (delayUs <= 0) return 0.0;
  return 1000000.0 / (2.0 * delayUs * STEPS_PER_MM);
}

void printVelocidadeMMS(int delayUs) {
  float mmS = velocidadeMMS(delayUs);
  Serial.print((int)(mmS + 0.5));
  Serial.print("mm/s");
}

int velocidadePercentual(int delayUs) {
  int percent = (int)((velocidadeMMS(delayUs) * 100.0 / CONFIG_SPEED_MAX_MMS) + 0.5);
  if (percent < CONFIG_SPEED_MIN_PERCENT) percent = CONFIG_SPEED_MIN_PERCENT;
  if (percent > CONFIG_SPEED_MAX_PERCENT) percent = CONFIG_SPEED_MAX_PERCENT;
  return percent;
}

void printVelocidadePercentual(int delayUs) {
  Serial.print(velocidadePercentual(delayUs));
  Serial.print("%");
}

int delayUsParaVelocidadeMMS(float mmS) {
  if (mmS < 0.1) mmS = 0.1;
  int delayUs = (int)(1000000.0 / (2.0 * mmS * STEPS_PER_MM) + 0.5);
  if (delayUs < RAMP_SPEED_MIN_US) delayUs = RAMP_SPEED_MIN_US;
  if (delayUs > RAMP_SPEED_MAX_US) delayUs = RAMP_SPEED_MAX_US;
  return delayUs;
}

int delayUsParaPercentual(int percent) {
  if (percent < CONFIG_SPEED_MIN_PERCENT) percent = CONFIG_SPEED_MIN_PERCENT;
  if (percent > CONFIG_SPEED_MAX_PERCENT) percent = CONFIG_SPEED_MAX_PERCENT;
  float mmS = CONFIG_SPEED_MAX_MMS * ((float)percent / 100.0);
  return delayUsParaVelocidadeMMS(mmS);
}

int limitarPercentualVelocidade(int percent) {
  if (percent < CONFIG_SPEED_MIN_PERCENT) return CONFIG_SPEED_MIN_PERCENT;
  if (percent > CONFIG_SPEED_MAX_PERCENT) return CONFIG_SPEED_MAX_PERCENT;
  return percent;
}

void aplicarVelocidadeRapidaPercentual(int percent) {
  configSpdFastPercent = limitarPercentualVelocidade(percent);
  configSpdFast = delayUsParaPercentual(configSpdFastPercent);
}

void aplicarVelocidadeRampaPercentual(int percent) {
  configSpdRampPercent = limitarPercentualVelocidade(percent);
  configSpdRamp = delayUsParaPercentual(configSpdRampPercent);
}

int ajustarVelocidadePercentual(int delayUs, int deltaPercent) {
  return delayUsParaPercentual(velocidadePercentual(delayUs) + deltaPercent);
}

void resetarConfig() {
  configDistMax   = 4000.0;
  configSpdFast   = 28;
  configSpdRamp   = 1325;
  configSpdFastPercent = velocidadePercentual(configSpdFast);
  configSpdRampPercent = velocidadePercentual(configSpdRamp);
  configRampMM    = 60.0;
  configCalibrado = false;

  EEPROM.put(EEPROM_DIST_ADDR, configDistMax);
  EEPROM.put(EEPROM_SPD_FAST_ADDR, configSpdFast);
  EEPROM.put(EEPROM_SPD_RAMP_ADDR, configSpdRamp);
  EEPROM.put(EEPROM_RAMP_MM_ADDR, configRampMM);
  EEPROM.write(EEPROM_MAGIC_ADDR, 0x00);
  EEPROM.commit();

  Serial.println("Config de calibracao resetada.");
}

// ── Salva/Carrega configuracoes ───────────────────────

void salvarConfig() {
  // distancia max
  EEPROM.put(EEPROM_DIST_ADDR, configDistMax);
  // velocidade rapida
  EEPROM.put(EEPROM_SPD_FAST_ADDR, configSpdFast);
  // velocidade rampa
  EEPROM.put(EEPROM_SPD_RAMP_ADDR, configSpdRamp);
  // mm rampa
  EEPROM.put(EEPROM_RAMP_MM_ADDR, configRampMM);
  // magic
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VAL);
  EEPROM.commit();
  Serial.println("Config salva na EEPROM.");
}

void carregarConfig() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC_VAL) {
    Serial.println("Config EEPROM nao encontrada — usando padroes.");
    configSpdFastPercent = velocidadePercentual(configSpdFast);
    configSpdRampPercent = velocidadePercentual(configSpdRamp);
    configCalibrado = false;
    return;
  }
  EEPROM.get(EEPROM_DIST_ADDR,     configDistMax);
  EEPROM.get(EEPROM_SPD_FAST_ADDR, configSpdFast);
  EEPROM.get(EEPROM_SPD_RAMP_ADDR, configSpdRamp);
  EEPROM.get(EEPROM_RAMP_MM_ADDR,  configRampMM);
  configSpdFastPercent = velocidadePercentual(configSpdFast);
  configSpdRampPercent = velocidadePercentual(configSpdRamp);
  configCalibrado = true;
  Serial.print("Config carregada: dist="); Serial.print(configDistMax);
  Serial.print("mm spdFast="); printVelocidadePercentual(configSpdFast);
  Serial.print(" spdRamp="); printVelocidadePercentual(configSpdRamp);
  Serial.print(" rampMM="); Serial.print(configRampMM);
  Serial.println("mm");
}

// ── Calibracao de Load ────────────────────────────────
// Roda load completo sem filamento carregado
// Mede distancia ate sensor hotend
// Salva como configDistMax na EEPROM

void calibrarLoad() {
  Serial.println("Calibracao iniciando...");
  lastOperationOk = true;

  // Seguranca — verifica sensores
  if (filamentoPresente(SENSOR_SAIDA)) {
    Serial.println("ERRO: Remova filamento do Hub antes de calibrar!");
    lastOperationOk = false;
    return;
  }
  if (filamentoPresente(SENSOR_HOTEND)) {
    Serial.println("ERRO: Remova filamento do Hotend antes de calibrar!");
    lastOperationOk = false;
    return;
  }

  digitalWrite(EXT_EN, LOW);
  digitalWrite(EXT_DIR, direcaoLoad(currentExtruder));

  long steps    = 0;
  long maxSteps = (long)(6000.0 * STEPS_PER_MM);  // limite de seguranca 6000mm
  bool ok       = false;

  while (steps < maxSteps) {
    if (filamentoPresente(SENSOR_HOTEND)) { ok = true; break; }
    extStep(configSpdFast);
    steps++;
  }

  digitalWrite(EXT_EN, HIGH);

  if (!ok) {
    Serial.println("ERRO: Sensor hotend nao acionou durante calibracao!");
    lastOperationOk = false;
    return;
  }

  float distMM = (float)steps / STEPS_PER_MM;

  // Salva a distancia real medida; a margem fica separada no limite dos movimentos.
  configDistMax   = distMM;
  configCalibrado = true;
  salvarConfig();

  Serial.print("Calibracao OK: ");
  Serial.print(distMM);
  Serial.print("mm - distancia salva: ");
  Serial.print(configDistMax);
  Serial.println("mm");
}

int velocidadeComRampa(long steps) {
  float inicioRampaMM = configDistMax - configRampMM;
  if (inicioRampaMM < 0) inicioRampaMM = 0;

  long inicioRampaSteps = (long)(inicioRampaMM * STEPS_PER_MM);
  return (steps < inicioRampaSteps) ? configSpdFast : configSpdRamp;
}

void extStep(int spd) {
  digitalWrite(EXT_STEP, HIGH); delayMicroseconds(spd);
  digitalWrite(EXT_STEP, LOW);  delayMicroseconds(spd);
}

void pararAlimentadorSemDestravar() {
  // Durante impressao, para de dar passos mas mantem o TMC energizado.
  digitalWrite(EXT_EN, imprimindo ? LOW : HIGH);
}

// ═══════════════════════════════════════════════════════
// I2C SLAVE — ESP32/DISPLAY OPCIONAL
// ═══════════════════════════════════════════════════════

uint8_t checksumBuf(const uint8_t* buf, int len) {
  uint8_t chk = 0;
  for (int i = 0; i < len; i++) chk ^= buf[i];
  return chk;
}

uint8_t estadoAtualI2C() {
  if (i2cStatus == STA_ERROR) return STA_ERROR;
  if (i2cStatus == STA_BUSY)  return STA_BUSY;
  if (imprimindo) return STA_PRINTING;
  return STA_IDLE;
}

void i2cReceiveEvent(int len) {
  uint8_t buf[CMD_PKT_SIZE] = {0};
  int i = 0;

  while (Wire.available() && i < CMD_PKT_SIZE) {
    buf[i++] = Wire.read();
  }
  while (Wire.available()) Wire.read();

  if (i != CMD_PKT_SIZE) return;
  if (buf[0] != PKT_CMD) return;
  if (checksumBuf(buf, CMD_PKT_SIZE - 1) != buf[CMD_PKT_SIZE - 1]) return;

  i2cLastCmd  = buf[1];
  i2cLastArg  = buf[2];
  i2cCmdReady = true;
}

void i2cRequestEvent() {
  uint8_t tx[ST_PKT_SIZE] = {0};

  uint8_t fils = 0;
  for (int i = 0; i < 4; i++) {
    if (filamentoPresente(sensorEntradaPin(i))) fils |= (1 << i);
  }

  tx[0] = PKT_ST;
  tx[1] = estadoAtualI2C();
  tx[2] = (uint8_t)(currentExtruder + 1);  // ESP espera 1 a 4
  tx[3] = fils;
  tx[4] = (bufferCheio() ? 0x01 : 0x00) | (filamentoPresente(SENSOR_SAIDA) ? 0x02 : 0x00);
  tx[5] = filamentoPresente(SENSOR_HOTEND) ? 1 : 0;
  tx[6] = (imprimindo ? 0x01 : 0x00) |
          (configCalibrado ? 0x02 : 0x00) |
          (unloadSensorError ? 0x04 : 0x00);

  uint16_t distMaxMM = (uint16_t)constrain((int)configDistMax, 0, 65535);
  uint16_t rampMM    = (uint16_t)constrain((int)configRampMM, 0, 65535);
  uint16_t spdFast   = (uint16_t)constrain(configSpdFast, 0, 65535);
  uint16_t spdRamp   = (uint16_t)constrain(configSpdRamp, 0, 65535);

  tx[7]  = lowByte(distMaxMM);
  tx[8]  = highByte(distMaxMM);
  tx[9]  = lowByte(spdFast);
  tx[10] = highByte(spdFast);
  tx[11] = lowByte(spdRamp);
  tx[12] = highByte(spdRamp);
  tx[13] = lowByte(rampMM);
  tx[14] = highByte(rampMM);
  tx[15] = checksumBuf(tx, ST_PKT_SIZE - 1);

  Wire.write(tx, ST_PKT_SIZE);
}

void processarComandoI2C(uint8_t cmd, uint8_t arg) {
  i2cStatus = STA_BUSY;
  lastOperationOk = true;

  switch (cmd) {
    case I2C_CMD_T0:          processarComando("T0");          break;
    case I2C_CMD_T1:          processarComando("T1");          break;
    case I2C_CMD_T2:          processarComando("T2");          break;
    case I2C_CMD_T3:          processarComando("T3");          break;
    case I2C_CMD_LOAD:        processarComando("LOAD");        break;
    case I2C_CMD_UNLOAD:      processarComando("UNLOAD");      break;
    case I2C_CMD_HOME:        processarComando("HOME");        break;
    case I2C_CMD_START:       processarComando("START_PRINT"); break;
    case I2C_CMD_STOP:        processarComando("STOP_PRINT");  break;
    case I2C_CMD_STATUS:      break;
    case I2C_CMD_CALIBRAR:    calibrarLoad();                  break;

    case I2C_CMD_SPD_FAST_UP:
      aplicarVelocidadeRapidaPercentual(configSpdFastPercent + 1);
      salvarConfig();
      Serial.print("Vel rapida: "); printVelocidadePercentual(configSpdFast); Serial.println();
      break;

    case I2C_CMD_SPD_FAST_DN:
      aplicarVelocidadeRapidaPercentual(configSpdFastPercent - 1);
      salvarConfig();
      Serial.print("Vel rapida: "); printVelocidadePercentual(configSpdFast); Serial.println();
      break;

    case I2C_CMD_SPD_RAMP_UP:
      aplicarVelocidadeRampaPercentual(configSpdRampPercent + 1);
      salvarConfig();
      Serial.print("Vel rampa: "); printVelocidadePercentual(configSpdRamp); Serial.println();
      break;

    case I2C_CMD_SPD_RAMP_DN:
      aplicarVelocidadeRampaPercentual(configSpdRampPercent - 1);
      salvarConfig();
      Serial.print("Vel rampa: "); printVelocidadePercentual(configSpdRamp); Serial.println();
      break;

    case I2C_CMD_RAMP_MM_UP:
      configRampMM = min(6000.0, configRampMM + 5.0);
      salvarConfig();
      Serial.print("Rampa mm: "); Serial.println(configRampMM);
      break;

    case I2C_CMD_RAMP_MM_DN:
      configRampMM = max(10.0, configRampMM - 5.0);
      salvarConfig();
      Serial.print("Rampa mm: "); Serial.println(configRampMM);
      break;

    case I2C_CMD_DIST_UP:
      configDistMax = min(6000.0, configDistMax + 50.0);
      salvarConfig();
      Serial.print("Dist max: "); Serial.println(configDistMax);
      break;

    case I2C_CMD_DIST_DN:
      configDistMax = max(100.0, configDistMax - 50.0);
      salvarConfig();
      Serial.print("Dist max: "); Serial.println(configDistMax);
      break;

    case I2C_CMD_RESET_CONFIG:
      resetarConfig();
      break;

    default:
      Serial.print("I2C: comando desconhecido 0x");
      Serial.println(cmd, HEX);
      i2cStatus = STA_ERROR;
      return;
  }

  if (!lastOperationOk) {
    i2cStatus = STA_ERROR;
    return;
  }

  i2cStatus = imprimindo ? STA_PRINTING : STA_IDLE;
}

// ═══════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("3DChameleon Pico Ready");
  EEPROM.begin(EEPROM_SIZE);

  // I2C Slave — ESP32/display.
  // Não bloqueia: se o ESP32 estiver desligado, o Pico continua iniciando normal.
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin(I2C_ADDR);
  Wire.onReceive(i2cReceiveEvent);
  Wire.onRequest(i2cRequestEvent);

  // Motores
  pinMode(SEL_STEP, OUTPUT); pinMode(SEL_DIR, OUTPUT); pinMode(SEL_EN, OUTPUT);
  pinMode(EXT_STEP, OUTPUT); pinMode(EXT_DIR, OUTPUT); pinMode(EXT_EN, OUTPUT);
  digitalWrite(SEL_EN, HIGH);
  digitalWrite(EXT_EN, HIGH);

  // Sensores
  pinMode(SENSOR_ENTRADA_T0, INPUT_PULLUP);
  pinMode(SENSOR_ENTRADA_T1, INPUT_PULLUP);
  pinMode(SENSOR_ENTRADA_T2, INPUT_PULLUP);
  pinMode(SENSOR_ENTRADA_T3, INPUT_PULLUP);
  pinMode(SENSOR_SAIDA,      INPUT_PULLUP);
  pinMode(ENDSTOP_SELETOR,   INPUT_PULLUP);
  pinMode(SENSOR_HOTEND,     INPUT_PULLUP);
  pinMode(BUFFER_VAZIO_PIN,  INPUT_PULLUP);
  pinMode(BUFFER_CHEIO_PIN,  INPUT_PULLUP);

  lastBufferVazio = bufferVazio();
  lastBufferCheio = bufferCheio();
  for (int t = 0; t < 4; t++) {
    lastEntradaPreload[t] = filamentoPresente(sensorEntradaPin(t));
  }

  // Carrega configuracoes da EEPROM
  carregarConfig();

  homeSelector();

  int saved = carregarTool();
  if (saved >= 0 && saved <= 3) {
    currentExtruder = saved;
    lastExtruder    = saved;
    moverSeletor(0, currentExtruder);
    Serial.print("Tool restaurada: T"); Serial.println(currentExtruder);
  } else {
    currentExtruder = 0;
    lastExtruder    = 0;
    salvarTool(0);
    Serial.println("Tool default: T0");
  }

  for (int t = 0; t < 4; t++) {
    Serial.print("T"); Serial.print(t);
    Serial.print(" entrada: ");
    Serial.println(filamentoPresente(sensorEntradaPin(t)) ? "Filamento OK" : "Vazio");
  }

  Serial.println("\nCmds: T0-T3 HOME IDLE LOAD [mm] UNLOAD UNLOAD_RETRACAO <mm> BUFFER_DRAIN START_PRINT STOP_PRINT STATUS\n");
}

// ═══════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════

void loop() {

  // ── I2C — comando opcional vindo do ESP32/display ───
  if (i2cCmdReady) {
    noInterrupts();
    uint8_t cmd = i2cLastCmd;
    uint8_t arg = i2cLastArg;
    i2cCmdReady = false;
    interrupts();

    processarComandoI2C(cmd, arg);
  }

  // ── Serial USB — K1 Max ──────────────────────────────
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) { commandReceived = true; break; }
    } else {
      serialBuffer += c;
    }
  }

  if (commandReceived) {
    Serial.print("CMD: "); Serial.println(serialBuffer);
    processarComando(serialBuffer);
    serialBuffer    = "";
    commandReceived = false;
  }

  // ── Monitoramento buffer ─────────────────────────────
  if (imprimindo && !bufferDrainMode) {
    unsigned long now = millis();
    if (now - lastBufferCheck >= BUFFER_CHECK_MS) {
      monitorarBuffer();
      lastBufferCheck = now;
    }
  }

  // ── Buffer Drain ─────────────────────────────────────
  if (bufferDrainMode) {
    executarBufferDrain();
  }

  if (!imprimindo && !bufferDrainMode) {
    unsigned long now = millis();
    if (now - lastPreloadCheck >= PRELOAD_CHECK_MS) {
      monitorarPreLoad();
      lastPreloadCheck = now;
    }
  }

  delay(5);
}

// ═══════════════════════════════════════════════════════
// PROCESSAR COMANDOS
// ═══════════════════════════════════════════════════════

void processarComando(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  unloadSensorError = false;

  if (cmd == "T0") { selecionarTool(0); return; }
  if (cmd == "T1") { selecionarTool(1); return; }
  if (cmd == "T2") { selecionarTool(2); return; }
  if (cmd == "T3") { selecionarTool(3); return; }

  if (cmd == "HOME") {
    int loadedTool = carregarToolCarregada();
    homeSelector();
    if (loadedTool >= 0 &&
        filamentoPresente(SENSOR_SAIDA) &&
        filamentoPresente(SENSOR_HOTEND)) {
      moverSeletor(0, loadedTool);
      currentExtruder = loadedTool;
      lastExtruder = loadedTool;
      salvarTool(loadedTool);
      Serial.print("HOME: voltou para T"); Serial.println(loadedTool);
    }
    Serial.println("HOME OK");
    return;
  }

  if (cmd == "IDLE") {
    moverIdle();
    return;
  }

  if (cmd == "BUFFER_DRAIN") {
    iniciarBufferDrain();
    return;
  }

  if (cmd == "UNLOAD") {
    if (bufferDrainMode) {
      bufferDrainMode = false;
      Serial.println("Buffer drain desativado");
    }
    imprimindo = false;
    int loadedTool = carregarToolCarregada();
    if (loadedTool < 0 && filamentoPresente(SENSOR_SAIDA)) {
      loadedTool = currentExtruder;
      Serial.print("UNLOAD: usando ferramenta atual pelo sensor do Hub: T");
      Serial.println(loadedTool);
    }
    if (loadedTool < 0) {
      lastOperationOk = false;
      Serial.println("ERRO: UNLOAD bloqueado: nenhuma ferramenta carregada na EEPROM.");
      return;
    }
    if (currentExtruder != loadedTool) {
      moverSeletor(currentExtruder, loadedTool);
      currentExtruder = loadedTool;
      lastExtruder = loadedTool;
      salvarTool(loadedTool);
    }
    unloadAteSensor(loadedTool);
    return;
  }

  if (cmd.startsWith("UNLOAD_RETRACAO ")) {
    float mm = cmd.substring(16).toFloat();
    if (mm <= 0) { Serial.println("ERRO: Distancia invalida"); return; }
    if (currentExtruder < 0) { Serial.println("ERRO: Selecione tool primeiro"); return; }
    unloadRetracao(currentExtruder, mm);
    return;
  }

  if (cmd == "LOAD") {
    loadContinuo(currentExtruder);
    return;
  }

  if (cmd == "LOAD_MANUAL") {
    loadManual(currentExtruder);
    return;
  }

  if (cmd.startsWith("LOAD ")) {
    float mm = cmd.substring(5).toFloat();
    if (mm <= 0) { Serial.println("ERRO: Distancia invalida"); return; }
    loadAteSensorHotend(currentExtruder, mm);
    return;
  }

  if (cmd == "START_PRINT") {
    imprimindo      = true;
    bufferDrainMode = false;
    Serial.println("Print started");
    return;
  }

  if (cmd == "STOP_PRINT") {
    imprimindo = false;
    digitalWrite(SEL_EN, HIGH);
    digitalWrite(EXT_EN, HIGH);
    Serial.println("Print stopped");
    return;
  }

  if (cmd == "STATUS") {
    printStatus();
    return;
  }

  // ── Configuracoes ────────────────────────────────────

  if (cmd == "CALIBRAR") {
    calibrarLoad();
    return;
  }

  if (cmd.startsWith("SET_DIST ")) {
    float v = cmd.substring(9).toFloat();
    if (v > 100.0 && v <= 6000.0) {
      configDistMax = v;
      salvarConfig();
      Serial.print("Dist max: "); Serial.print(configDistMax); Serial.println("mm OK");
    } else { Serial.println("ERRO: Distancia invalida (100-6000mm)"); }
    return;
  }

  if (cmd.startsWith("SET_SPD_FAST ")) {
    int v = cmd.substring(13).toInt();
    if (v >= 10 && v <= 500) {
      configSpdFast = v;
      configSpdFastPercent = velocidadePercentual(configSpdFast);
      salvarConfig();
      Serial.print("Vel rapida: "); printVelocidadePercentual(configSpdFast); Serial.println(" OK");
    } else { Serial.println("ERRO: Velocidade invalida (10-500us)"); }
    return;
  }

  if (cmd.startsWith("SET_SPD_RAMP ")) {
    int v = cmd.substring(13).toInt();
    if (v >= RAMP_SPEED_MIN_US && v <= 5000) {
      configSpdRamp = v;
      configSpdRampPercent = velocidadePercentual(configSpdRamp);
      salvarConfig();
      Serial.print("Vel rampa: "); printVelocidadePercentual(configSpdRamp); Serial.println(" OK");
    } else { Serial.println("ERRO: Velocidade invalida (10-5000us)"); }
    return;
  }

  if (cmd.startsWith("SET_RAMP_MM ")) {
    float v = cmd.substring(12).toFloat();
    if (v >= 10.0 && v <= 6000.0) {
      configRampMM = v;
      salvarConfig();
      Serial.print("Rampa: "); Serial.print(configRampMM); Serial.println("mm OK");
    } else { Serial.println("ERRO: Valor invalido (10-6000mm)"); }
    return;
  }

  if (cmd == "GET_CONFIG") {
    Serial.println("==== CONFIG ====");
    Serial.print("Dist max  : "); Serial.print(configDistMax);   Serial.println("mm");
    Serial.print("Vel rapida: "); printVelocidadePercentual(configSpdFast); Serial.println();
    Serial.print("Vel rampa : "); printVelocidadePercentual(configSpdRamp); Serial.println();
    Serial.print("Rampa mm  : "); Serial.print(configRampMM);    Serial.println("mm");
    Serial.print("Calibrado : "); Serial.println(configCalibrado ? "SIM" : "NAO");
    Serial.println("================");
    return;
  }

  if (cmd == "RESET_CONFIG") {
    resetarConfig();
    return;
  }

  Serial.println("ERRO: Comando invalido");
}

// ═══════════════════════════════════════════════════════
// STATUS
// ═══════════════════════════════════════════════════════

void printStatus() {
  Serial.println("=======================");
  Serial.print("Tool ativa   : T"); Serial.println(currentExtruder);
  Serial.print("Imprimindo   : "); Serial.println(imprimindo ? "SIM" : "NAO");
  Serial.print("Buffer drain : "); Serial.println(bufferDrainMode ? "ATIVO" : "INATIVO");
  Serial.print("Buffer vazio : "); Serial.println(bufferVazio() ? "SIM" : "NAO");
  Serial.print("Buffer cheio : "); Serial.println(bufferCheio() ? "SIM" : "NAO");
  Serial.print("Hotend       : "); Serial.println(filamentoPresente(SENSOR_HOTEND) ? "Filamento OK" : "Sem filamento");
  for (int t = 0; t < 4; t++) {
    Serial.print("T"); Serial.print(t);
    Serial.print(" entrada: ");
    Serial.println(filamentoPresente(sensorEntradaPin(t)) ? "OK" : "Vazio");
  }
  Serial.println("STATUS OK");
  Serial.println("=======================");
}

// ═══════════════════════════════════════════════════════
// SELECIONAR TOOL
// ═══════════════════════════════════════════════════════

void selecionarTool(int tool) {
  Serial.print("Selecionando T"); Serial.println(tool);

  if (tool == currentExtruder) {
    lastOperationOk = true;
    Serial.print("T"); Serial.print(tool); Serial.println(" OK");
    return;
  }

  moverSeletor(currentExtruder, tool);
  currentExtruder = tool;
  lastExtruder    = tool;
  salvarTool(tool);
  Serial.print("T"); Serial.print(tool); Serial.println(" OK");
}

// ═══════════════════════════════════════════════════════
// ROTATE SELECTOR
// ═══════════════════════════════════════════════════════

void rotateSelector(bool direction, int moveDistance) {
  digitalWrite(SEL_EN, LOW);
  digitalWrite(SEL_DIR, direction);
  for (int x = 0; x < (moveDistance - 1); x++) {
    digitalWrite(SEL_STEP, HIGH); delayMicroseconds(SEL_SPEED_DELAY);
    digitalWrite(SEL_STEP, LOW);  delayMicroseconds(SEL_SPEED_DELAY);
  }
  // SEL_EN mantido LOW — motor travado na posicao
}

// ═══════════════════════════════════════════════════════
// HOME DO SELETOR
// ═══════════════════════════════════════════════════════

void homeSelector() {
  Serial.println("Homing seletor...");

  digitalWrite(SEL_EN, LOW);
  digitalWrite(SEL_DIR, CW);

  long maxSteps = (long)STEPS_PER_REV * MICROSTEPS * 10;
  long steps    = 0;

  while (digitalRead(ENDSTOP_SELETOR) == HIGH && steps < maxSteps) {
    digitalWrite(SEL_STEP, HIGH); delayMicroseconds(SEL_SPEED_DELAY);
    digitalWrite(SEL_STEP, LOW);  delayMicroseconds(SEL_SPEED_DELAY);
    steps++;
  }

  if (steps >= maxSteps) {
    Serial.println("ERRO: Endstop nao encontrado no home!");
    return;
  }

  digitalWrite(SEL_DIR, CCW);
  for (int i = 0; i < DEFAULT_BACKOFF * MICROSTEPS; i++) {
    digitalWrite(SEL_STEP, HIGH); delayMicroseconds(SEL_SPEED_DELAY);
    digitalWrite(SEL_STEP, LOW);  delayMicroseconds(SEL_SPEED_DELAY);
  }

  // Ambos motores ativos apos home
  digitalWrite(SEL_EN, LOW);
  digitalWrite(EXT_EN, LOW);
  currentExtruder = 0;
  Serial.println("Home OK");
}

// ═══════════════════════════════════════════════════════
// MOVER SELETOR
// ═══════════════════════════════════════════════════════

void moverSeletor(int de, int para) {
  if (de == para) return;

  Serial.print("Seletor T"); Serial.print(de);
  Serial.print(" -> T"); Serial.println(para);

  int  delta     = para - de;
  bool direction = CCW;

  if (delta < 0) {
    direction = CW;
    delta     = abs(delta);
  }

  for (int i = 0; i < delta; i++) {
    rotateSelector(direction, (STEPS_PER_REV / 4) * MICROSTEPS);
  }

  Serial.println("Seletor OK");
}

// ═══════════════════════════════════════════════════════
// IDLE
// ═══════════════════════════════════════════════════════

void moverIdle() {
  int idlePos;
  if      (lastExtruder == 0) idlePos = 2;
  else if (lastExtruder == 1) idlePos = 3;
  else if (lastExtruder == 2) idlePos = 0;
  else                         idlePos = 1;

  if (currentExtruder != idlePos) {
    moverSeletor(currentExtruder, idlePos);
    currentExtruder = idlePos;
    salvarTool(currentExtruder);
  }
  Serial.println("Idle OK");
}

// ═══════════════════════════════════════════════════════
// BUFFER DRAIN
// ═══════════════════════════════════════════════════════

void iniciarBufferDrain() {
  Serial.println("Buffer drain iniciando...");
  imprimindo = false;

  if (bufferCheio()) {
    Serial.println("Esvaziando buffer...");
    digitalWrite(EXT_EN, LOW);
    digitalWrite(EXT_DIR, direcaoUnload(lastExtruder));

    long maxSteps = (long)(300.0 * STEPS_PER_MM);
    long steps    = 0;

    while (steps < maxSteps) {
      if (bufferVazio()) break;
      extStep(SPEED_DELAY);
      steps++;
    }

    digitalWrite(EXT_EN, HIGH);
    Serial.print("Buffer esvaziado: ");
    Serial.print((float)steps / STEPS_PER_MM);
    Serial.println("mm");
  } else {
    Serial.println("Buffer ja vazio.");
  }

  bufferDrainMode = true;
  Serial.println("Buffer Drain OK");
}

void executarBufferDrain() {
  if (bufferCheio()) {
    Serial.println("Buffer drain: esvaziando...");
    digitalWrite(EXT_EN, LOW);
    digitalWrite(EXT_DIR, direcaoUnload(lastExtruder));

    long maxSteps = (long)(300.0 * STEPS_PER_MM);
    long steps    = 0;

    while (steps < maxSteps) {
      if (Serial.available() > 0) {
        String incoming = Serial.readStringUntil('\n');
        incoming.trim();
        incoming.toUpperCase();
        if (incoming == "UNLOAD") {
          digitalWrite(EXT_EN, HIGH);
          bufferDrainMode = false;
          Serial.println("Buffer drain desativado");
          int loadedTool = carregarToolCarregada();
          if (loadedTool < 0 && filamentoPresente(SENSOR_SAIDA)) {
            loadedTool = currentExtruder;
            Serial.print("UNLOAD: usando ferramenta atual pelo sensor do Hub: T");
            Serial.println(loadedTool);
          }
          if (loadedTool >= 0) {
            if (currentExtruder != loadedTool) {
              moverSeletor(currentExtruder, loadedTool);
              currentExtruder = loadedTool;
              lastExtruder = loadedTool;
              salvarTool(loadedTool);
            }
            unloadAteSensor(loadedTool);
          } else {
            lastOperationOk = false;
            Serial.println("ERRO: UNLOAD bloqueado: nenhuma ferramenta carregada na EEPROM.");
          }
          return;
        }
      }
      if (bufferVazio()) break;
      extStep(SPEED_DELAY_FAST);
      steps++;
    }

    digitalWrite(EXT_EN, HIGH);
    Serial.println("Buffer drain: vazio.");
  }
}

// ═══════════════════════════════════════════════════════
// UNLOAD
// ═══════════════════════════════════════════════════════

void unloadAteSensor(int tool) {
  Serial.print("Unload T"); Serial.println(tool);

  digitalWrite(EXT_EN, LOW);
  digitalWrite(EXT_DIR, direcaoUnload(tool));

  long steps    = 0;
  long maxSteps = (long)(configDistMax * STEPS_PER_MM);
  bool ok       = false;

  while (steps < maxSteps) {
    if (!filamentoPresente(SENSOR_SAIDA)) { ok = true; break; }
    extStep(velocidadeComRampa(steps));
    steps++;
  }

  if (!ok) {
    digitalWrite(EXT_EN, HIGH);
    lastOperationOk = false;
    unloadSensorError = true;
    i2cStatus = STA_ERROR;
    Serial.println("ERRO UNLOAD: filamento quebrou ou sensor do Hub com defeito.");
    return;
  }

  Serial.print("Sensor saida OK: ");
  Serial.print((float)steps / STEPS_PER_MM);
  Serial.println("mm");

  long extraSteps = (long)(UNLOAD_AFTER_HUB_MM * STEPS_PER_MM);
  for (long i = 0; i < extraSteps; i++) extStep(configSpdFast);

  digitalWrite(EXT_EN, HIGH);
  unloadSensorError = false;
  salvarToolCarregada(-1);
  Serial.print("Unload OK: ");
  Serial.print((float)(steps + extraSteps) / STEPS_PER_MM);
  Serial.println("mm");
}

// ═══════════════════════════════════════════════════════
// UNLOAD RETRACAO
// ═══════════════════════════════════════════════════════

void unloadRetracao(int tool, float mm) {
  Serial.print("Unload retracao T"); Serial.print(tool);
  Serial.print(" "); Serial.print(mm); Serial.println("mm");

  long steps = (long)(mm * STEPS_PER_MM);

  digitalWrite(EXT_EN, LOW);
  digitalWrite(EXT_DIR, direcaoUnload(tool));

  for (long x = 0; x < steps; x++) extStep(configSpdFast);

  digitalWrite(EXT_EN, HIGH);
  Serial.println("Unload retracao OK");
}

// ═══════════════════════════════════════════════════════
// LOAD MANUAL — display ESP
// ═══════════════════════════════════════════════════════

void loadManual(int tool) {
  Serial.print("Load manual T"); Serial.println(tool);

  int inicio = verificarInicioLoad(tool);
  if (inicio == LOAD_JA_CARREGADO) {
    Serial.println("Load OK: filamento ja presente");
    return;
  }
  if (inicio == LOAD_BLOQUEADO) {
    return;
  }

  digitalWrite(EXT_EN, LOW);
  digitalWrite(EXT_DIR, direcaoLoad(tool));

  long steps    = 0;
  long maxSteps = distanciaLimiteLoadSteps();
  bool ok       = false;

  while (steps < maxSteps) {
    if (filamentoPresente(SENSOR_HOTEND)) { ok = true; break; }
    extStep(velocidadeComRampa(steps));
    steps++;
  }

  if (ok) {
    long extra4mm = (long)(4.0 * STEPS_PER_MM);
    for (long i = 0; i < extra4mm; i++) extStep(configSpdFast);
    Serial.print("Load OK: ");
    Serial.print((float)(steps + extra4mm) / STEPS_PER_MM);
    Serial.println("mm");
    salvarToolCarregada(tool);
    imprimindo = true;
    encherBuffer(tool);
  } else {
    digitalWrite(EXT_EN, HIGH);
    Serial.println("ERRO: Load atingiu limite maximo sem sensor!");
  }
}

// ═══════════════════════════════════════════════════════
// LOAD AUTOMATICO — Klipper
// ═══════════════════════════════════════════════════════

void loadContinuo(int tool) {
  Serial.print("Load auto T"); Serial.println(tool);

  int inicio = verificarInicioLoad(tool);
  if (inicio == LOAD_JA_CARREGADO) {
    Serial.println("Load OK: filamento ja presente");
    return;
  }
  if (inicio == LOAD_BLOQUEADO) {
    return;
  }

  digitalWrite(EXT_EN, LOW);
  digitalWrite(EXT_DIR, direcaoLoad(tool));

  long steps    = 0;
  long maxSteps = distanciaLimiteLoadSteps();
  bool ok       = false;

  while (steps < maxSteps) {
    if (filamentoPresente(SENSOR_HOTEND)) { ok = true; break; }
    extStep(velocidadeComRampa(steps));
    steps++;
  }

  if (ok) {
    long extra4mm = (long)(4.0 * STEPS_PER_MM);
    for (long i = 0; i < extra4mm; i++) extStep(configSpdFast);
    Serial.print("Load OK: ");
    Serial.print((float)(steps + extra4mm) / STEPS_PER_MM);
    Serial.println("mm");
    salvarToolCarregada(tool);
    imprimindo = true;
    encherBuffer(tool);
  } else {
    digitalWrite(EXT_EN, HIGH);
    Serial.println("ERRO: Load atingiu limite maximo sem sensor!");
  }
}

// ═══════════════════════════════════════════════════════
// ENCHER BUFFER
// ═══════════════════════════════════════════════════════

void encherBuffer(int tool) {
  if (bufferCheio()) {
    Serial.println("Buffer ja cheio.");
    pararAlimentadorSemDestravar();
    return;
  }

  Serial.println("Enchendo buffer...");
  digitalWrite(EXT_EN, LOW);
  digitalWrite(EXT_DIR, direcaoLoad(tool));

  long steps    = 0;
  long maxSteps = (long)(300.0 * STEPS_PER_MM);

  while (steps < maxSteps) {
    if (bufferCheio()) break;
    extStep(SPEED_DELAY);
    steps++;
  }

  pararAlimentadorSemDestravar();
  Serial.print("Buffer cheio: ");
  Serial.print((float)steps / STEPS_PER_MM);
  Serial.println("mm");
}

// ═══════════════════════════════════════════════════════
// LOAD ATE SENSOR HOTEND + EXTRA
// ═══════════════════════════════════════════════════════

void executarPreLoad(int tool) {
  Serial.print("Pre-load T"); Serial.println(tool);

  if (filamentoPresente(SENSOR_SAIDA)) {
    Serial.println("Pre-load ignorado: filamento ja esta no Hub.");
    return;
  }

  if (currentExtruder != tool) {
    moverSeletor(currentExtruder, tool);
    currentExtruder = tool;
    lastExtruder = tool;
  }

  i2cStatus = STA_BUSY;
  digitalWrite(EXT_EN, LOW);
  digitalWrite(EXT_DIR, direcaoLoad(tool));

  long steps = 0;
  long maxSteps = (long)(PRELOAD_MAX_MM * STEPS_PER_MM);
  bool chegouHub = false;

  while (steps < maxSteps) {
    if (filamentoPresente(SENSOR_SAIDA)) { chegouHub = true; break; }
    extStep(configSpdFast);
    steps++;
  }

  if (!chegouHub) {
    digitalWrite(EXT_EN, HIGH);
    i2cStatus = STA_ERROR;
    Serial.println("ERRO: Pre-load nao chegou no Hub.");
    return;
  }

  Serial.print("Pre-load Hub OK: ");
  Serial.print((float)steps / STEPS_PER_MM);
  Serial.println("mm");

  digitalWrite(EXT_DIR, direcaoUnload(tool));
  long retractSteps = (long)(PRELOAD_RETRACT_MM * STEPS_PER_MM);
  for (long i = 0; i < retractSteps; i++) extStep(configSpdFast);

  digitalWrite(EXT_EN, HIGH);
  i2cStatus = STA_IDLE;
  Serial.println("Pre-load OK: retraiu 40mm.");
}

void monitorarPreLoad() {
  for (int tool = 0; tool < 4; tool++) {
    bool presente = filamentoPresente(sensorEntradaPin(tool));
    if (presente && !lastEntradaPreload[tool]) {
      lastEntradaPreload[tool] = true;
      if (filamentoPresente(SENSOR_SAIDA)) {
        Serial.print("Pre-load T"); Serial.print(tool);
        Serial.println(" ignorado: Hub ocupado.");
        continue;
      }
      executarPreLoad(tool);
    } else {
      lastEntradaPreload[tool] = presente;
    }
  }
}

void loadAteSensorHotend(int tool, float extraMM) {
  Serial.print("Load T"); Serial.print(tool);
  Serial.print(" extra: "); Serial.print(extraMM); Serial.println("mm");

  int inicio = verificarInicioLoad(tool);
  if (inicio == LOAD_JA_CARREGADO) {
    Serial.println("Load OK: filamento ja presente");
    return;
  }
  if (inicio == LOAD_BLOQUEADO) {
    return;
  }

  digitalWrite(EXT_EN, LOW);
  digitalWrite(EXT_DIR, direcaoLoad(tool));

  long steps    = 0;
  long maxSteps = distanciaLimiteLoadSteps();
  bool ok       = false;

  while (steps < maxSteps) {
    if (filamentoPresente(SENSOR_HOTEND)) { ok = true; break; }
    extStep(velocidadeComRampa(steps));
    steps++;
  }

  if (!ok) {
    Serial.println("ERRO: Sensor hotend nao detectou filamento!");
    digitalWrite(EXT_EN, HIGH);
    return;
  }

  Serial.print("Sensor hotend OK: ");
  Serial.print((float)steps / STEPS_PER_MM);
  Serial.println("mm");

  if (extraMM > 0) {
    long extraSteps = (long)(extraMM * STEPS_PER_MM);
    for (long i = 0; i < extraSteps; i++) extStep(configSpdFast);
    Serial.print("Extra "); Serial.print(extraMM); Serial.println("mm OK");
  }

  digitalWrite(EXT_EN, HIGH);
  salvarToolCarregada(tool);
  Serial.println("Load OK");
}

// ═══════════════════════════════════════════════════════
// MONITOR BUFFER
// ═══════════════════════════════════════════════════════

void monitorarBuffer() {
  bool vazio = bufferVazio();
  bool cheio = bufferCheio();

  if (vazio != lastBufferVazio) {
    Serial.print("Buffer vazio: "); Serial.println(vazio ? "SIM" : "NAO");
    lastBufferVazio = vazio;
  }
  if (cheio != lastBufferCheio) {
    Serial.print("Buffer cheio: "); Serial.println(cheio ? "SIM" : "NAO");
    lastBufferCheio = cheio;
  }

  if (!filamentoPresente(sensorEntradaPin(lastExtruder))) {
    Serial.print("RUNOUT T"); Serial.println(lastExtruder);
  }

  if (vazio && !cheio) {
    if (!filamentoPresente(SENSOR_HOTEND)) return;
    Serial.println("Reabastecendo buffer...");
    if (currentExtruder != lastExtruder) {
      moverSeletor(currentExtruder, lastExtruder);
      currentExtruder = lastExtruder;
    }
    reabastecerBuffer();
  }
}

void reabastecerBuffer() {
  digitalWrite(EXT_EN, LOW);
  digitalWrite(EXT_DIR, direcaoLoad(lastExtruder));

  long steps    = 0;
  long maxSteps = (long)(300.0 * STEPS_PER_MM);
  bool cheio    = false;

  while (steps < maxSteps) {
    if (bufferCheio()) { cheio = true; break; }
    extStep(SPEED_DELAY);
    steps++;
  }

  pararAlimentadorSemDestravar();

  if (cheio) {
    Serial.print("Buffer reabastecido: ");
    Serial.print((float)steps / STEPS_PER_MM);
    Serial.println("mm");
  } else {
    Serial.println("ERRO: Buffer nao encheu!");
  }
}
