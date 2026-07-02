/* ════════════════════════════════════════════════════════════════════════
   3D CHAMELEON MK4
   ESP32 + ST7789 320x240
   LAYOUT CLEAN V14 — I2C Pico + ATIVO por LOAD + Menos Pisca
════════════════════════════════════════════════════════════════════════ */

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <Wire.h>
#include <DHT.h>
#include <Preferences.h>

// ═══════════════════════════════════════
// DISPLAY (VSPI — pinos padrao)
// ═══════════════════════════════════════

#define TFT_CS    15
#define TFT_DC     2
#define TFT_RST    4
#define TFT_MOSI  23
#define TFT_SCLK  18

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// ═══════════════════════════════════════
// I2C PICO (ESP32 mestre / Pico escravo)
//
// Ligacao ESP32 <-> Pico:
//   ESP GPIO21 (SDA) -> Pico GPIO4 (SDA)
//   ESP GPIO22 (SCL) -> Pico GPIO5 (SCL)
//   GND              -> GND
//
// Use pull-up de 4.7k em SDA e SCL para 3.3V.
// ═══════════════════════════════════════

#define PICO_I2C_SDA   21
#define PICO_I2C_SCL   22
#define PICO_I2C_ADDR  0x12

#define I2C_CMD_SIZE   4
#define I2C_STA_SIZE   16
#define I2C_SOF_CMD    0xAA
#define I2C_SOF_STA    0xBB
#define STEPS_PER_MM_F 428.0f
#define CONFIG_SPEED_MAX_MMS_F 70.0f

// Comandos
#define CMD_T0          0x01
#define CMD_T1          0x02
#define CMD_T2          0x03
#define CMD_T3          0x04
#define CMD_LOAD        0x05
#define CMD_UNLOAD      0x06
#define CMD_HOME        0x07
#define CMD_START_PRINT 0x08
#define CMD_STOP_PRINT  0x09
#define CMD_STATUS      0x0A
#define CMD_CALIBRAR    0x70
#define CMD_SPD_FAST_UP  0x80
#define CMD_SPD_FAST_DN  0x81
#define CMD_SPD_RAMP_UP  0x82
#define CMD_SPD_RAMP_DN  0x83
#define CMD_RAMP_MM_UP   0x84
#define CMD_RAMP_MM_DN   0x85
#define CMD_DIST_UP      0x86
#define CMD_DIST_DN      0x87
#define CMD_RESET_CONFIG 0x88

// Status do Pico
#define STA_IDLE     0x00
#define STA_BUSY     0x01
#define STA_PRINTING 0x02
#define STA_ERROR    0x03

// ═══════════════════════════════════════
// DHT22
// ═══════════════════════════════════════

#define DHT_PIN  27
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

// Secagem de filamento (controle local no ESP32)
#define NTC_PIN       34
#define HEATER_PIN    26
#define DRYER_FAN_PIN 14

const float NTC_FIXED_OHMS = 100000.0f;
const float NTC_NOMINAL_OHMS = 100000.0f;
const float NTC_NOMINAL_C = 25.0f;
const float NTC_BETA = 3950.0f;
const float DRYER_COOLDOWN_C = 30.0f;
const float DRYER_HYSTERESIS_C = 1.0f;

// ═══════════════════════════════════════
// PREFERENCES
// ═══════════════════════════════════════

Preferences prefs;

// ═══════════════════════════════════════
// ENCODER
// ═══════════════════════════════════════

#define ENC_CLK 32
#define ENC_DT  33
#define ENC_SW  25

volatile int      encDelta         = 0;
volatile bool     btnPressed       = false;
volatile uint32_t lastBtnInterrupt = 0;
const int         ENC_COUNTS_PER_STEP = 2;
int               lastCLK_val      = HIGH;

// ═══════════════════════════════════════
// CORES
// ═══════════════════════════════════════

#define C_BG     0x0841
#define C_WHITE  0xFFFF
#define C_BLACK  0x0000
#define C_GRAY   0x7BEF
#define C_DARK   0x2104
#define C_CYAN   0x07FF
#define C_RED    0xF800
#define C_BLUE   0x001F
#define C_ORANGE 0xFD20
#define C_YELLOW 0xFFE0
#define C_ACTIVE 0x05BF
#define C_GREEN  0x07E0

const int COLOR_COUNT = 11;
const char* colorNames[COLOR_COUNT] = {
  "VERMELHO","VERDE","AZUL","AMARELO","LARANJA",
  "ROXO","BRANCO","CINZA","ROSA","MARROM","PRETO"
};
const uint16_t colorValues[COLOR_COUNT] = {
  0xF800, 0x07E0, 0x001F, 0xFFE0, 0xFD20,
  0x780F, 0xFFFF, 0x7BEF, 0xF81F, 0x8200, 0x0000
};

uint16_t toolColors[4] = { 0xF800, 0x07E0, 0x001F, 0xFFE0 };

// ═══════════════════════════════════════
// ESTADO ATUAL E ANTERIOR
// ═══════════════════════════════════════

struct ChameleonState {
  int   toolAtiva    = 0;
  bool  filamento[4] = { false, false, false, false };
  bool  bufferCheio  = false;
  bool  hub          = false;
  bool  hotend       = false;
  bool  imprimindo   = false;
  bool  unloadError  = false;
  float temperatura  = 0;
  float umidade      = 0;
  bool  picoOK       = false;
};

ChameleonState state;
ChameleonState prevState;
uint8_t picoStatusCode = STA_IDLE;

// Controle local do ESP32: etiqueta ATIVO aparece somente depois de LOAD
// e some depois de UNLOAD. Nao depende apenas da tool selecionada.
bool toolLoaded[4]     = { false, false, false, false };
bool prevToolLoaded[4] = { false, false, false, false };

bool homeDrawn = false;
bool unloadErrorShown = false;
bool prevDryerActive = false;

// ═══════════════════════════════════════
// NAVEGACAO
// ═══════════════════════════════════════

enum Screen {
  SCR_HOME,
  SCR_MENU,
  SCR_TOOL_ACTION,
  SCR_COLOR_SELECT,
  SCR_CONFIG,        // submenu de configuracoes
  SCR_DRYER
};

struct DryerState {
  float temperature = NAN;
  int targetTemperature = 50;
  int targetHumidity = 20;
  bool ntcOK = false;
  bool active = false;
  bool cooling = false;
  bool heaterOn = false;
  bool fanOn = false;
};

DryerState dryer;
int dryerIdx = 0;
bool dryerEditing = false;
const int DRYER_COUNT = 4;
unsigned long lastDryerControl = 0;
unsigned long lastDryerDraw = 0;

// Estado das configuracoes recebidas do Pico
struct ConfigState {
  float distMax  = 4000.0;
  int   spdFast  = 28;
  int   spdRamp  = 1325;
  float rampMM   = 60.0;
  bool  calibrado = false;
};

ConfigState cfg;

// Qual item do submenu config esta sendo editado
int cfgIdx = 0;
bool cfgEditing = false;
int cfgFastPercentEdit = 0;
int cfgRampPercentEdit = 0;

const int CONFIG_COUNT = 7;  // Calibrar + 4 ajustes + Reset + Voltar

Screen currentScreen = SCR_HOME;

int menuIdx      = 0;
int prevMenuIdx  = -1;
int toolActionIdx = 0;
int selectedToolIdx = 0;
int colorToolIdx = 0;
int colorPickIdx = 0;

const int TOOL_ACTION_COUNT = 4;
const char* toolActionLabels[TOOL_ACTION_COUNT] = {
  "EXTRUSAO",
  "RETRAIR",
  "COR FILAMENTO",
  "< VOLTAR"
};

// ═══════════════════════════════════════
// MENU
// ═══════════════════════════════════════

const int   MENU_COUNT = 12;
const char* menuLabels[MENU_COUNT] = {
  "< VOLTAR",
  "FILAMENTO T0","FILAMENTO T1","FILAMENTO T2","FILAMENTO T3",
  "LOAD","UNLOAD","HOME",
  "START","STOP",
  "SECAGEM FILAMENTO",
  "CONFIGURACAO"
};
const uint8_t menuCmds[MENU_COUNT] = {
  0x00,
  CMD_T0, CMD_T1, CMD_T2, CMD_T3,
  CMD_LOAD, CMD_UNLOAD, CMD_HOME,
  CMD_START_PRINT, CMD_STOP_PRINT,
  0x00,
  0x00  // CONFIGURACAO — tratado localmente
};

// ═══════════════════════════════════════
// FEEDBACK
// ═══════════════════════════════════════

bool          showingFeedback = false;
unsigned long feedbackStart   = 0;

// ═══════════════════════════════════════
// TIMERS
// ═══════════════════════════════════════

unsigned long lastDHT     = 0;
unsigned long lastRefresh = 0;

// ═══════════════════════════════════════
// ISR
// ═══════════════════════════════════════

void IRAM_ATTR encoderISR() {
  int clk = digitalRead(ENC_CLK);
  int dt  = digitalRead(ENC_DT);
  if (clk != lastCLK_val) {
    encDelta += (dt != clk) ? 1 : -1;
    lastCLK_val = clk;
  }
}

void IRAM_ATTR buttonISR() {
  uint32_t now = millis();
  if (now - lastBtnInterrupt > 500) {
    btnPressed       = true;
    lastBtnInterrupt = now;
  }
}

// ═══════════════════════════════════════
// I2C PICO
// ═══════════════════════════════════════

uint8_t i2cChecksum(uint8_t* buf, int len) {
  uint8_t chk = 0;
  for (int i = 0; i < len; i++) chk ^= buf[i];
  return chk;
}

bool picoReadStatus() {
  uint8_t rxBuf[I2C_STA_SIZE] = {0};

  int n = Wire.requestFrom(PICO_I2C_ADDR, I2C_STA_SIZE);
  if (n != I2C_STA_SIZE) {
    while (Wire.available()) Wire.read();
    return false;
  }

  for (int i = 0; i < I2C_STA_SIZE; i++) {
    rxBuf[i] = Wire.read();
  }

  if (rxBuf[0] != I2C_SOF_STA) return false;
  if (i2cChecksum(rxBuf, I2C_STA_SIZE - 1) != rxBuf[I2C_STA_SIZE - 1]) return false;

  // Pico envia:
  // [0xBB][STA][T][FILS][BUF][HE][FLAGS][DIST_LH][FAST_LH][RAMP_LH][RAMP_MM_LH][CHK]
  uint8_t sta  = rxBuf[1];
  uint8_t tool = rxBuf[2];
  uint8_t fils = rxBuf[3];
  uint8_t flags = rxBuf[6];

  state.picoOK    = true;
  state.toolAtiva = (tool > 0 && tool <= 4) ? tool - 1 : 0;
  picoStatusCode  = sta;

  for (int i = 0; i < 4; i++) {
    state.filamento[i] = (fils >> i) & 0x01;
  }

  state.bufferCheio = rxBuf[4] & 0x01;
  state.hub         = rxBuf[4] & 0x02;
  state.hotend      = rxBuf[5] & 0x01;
  state.imprimindo  = (sta == STA_PRINTING) || (flags & 0x01);
  state.unloadError = flags & 0x04;

  cfg.calibrado = flags & 0x02;
  cfg.distMax   = (float)((uint16_t)rxBuf[7]  | ((uint16_t)rxBuf[8]  << 8));
  cfg.spdFast   =         (uint16_t)rxBuf[9]  | ((uint16_t)rxBuf[10] << 8);
  cfg.spdRamp   =         (uint16_t)rxBuf[11] | ((uint16_t)rxBuf[12] << 8);
  cfg.rampMM    = (float)((uint16_t)rxBuf[13] | ((uint16_t)rxBuf[14] << 8));

  return true;
}

bool picoSendOnly(uint8_t cmd, uint8_t arg = 0) {
  uint8_t txBuf[I2C_CMD_SIZE] = {0};

  txBuf[0] = I2C_SOF_CMD;
  txBuf[1] = cmd;
  txBuf[2] = arg;
  txBuf[3] = i2cChecksum(txBuf, 3);

  Wire.beginTransmission(PICO_I2C_ADDR);
  Wire.write(txBuf, I2C_CMD_SIZE);
  uint8_t err = Wire.endTransmission();

  return err == 0;
}

bool picoTransaction(uint8_t cmd, uint8_t arg = 0) {
  if (!picoSendOnly(cmd, arg)) return false;
  delay(10);
  return picoReadStatus();
}

void picoPollStatus() {
  // Primeiro tenta leitura direta; se falhar, tenta pedir STATUS.
  if (!picoReadStatus()) {
    if (!picoTransaction(CMD_STATUS)) {
      state.picoOK = false;
    }
  }
}

bool picoSendCommand(uint8_t cmd, uint8_t arg = 0) {
  bool ok = picoTransaction(cmd, arg);
  if (!ok) {
    delay(60);
    ok = picoTransaction(cmd, arg);
  }
  return ok;
}

bool picoSendCommandWait(uint8_t cmd, uint8_t arg = 0, unsigned long timeoutMs = 30000) {
  if (!picoSendOnly(cmd, arg)) return false;

  unsigned long start = millis();
  bool gotStatus = false;

  while (millis() - start < timeoutMs) {
    delay(120);
    if (picoReadStatus()) {
      gotStatus = true;
      if (picoStatusCode != STA_BUSY) return picoStatusCode != STA_ERROR;
    }
  }

  return gotStatus && picoStatusCode != STA_ERROR;
}

float speedUsToMMS(uint16_t delayUs) {
  if (delayUs == 0) return 0.0f;
  return 1000000.0f / (2.0f * (float)delayUs * STEPS_PER_MM_F);
}

void printSpeedPercent(uint16_t delayUs) {
  float mmS = speedUsToMMS(delayUs);
  int percent = (int)((mmS * 100.0f / CONFIG_SPEED_MAX_MMS_F) + 0.5f);
  if (percent < 1 && mmS > 0.0f) percent = 1;
  if (percent > 100) percent = 100;
  tft.print(percent);
  tft.print("%");
}

int speedUsToPercent(uint16_t delayUs) {
  float mmS = speedUsToMMS(delayUs);
  int percent = (int)((mmS * 100.0f / CONFIG_SPEED_MAX_MMS_F) + 0.5f);
  if (percent < 1 && mmS > 0.0f) percent = 1;
  if (percent > 100) percent = 100;
  return percent;
}

uint16_t percentToSpeedUs(int percent) {
  if (percent < 1) percent = 1;
  if (percent > 100) percent = 100;
  float mmS = CONFIG_SPEED_MAX_MMS_F * ((float)percent / 100.0f);
  float delayUs = 1000000.0f / (2.0f * mmS * STEPS_PER_MM_F);
  if (delayUs < 10.0f) delayUs = 10.0f;
  if (delayUs > 5000.0f) delayUs = 5000.0f;
  return (uint16_t)(delayUs + 0.5f);
}

void printSpeedValue(uint16_t delayUs) {
  float mmS = speedUsToMMS(delayUs);
  if (mmS >= 10.0f) tft.print((int)(mmS + 0.5f));
  else tft.print(mmS, 1);
  tft.print("mm/s");
}

// ═══════════════════════════════════════
// PREFERENCES
// ═══════════════════════════════════════

void saveColors() {
  prefs.begin("chameleon", false);
  prefs.putUShort("c0", toolColors[0]);
  prefs.putUShort("c1", toolColors[1]);
  prefs.putUShort("c2", toolColors[2]);
  prefs.putUShort("c3", toolColors[3]);
  prefs.end();
}

void loadColors() {
  prefs.begin("chameleon", true);
  toolColors[0] = prefs.getUShort("c0", 0xF800);
  toolColors[1] = prefs.getUShort("c1", 0x07E0);
  toolColors[2] = prefs.getUShort("c2", 0x001F);
  toolColors[3] = prefs.getUShort("c3", 0xFFE0);
  prefs.end();
}

// ═══════════════════════════════════════
// DHT
// ═══════════════════════════════════════

void readDHT() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) state.temperatura = t;
  if (!isnan(h)) state.umidade     = h;
}

// ═══════════════════════════════════════
// COPIA ESTADO
// ═══════════════════════════════════════

void copyStateToPrev() {
  prevState = state;
  prevDryerActive = dryer.active;
  for (int i = 0; i < 4; i++) {
    prevToolLoaded[i] = toolLoaded[i];
  }
}

// ═══════════════════════════════════════
// UPDATE PARCIAL
// ═══════════════════════════════════════

void updateHomePartial() {
  for (int i = 0; i < 4; i++) {
    bool toolMudou  = (i == state.toolAtiva) != (i == prevState.toolAtiva);
    bool filMudou   = state.filamento[i] != prevState.filamento[i];
    bool loadMudou  = toolLoaded[i] != prevToolLoaded[i];

    if (toolMudou || filMudou || loadMudou) {
      drawRolo(i);
    }
  }

  // Compara inteiros para evitar pisca por variacao decimal pequena do DHT.
  if ((int)state.temperatura != (int)prevState.temperatura ||
      (int)state.umidade     != (int)prevState.umidade ||
      state.picoOK           != prevState.picoOK ||
      dryer.active           != prevDryerActive) {
    drawTempUmidade();
  }

  if (state.hotend      != prevState.hotend)      drawCardHotend();
  if (state.hub         != prevState.hub)         drawCardHub();
  if (state.bufferCheio != prevState.bufferCheio) drawCardBuffer();
  if (state.imprimindo  != prevState.imprimindo)  drawCardPrint();

  copyStateToPrev();
}

// ═══════════════════════════════════════
// SETUP
// ═══════════════════════════════════════

void setup() {
  Serial.begin(115200);

  pinMode(HEATER_PIN, OUTPUT);
  pinMode(DRYER_FAN_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);
  digitalWrite(DRYER_FAN_PIN, LOW);

  // VSPI — display
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
  tft.init(240, 320);
  tft.setRotation(3);
  tft.invertDisplay(false);
  tft.setSPISpeed(40000000);
  tft.fillScreen(C_BLACK);

  // I2C — Pico
  Wire.begin(PICO_I2C_SDA, PICO_I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(20);   // evita travar display se o Pico demorar no I2C

  // DHT22
  dht.begin();

  // Encoder
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT,  INPUT_PULLUP);
  pinMode(ENC_SW,  INPUT_PULLUP);
  lastCLK_val = digitalRead(ENC_CLK);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_SW),  buttonISR,  FALLING);

  loadColors();
  prefs.begin("dryer", true);
  dryer.targetTemperature = prefs.getInt("temp", 50);
  dryer.targetHumidity = prefs.getInt("humidity", 20);
  prefs.end();
  drawSplash();
  delay(1200);
  readDHT();

  // Primeira leitura do Pico
  picoPollStatus();

  drawHomeFull();
  copyStateToPrev();
}

// ═══════════════════════════════════════
// LOOP
// ═══════════════════════════════════════

void loop() {
  handleEncoder();
  handleButton();
  handleSerial();

  if (millis() - lastDryerControl >= 250) {
    controlDryer();
    lastDryerControl = millis();
  }

  if (millis() - lastDHT > 2500) {
    readDHT();
    lastDHT = millis();
  }

  if (currentScreen == SCR_DRYER && millis() - lastDryerDraw >= 1000) {
    drawDryerStatus();
    lastDryerDraw = millis();
  }

  if (currentScreen == SCR_HOME && !showingFeedback) {

    // Poll Pico em ritmo mais calmo para reduzir piscadas no display.
    if (millis() - lastRefresh > 1000) {
      picoPollStatus();
      if (state.unloadError && !unloadErrorShown) {
        unloadErrorShown = true;
        drawLoadAlert();
        delay(3000);
        drawHomeFull();
        copyStateToPrev();
      } else if (!state.unloadError) {
        unloadErrorShown = false;
      }
      updateHomePartial();
      lastRefresh = millis();
    }

  }
}

// ═══════════════════════════════════════
// CONTROLE SERIAL
// Comandos via monitor serial 115200:
//   U    → encoder cima (menu sobe)
//   D    → encoder baixo (menu desce)
//   OK   → botão confirmar
//   HOME → volta para tela home
//   SIM  → simula dados para teste
// ═══════════════════════════════════════

String serialCmd = "";

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      serialCmd.trim();
      serialCmd.toUpperCase();

      if (serialCmd == "U") {
        // Simula encoder para cima
        encDelta = ENC_COUNTS_PER_STEP;
        handleEncoder();

      } else if (serialCmd == "D") {
        // Simula encoder para baixo
        encDelta = -ENC_COUNTS_PER_STEP;
        handleEncoder();

      } else if (serialCmd == "OK") {
        // Simula botão
        btnPressed = true;
        handleButton();

      } else if (serialCmd == "HOME") {
        currentScreen = SCR_HOME;
        drawHomeFull();
        copyStateToPrev();
        Serial.println(">> Home");

      } else if (serialCmd == "SIM") {
        // Simula dados para teste visual
        state.filamento[0] = !state.filamento[0];
        state.filamento[1] = true;
        state.filamento[2] = false;
        state.filamento[3] = !state.filamento[3];
        state.bufferCheio  = !state.bufferCheio;
        state.hotend       = !state.hotend;
        state.imprimindo   = !state.imprimindo;
        state.temperatura  = 27.5;
        state.umidade      = 62.0;
        state.picoOK       = true;
        toolLoaded[state.toolAtiva] = state.hotend && state.bufferCheio;
        updateHomePartial();
        Serial.println(">> Dados simulados");

      } else if (serialCmd.startsWith("TOOL ")) {
        int t = serialCmd.substring(5).toInt();
        if (t >= 0 && t <= 3) {
          state.toolAtiva = t;
          updateHomePartial();
          Serial.print(">> Tool "); Serial.println(t);
        }

      } else if (serialCmd != "") {
        Serial.println("Cmds: U D OK HOME SIM TOOL 0-3");
      }

      serialCmd = "";
    } else {
      serialCmd += c;
    }
  }
}

// ═══════════════════════════════════════
// ENCODER
// ═══════════════════════════════════════

void handleEncoder() {
  int delta = 0;
  noInterrupts();
  if      (encDelta >= ENC_COUNTS_PER_STEP)  { delta =  1; encDelta = 0; }
  else if (encDelta <= -ENC_COUNTS_PER_STEP) { delta = -1; encDelta = 0; }
  interrupts();
  if (delta == 0) return;

  switch (currentScreen) {
    case SCR_MENU:
    {
      int oldIdx = menuIdx;
      menuIdx += delta;
      if (menuIdx < 0)           menuIdx = MENU_COUNT - 1;
      if (menuIdx >= MENU_COUNT) menuIdx = 0;
      drawMenuPartial(oldIdx);
      break;
    }

    case SCR_TOOL_ACTION:
      toolActionIdx += delta;
      if (toolActionIdx < 0) toolActionIdx = TOOL_ACTION_COUNT - 1;
      if (toolActionIdx >= TOOL_ACTION_COUNT) toolActionIdx = 0;
      drawToolActionMenu();
      break;

    case SCR_COLOR_SELECT:
      colorPickIdx += delta;
      if (colorPickIdx < 0)            colorPickIdx = COLOR_COUNT - 1;
      if (colorPickIdx >= COLOR_COUNT) colorPickIdx = 0;
      drawColorSelect();
      break;

    case SCR_CONFIG:
      // Configuracao agora funciona igual menu:
      // gira navega; OK entra/sai da edicao dos valores.
      if (cfgEditing && cfgIdx >= 1 && cfgIdx <= 4) {
        adjustConfig(cfgIdx, delta);
        drawConfig();
      } else {
        cfgIdx += delta;
        if (cfgIdx < 0)                cfgIdx = CONFIG_COUNT - 1;
        if (cfgIdx >= CONFIG_COUNT)    cfgIdx = 0;
        drawConfig();
      }
      break;

    case SCR_DRYER:
      if (dryerEditing && (dryerIdx == 1 || dryerIdx == 2)) {
        if (dryerIdx == 1) {
          dryer.targetTemperature += delta;
          if (dryer.targetTemperature < 30) dryer.targetTemperature = 30;
          if (dryer.targetTemperature > 80) dryer.targetTemperature = 80;
        } else {
          dryer.targetHumidity += delta;
          if (dryer.targetHumidity < 5) dryer.targetHumidity = 5;
          if (dryer.targetHumidity > 60) dryer.targetHumidity = 60;
        }
      } else {
        dryerIdx += delta;
        if (dryerIdx < 0) dryerIdx = DRYER_COUNT - 1;
        if (dryerIdx >= DRYER_COUNT) dryerIdx = 0;
      }
      drawDryer();
      break;

    default:
      break;
  }
}

// ═══════════════════════════════════════
// BOTAO
// ═══════════════════════════════════════

void handleButton() {
  if (!btnPressed) return;
  btnPressed = false;

  switch (currentScreen) {

    case SCR_HOME:
      currentScreen = SCR_MENU;
      menuIdx = 0;
      drawMenu();
      break;

    case SCR_MENU:
      if (menuIdx == 0) {
        currentScreen = SCR_HOME;
        drawHomeFull();
        copyStateToPrev();
      } else if (menuIdx >= 1 && menuIdx <= 4) {
        openToolAction(menuIdx - 1);
      } else if (menuIdx == MENU_COUNT - 2) {
        currentScreen = SCR_DRYER;
        dryerIdx = 0;
        dryerEditing = false;
        drawDryer();
      } else if (menuIdx == MENU_COUNT - 1) {
        currentScreen = SCR_CONFIG;
        cfgIdx = 0;
        cfgEditing = false;
        picoPollStatus();
        drawConfig();
      } else {
        executeMenu(menuIdx);
      }
      break;

    case SCR_TOOL_ACTION:
      if (toolActionIdx == 0) {
        executeToolAction(CMD_LOAD, "EXTRUSAO");
      } else if (toolActionIdx == 1) {
        executeToolAction(CMD_UNLOAD, "RETRAIR");
      } else if (toolActionIdx == 2) {
        colorToolIdx = selectedToolIdx;
        colorPickIdx = 0;
        for (int i = 0; i < COLOR_COUNT; i++) {
          if (colorValues[i] == toolColors[colorToolIdx]) {
            colorPickIdx = i;
            break;
          }
        }
        currentScreen = SCR_COLOR_SELECT;
        drawColorSelect();
      } else {
        currentScreen = SCR_MENU;
        drawMenu();
      }
      break;

    case SCR_CONFIG:
      // Item 0: calibrar
      if (cfgIdx == 0) {
        cfgEditing = false;
        drawFeedback("CALIBRAR", 0);
        bool ok = picoSendCommandWait(CMD_CALIBRAR, 0, 180000);
        if (ok) picoPollStatus();
        drawFeedback("CALIBRAR", ok ? 1 : 2);
        delay(900);
        drawConfig();
      }
      // Itens 1 a 4: entra/sai modo edicao
      else if (cfgIdx >= 1 && cfgIdx <= 4) {
        if (!cfgEditing && cfgIdx == 2) {
          cfgFastPercentEdit = speedUsToPercent(cfg.spdFast);
        } else if (!cfgEditing && cfgIdx == 3) {
          cfgRampPercentEdit = speedUsToPercent(cfg.spdRamp);
        }
        cfgEditing = !cfgEditing;
        drawConfig();
      }
      // Reset da configuracao de calibracao
      else if (cfgIdx == 5) {
        cfgEditing = false;
        drawFeedback("RESET CAL", 0);
        bool ok = picoSendCommand(CMD_RESET_CONFIG);
        if (ok) picoPollStatus();
        drawFeedback("RESET CAL", ok ? 1 : 2);
        delay(900);
        drawConfig();
      }
      // Ultimo item: voltar
      else if (cfgIdx == CONFIG_COUNT - 1) {
        cfgEditing = false;
        currentScreen = SCR_MENU;
        drawMenu();
      }
      break;

    case SCR_DRYER:
      if (dryerIdx == 0) {
        if (dryer.active) {
          stopDryer();
        } else if (dryer.ntcOK) {
          dryer.active = true;
          dryer.cooling = false;
          Serial.println("Secagem iniciada.");
        }
        drawDryer();
      } else if (dryerIdx == 1 || dryerIdx == 2) {
        dryerEditing = !dryerEditing;
        if (!dryerEditing) saveDryerSettings();
        drawDryer();
      } else {
        dryerEditing = false;
        currentScreen = SCR_MENU;
        drawMenu();
      }
      break;

    case SCR_COLOR_SELECT:
      toolColors[colorToolIdx] = colorValues[colorPickIdx];
      saveColors();
      currentScreen = SCR_TOOL_ACTION;
      drawToolActionMenu();
      break;
  }
}

// ═══════════════════════════════════════
// EXECUTE MENU
// ═══════════════════════════════════════

void openToolAction(int tool) {
  selectedToolIdx = tool;
  toolActionIdx = 0;

  char label[8];
  snprintf(label, sizeof(label), "TOOL T%d", tool);
  drawFeedback(label, 0);

  bool ok = picoSendCommand(CMD_T0 + tool);
  if (ok) {
    state.toolAtiva = tool;
    picoPollStatus();
  }

  drawFeedback(label, ok ? 1 : 2);
  delay(650);

  if (ok) {
    currentScreen = SCR_TOOL_ACTION;
    drawToolActionMenu();
  } else {
    currentScreen = SCR_MENU;
    drawMenu();
  }
}

void executeToolAction(uint8_t cmd, const char* label) {
  int toolAntes = selectedToolIdx;

  if (cmd == CMD_LOAD) {
    picoPollStatus();
    if (state.picoOK && state.hotend) {
      if (state.hub) {
        drawHotendLoadedAlert();
        delay(2400);
        drawToolActionMenu();
        return;
      } else {
        drawLoadAlert();
        delay(2600);
        drawToolActionMenu();
        return;
      }
    }
  }

  drawFeedback(label, 0);
  bool ok = picoSendCommand(cmd);

  if (!ok && cmd == CMD_UNLOAD && state.unloadError) {
    unloadErrorShown = true;
    drawLoadAlert();
    delay(3000);
    currentScreen = SCR_TOOL_ACTION;
    drawToolActionMenu();
    return;
  }

  if (ok) {
    if (cmd == CMD_LOAD) {
      toolLoaded[toolAntes] = true;
      state.filamento[toolAntes] = true;
    } else if (cmd == CMD_UNLOAD) {
      toolLoaded[toolAntes] = false;
    }
    picoPollStatus();
  }

  drawFeedback(label, ok ? 1 : 2);
  delay(650);
  currentScreen = SCR_TOOL_ACTION;
  drawToolActionMenu();
}

void executeMenu(int idx) {
  uint8_t cmd = menuCmds[idx];
  if (cmd == 0x00) return;

  int toolAntes = state.toolAtiva;

  if (cmd == CMD_LOAD) {
    picoPollStatus();
    if (state.picoOK && state.hotend) {
      if (state.hub) {
        drawHotendLoadedAlert();
        delay(2400);
        currentScreen = SCR_MENU;
        drawMenu();
        return;
      }
      else {
        drawLoadAlert();
        delay(2600);
        currentScreen = SCR_MENU;
        drawMenu();
        return;
      }
    }
  }

  drawFeedback(menuLabels[idx], 0);          // enviando
  bool ok = picoSendCommand(cmd);

  if (!ok && cmd == CMD_UNLOAD && state.unloadError) {
    unloadErrorShown = true;
    drawLoadAlert();
    delay(3000);
    currentScreen = SCR_MENU;
    drawMenu();
    return;
  }

  if (ok) {
    // Atualiza estado local imediatamente para a tela responder rapido.
    if (cmd >= CMD_T0 && cmd <= CMD_T3) {
      state.toolAtiva = cmd - CMD_T0;
    }
    else if (cmd == CMD_LOAD) {
      toolLoaded[toolAntes] = true;
      state.filamento[toolAntes] = true;
    }
    else if (cmd == CMD_UNLOAD) {
      toolLoaded[toolAntes] = false;
    }

    // Busca status mais recente do Pico sem depender do proximo ciclo.
    picoPollStatus();
  }

  drawFeedback(menuLabels[idx], ok ? 1 : 2);  // OK ou ERRO
  delay(650);

  currentScreen = SCR_MENU;
  drawMenu();
}

// ═══════════════════════════════════════
// SPLASH
// ═══════════════════════════════════════

void drawSplash() {
  tft.fillScreen(C_BLACK);

  tft.setTextSize(7);
  tft.setTextColor(C_YELLOW);
  tft.setCursor(48, 46);
  tft.print("3");
  tft.setTextColor(C_GREEN);
  tft.setCursor(122, 46);
  tft.print("D");
  tft.setTextColor(C_BLUE);
  tft.setCursor(202, 46);
  tft.print("C");

  tft.setTextSize(3);
  tft.setTextColor(C_GREEN);
  tft.setCursor(76, 132);
  tft.print("CAMALEAO");

  tft.setTextSize(2);
  tft.setTextColor(C_YELLOW);
  tft.setCursor(82, 202);
  tft.print("VERSAO BRASIL");
}

// ═══════════════════════════════════════
// HOME COMPLETO
// ═══════════════════════════════════════

void drawHomeFull() {
  tft.fillScreen(C_BG);
  for (int i = 0; i < 4; i++) drawRolo(i);
  tft.drawFastHLine(20, 135, 280, C_DARK);
  drawTempUmidade();
  drawCardHotend();
  drawCardHub();
  drawCardBuffer();
  drawCardPrint();
  homeDrawn = true;
}

// ═══════════════════════════════════════
// INDICADOR PICO
// ═══════════════════════════════════════

void drawPicoIndicator() {
  // Indicador removido do canto superior.
  // Agora ele e desenhado junto da temperatura/umidade em drawTempUmidade().
}

// ═══════════════════════════════════════
// ROLO INDIVIDUAL
// ═══════════════════════════════════════

void drawRolo(int i) {
  // Rolo menor para nao sobrepor na tela 320x240.
  // Borda externa, centro e texto T0/T1/T2/T3 ficam brancos por padrao.
  // Quando a ferramenta com filamento esta selecionada, esses detalhes ficam C_CYAN.
  // A etiqueta ATIVO aparece somente depois de LOAD.
  const int spacing = 74;
  const int startX  = 49;
  const int x       = startX + (i * spacing);

  bool ativoSelecionado = (i == state.toolAtiva);
  bool fil              = state.filamento[i];
  bool ativoLoad        = ativoSelecionado && fil && toolLoaded[i];
  bool destaque         = ativoSelecionado && fil;

  uint16_t corRolo      = fil ? toolColors[i] : C_GRAY;
  uint16_t corDetalhe   = destaque ? C_CYAN : (fil ? C_WHITE : C_DARK);
  uint16_t corTextoTool = destaque ? C_CYAN : (fil ? C_WHITE : C_DARK);

  // Limpa apenas a area do rolo para reduzir pisca.
  tft.fillRect(x - 34, 8, 68, 120, C_BG);

  // Corpo do rolo
  tft.fillCircle(x, 38, 22, corRolo);

  // Circulo externo: branco normal, C_CYAN quando selecionado com filamento.
  tft.drawCircle(x, 38, 22, corDetalhe);
  if (destaque) {
    tft.drawCircle(x, 38, 23, C_CYAN);
  }

  // Centro do rolo: fundo vazado com borda branca/C_CYAN.
  tft.fillCircle(x, 38, 8, C_BG);
  tft.drawCircle(x, 38, 8, corDetalhe);

  // Nome da ferramenta: branco normal, C_CYAN quando selecionado com filamento.
  tft.setTextSize(2);
  tft.setTextColor(corTextoTool);
  tft.setCursor(x - 10, 78);
  tft.print("T");
  tft.print(i);

  // Status compacto
  tft.setTextSize(1);

  if (ativoLoad) {
    tft.fillRoundRect(x - 20, 101, 40, 14, 4, C_GREEN);
    tft.setTextColor(C_BLACK);
    tft.setCursor(x - 14, 105);
    tft.print("ATIVO");
  }
  else if (!fil) {
    tft.fillRoundRect(x - 20, 101, 40, 14, 4, C_DARK);
    tft.setTextColor(C_GRAY);
    tft.setCursor(x - 15, 105);
    tft.print("VAZIO");
  }
  else {
    // Filamento existe, mas ainda nao foi carregado por LOAD.
    // Limpa a area da etiqueta para nao deixar sombra/pisca.
    tft.fillRect(x - 22, 99, 44, 18, C_BG);
  }
}

// ═══════════════════════════════════════
// TEMPERATURA E UMIDADE
// ═══════════════════════════════════════

void drawGotaUmidade(int x, int y, uint16_t cor) {
  // Gota simples desenhada por primitivos, sem imagem externa
  tft.fillCircle(x, y + 9, 7, cor);
  tft.fillTriangle(x, y - 7, x - 7, y + 8, x + 7, y + 8, cor);

  // pequeno brilho para dar aspecto de icone
  tft.fillCircle(x - 3, y + 5, 2, C_BG);
}

void drawTempUmidade() {
  // Area central: temperatura, umidade, I2C e secador
  tft.fillRect(20, 138, 280, 52, C_BG);

  // Temperatura
  tft.setTextSize(3);
  tft.setTextColor(C_ORANGE);
  tft.setCursor(28, 148);
  tft.print((int)state.temperatura);

  int cx = tft.getCursorX();
  int cy = tft.getCursorY();

  tft.fillCircle(cx + 2, cy + 2, 3, C_ORANGE);
  tft.setCursor(cx + 8, cy);
  tft.print("C");

  // Umidade com icone de gota no lugar do UR
  tft.setTextColor(C_CYAN);
  tft.setTextSize(3);
  tft.setCursor(108, 148);
  tft.print((int)state.umidade);
  tft.print("%");

  // Gota de agua depois do percentual
  drawGotaUmidade(190, 156, C_CYAN);

  // Status I2C em cima e secador logo abaixo, com a mesma fonte.
  tft.fillCircle(226, 150, 4, state.picoOK ? C_GREEN : C_RED);
  tft.setTextColor(state.picoOK ? C_GREEN : C_RED);
  tft.setTextSize(1);
  tft.setCursor(236, 146);
  tft.print(state.picoOK ? "I2C ON" : "I2C OFF");

  tft.fillCircle(226, 174, 4, dryer.active ? C_GREEN : C_GRAY);
  tft.setTextColor(dryer.active ? C_GREEN : C_GRAY);
  tft.setCursor(236, 170);
  tft.print(dryer.active ? "SECADOR ON" : "SECADOR OFF");
}

// ═══════════════════════════════════════
// STATUS CARDS
// ═══════════════════════════════════════

void drawCardHotend() {
  int x = 8, y = 190, w = 74, h = 42;
  tft.fillRoundRect(x, y, w, h, 8, C_BG);
  tft.drawRoundRect(x, y, w, h, 8, C_DARK);
  tft.setTextColor(C_WHITE);
  tft.setTextSize(1);
  tft.setCursor(x + 18, y + 8);
  tft.print("HOTEND");
  tft.setTextSize(1);
  tft.setTextColor(state.hotend ? C_CYAN : C_RED);
  tft.setCursor(x + 29, y + 25);
  tft.print(state.hotend ? "ON" : "OFF");
}

void drawCardHub() {
  int x = 86, y = 190, w = 74, h = 42;
  tft.fillRoundRect(x, y, w, h, 8, C_BG);
  tft.drawRoundRect(x, y, w, h, 8, C_DARK);
  tft.setTextColor(C_WHITE);
  tft.setTextSize(1);
  tft.setCursor(x + 26, y + 8);
  tft.print("HUB");
  tft.setTextColor(state.hub ? C_CYAN : C_GRAY);
  tft.setCursor(x + 29, y + 25);
  tft.print(state.hub ? "ON" : "OFF");
}

void drawCardBuffer() {
  int x = 164, y = 190, w = 74, h = 42;
  tft.fillRoundRect(x, y, w, h, 8, C_BG);
  tft.drawRoundRect(x, y, w, h, 8, C_DARK);
  tft.setTextColor(C_WHITE);
  tft.setTextSize(1);
  tft.setCursor(x + 16, y + 8);
  tft.print("BUFFER");
  tft.setTextColor(state.bufferCheio ? C_CYAN : C_ORANGE);
  tft.setCursor(x + 16, y + 25);
  tft.print(state.bufferCheio ? "CHEIO" : "VAZIO");
}

void drawCardPrint() {
  int x = 242, y = 190, w = 74, h = 42;
  tft.fillRoundRect(x, y, w, h, 8, C_BG);
  tft.drawRoundRect(x, y, w, h, 8, C_DARK);
  tft.setTextColor(C_WHITE);
  tft.setTextSize(1);
  tft.setCursor(x + 22, y + 8);
  tft.print("PRINT");
  tft.setTextColor(state.imprimindo ? C_CYAN : C_GRAY);
  tft.setCursor(x + 29, y + 25);
  tft.print(state.imprimindo ? "ON" : "OFF");
}

// ═══════════════════════════════════════
// MENU PRINCIPAL
// ═══════════════════════════════════════

// ═══════════════════════════════════════
// AJUSTA CONFIG — encoder envia comando ao Pico
// ═══════════════════════════════════════

void adjustConfig(int item, int delta) {
  if (item == 2) {
    int percent = cfgFastPercentEdit + delta;
    if (percent < 1) percent = 1;
    if (percent > 100) percent = 100;
    cfgFastPercentEdit = percent;
    cfg.spdFast = percentToSpeedUs(percent);
    uint8_t cmd = (delta > 0) ? CMD_SPD_FAST_UP : CMD_SPD_FAST_DN;
    picoSendOnly(cmd);
    return;
  }

  if (item == 3) {
    int percent = cfgRampPercentEdit + delta;
    if (percent < 1) percent = 1;
    if (percent > 100) percent = 100;
    cfgRampPercentEdit = percent;
    cfg.spdRamp = percentToSpeedUs(percent);
    uint8_t cmd = (delta > 0) ? CMD_SPD_RAMP_UP : CMD_SPD_RAMP_DN;
    picoSendOnly(cmd);
    return;
  }

  uint8_t cmd = 0x00;
  switch (item) {
    case 1: cmd = (delta > 0) ? CMD_DIST_UP      : CMD_DIST_DN;      break;
    case 4: cmd = (delta > 0) ? CMD_RAMP_MM_UP   : CMD_RAMP_MM_DN;   break;
    default: return;
  }

  picoSendOnly(cmd);

  switch (item) {
    case 1:
      cfg.distMax += (delta > 0) ? 50.0f : -50.0f;
      if (cfg.distMax < 100.0f) cfg.distMax = 100.0f;
      if (cfg.distMax > 6000.0f) cfg.distMax = 6000.0f;
      break;
    case 4:
      cfg.rampMM += (delta > 0) ? 5.0f : -5.0f;
      if (cfg.rampMM < 10.0f) cfg.rampMM = 10.0f;
      if (cfg.rampMM > 6000.0f) cfg.rampMM = 6000.0f;
      break;
  }
}

// ═══════════════════════════════════════
// TELA DE CONFIGURACAO
// ═══════════════════════════════════════

void drawConfig() {
  tft.fillRect(0, 38, 320, 202, C_BG);

  // Header
  tft.fillRect(0, 0, 320, 38, C_DARK);
  tft.setTextColor(C_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.print("CONFIGURACAO");

  char pos[12];
  snprintf(pos, sizeof(pos), "%d/%d", cfgIdx + 1, CONFIG_COUNT);
  tft.setTextColor(C_GRAY);
  tft.setTextSize(1);
  tft.setCursor(270, 13);
  tft.print(pos);

  // Caixas maiores para nao cortar letras/numeros
  const int START_Y = 46;
  const int ITEM_H  = 46;
  const int VISIBLE = 4;

  const char* labels[CONFIG_COUNT] = {
    "CALIBRAR",
    "DIST CAL",
    "VEL LOAD/UN",
    "VEL RAMPA",
    "RAMPA MM",
    "RESET CAL",
    "< VOLTAR"
  };

  int firstItem = cfgIdx - 1;
  if (firstItem < 0) firstItem = 0;
  if (firstItem > CONFIG_COUNT - VISIBLE) firstItem = CONFIG_COUNT - VISIBLE;
  if (firstItem < 0) firstItem = 0;

  for (int line = 0; line < VISIBLE; line++) {
    int i = firstItem + line;
    if (i >= CONFIG_COUNT) break;

    int y = START_Y + line * ITEM_H;
    bool sel = (i == cfgIdx);
    bool editingThis = (cfgEditing && sel && i >= 1 && i <= 4);

    uint16_t bgColor  = editingThis ? C_ORANGE : (sel ? C_ACTIVE : C_DARK);
    uint16_t txtColor = sel ? C_WHITE : C_GRAY;
    uint16_t borderColor = editingThis ? C_YELLOW : C_CYAN;

    // Fundo maior
    tft.fillRoundRect(8, y, 304, 42, 9, bgColor);

    if (sel) {
      tft.drawRoundRect(8, y, 304, 42, 9, borderColor);
      tft.fillTriangle(16, y + 13, 16, y + 25, 28, y + 19, C_WHITE);
    }

    // Nome do item
    tft.setTextColor(txtColor);
    tft.setTextSize(2);
    tft.setCursor(38, y + 7);
    tft.print(labels[i]);

    // Valor do item, alinhado mais para a direita
    tft.setTextSize(2);
    tft.setTextColor(sel ? C_WHITE : C_GRAY);
    tft.setCursor(210, y + 7);

    switch (i) {
      case 0:
        tft.setTextColor(cfg.calibrado ? C_GREEN : txtColor);
        tft.print(cfg.calibrado ? "OK" : "RUN");
        break;

      case 1:
        tft.print((int)cfg.distMax);
        tft.print("mm");
        break;

      case 2:
        if (editingThis) {
          tft.print(cfgFastPercentEdit);
          tft.print("%");
        } else {
          printSpeedPercent(cfg.spdFast);
        }
        break;

      case 3:
        if (editingThis) {
          tft.print(cfgRampPercentEdit);
          tft.print("%");
        } else {
          printSpeedPercent(cfg.spdRamp);
        }
        break;

      case 4:
        tft.print((int)cfg.rampMM);
        tft.print("mm");
        break;

      case 5:
        tft.print("OK?");
        break;

      case 6:
        tft.print("MENU");
        break;
    }

    if (editingThis) {
      tft.setTextSize(1);
      tft.setTextColor(C_BLACK);
      tft.setCursor(238, y + 28);
      tft.print("EDIT");
    }
  }

  // Rodape fixo
  tft.fillRect(0, 220, 320, 20, C_BG);
  tft.drawFastHLine(10, 218, 300, C_DARK);
  tft.setTextSize(1);
  tft.setTextColor(C_GRAY);
  tft.setCursor(22, 226);

  if (cfgEditing) {
    tft.print("GIRA=AJUSTA   OK=SALVA");
  } else {
    tft.print("GIRA=NAVEGA   OK=SELECIONA");
  }
}

const int MENU_START_Y = 42;
const int MENU_ITEM_H  = 34;
const int MENU_VISIBLE = 5;

int menuFirstItem(int selected) {
  int firstItem = selected - 2;
  if (firstItem < 0) firstItem = 0;
  if (firstItem > MENU_COUNT - MENU_VISIBLE) firstItem = MENU_COUNT - MENU_VISIBLE;
  if (firstItem < 0) firstItem = 0;
  return firstItem;
}

void drawMenuHeader() {
  tft.fillRect(0, 0, 320, 36, C_DARK);
  tft.setTextColor(C_CYAN);
  tft.setTextSize(2);
  tft.setCursor(12, 10);
  tft.print("MENU");

  char pos[12];
  snprintf(pos, sizeof(pos), "%d/%d", menuIdx + 1, MENU_COUNT);
  tft.setTextColor(C_GRAY);
  tft.setTextSize(1);
  tft.setCursor(270, 13);
  tft.print(pos);
}

void drawMenuItem(int i, int line) {
  if (i < 0 || i >= MENU_COUNT || line < 0 || line >= MENU_VISIBLE) return;

  int y = MENU_START_Y + line * MENU_ITEM_H;
  bool sel = (i == menuIdx);

  uint16_t bgColor  = sel ? C_ACTIVE : C_DARK;
  uint16_t txtColor = sel ? C_WHITE  : C_GRAY;

  tft.fillRect(0, y - 2, 320, MENU_ITEM_H, C_BG);
  tft.fillRect(10, y, 300, MENU_ITEM_H - 6, bgColor);

  if (sel) {
    tft.drawRect(10, y, 300, MENU_ITEM_H - 6, C_CYAN);
    tft.fillTriangle(18, y + 10, 18, y + 22, 29, y + 16, C_WHITE);
  }

  tft.setTextColor(txtColor);
  tft.setTextSize(2);
  if (i >= 1 && i <= 4) {
    int tool = i - 1;
    tft.fillCircle(43, y + 14, 8, toolColors[tool]);
    tft.drawCircle(43, y + 14, 9, sel ? C_WHITE : C_GRAY);
    tft.setCursor(58, y + 6);
  } else {
    tft.setCursor(38, y + 6);
  }
  tft.print(menuLabels[i]);
}

void drawMenuPartial(int oldIdx) {
  int oldFirst = menuFirstItem(oldIdx);
  int newFirst = menuFirstItem(menuIdx);

  if (oldFirst != newFirst) {
    drawMenu();
    return;
  }

  drawMenuItem(oldIdx, oldIdx - newFirst);
  drawMenuItem(menuIdx, menuIdx - newFirst);

  tft.fillRect(256, 6, 58, 18, C_DARK);
  char pos[12];
  snprintf(pos, sizeof(pos), "%d/%d", menuIdx + 1, MENU_COUNT);
  tft.setTextColor(C_GRAY);
  tft.setTextSize(1);
  tft.setCursor(270, 13);
  tft.print(pos);
}

void drawMenu() {
  tft.fillRect(0, 36, 320, 182, C_BG);
  drawMenuHeader();

  int firstItem = menuFirstItem(menuIdx);

  for (int line = 0; line < MENU_VISIBLE; line++) {
    int i = firstItem + line;
    if (i >= MENU_COUNT) break;
    drawMenuItem(i, line);
  }

  // Rodape fixo
  tft.fillRect(0, 218, 320, 22, C_BG);
  tft.drawFastHLine(10, 218, 300, C_DARK);
  tft.setTextColor(C_GRAY);
  tft.setTextSize(1);
  tft.setCursor(55, 225);
  tft.print("GIRA=NAVEGA   OK=SELECIONA");
}

// ═══════════════════════════════════════
// SELECAO DE COR
// ═══════════════════════════════════════

float readNTCTemperature() {
  uint32_t total = 0;
  for (int i = 0; i < 16; i++) total += analogRead(NTC_PIN);
  float adc = total / 16.0f;
  if (adc < 40.0f || adc > 4055.0f) return NAN;

  // Divisor: 3.3V -> resistor 100k -> GPIO34 -> NTC -> GND.
  float resistance = NTC_FIXED_OHMS * adc / (4095.0f - adc);
  float invKelvin = (1.0f / (NTC_NOMINAL_C + 273.15f)) +
                    (log(resistance / NTC_NOMINAL_OHMS) / NTC_BETA);
  float celsius = (1.0f / invKelvin) - 273.15f;
  if (celsius < -20.0f || celsius > 150.0f) return NAN;
  return celsius;
}

void setDryerOutputs(bool heater, bool fan) {
  dryer.heaterOn = heater;
  dryer.fanOn = fan;
  digitalWrite(HEATER_PIN, heater ? HIGH : LOW);
  digitalWrite(DRYER_FAN_PIN, fan ? HIGH : LOW);
}

void stopDryer() {
  dryer.active = false;
  dryer.cooling = dryer.ntcOK && dryer.temperature >= DRYER_COOLDOWN_C;
  setDryerOutputs(false, dryer.cooling);
  Serial.println("Secagem parada; resfriamento ativo ate 30C.");
}

void controlDryer() {
  dryer.temperature = readNTCTemperature();
  dryer.ntcOK = !isnan(dryer.temperature);
  if (!dryer.ntcOK) {
    setDryerOutputs(false, dryer.active || dryer.cooling);
    return;
  }

  if (dryer.active && !isnan(state.umidade) && state.umidade <= dryer.targetHumidity) {
    dryer.active = false;
    dryer.cooling = true;
    Serial.println("Secagem concluida: umidade alvo atingida.");
  }

  if (dryer.active) {
    bool heat = dryer.heaterOn;
    if (dryer.temperature <= dryer.targetTemperature - DRYER_HYSTERESIS_C) heat = true;
    if (dryer.temperature >= dryer.targetTemperature + DRYER_HYSTERESIS_C) heat = false;
    setDryerOutputs(heat, true);
  } else if (dryer.cooling) {
    if (dryer.temperature < DRYER_COOLDOWN_C) dryer.cooling = false;
    setDryerOutputs(false, dryer.cooling);
  } else {
    setDryerOutputs(false, false);
  }
}

void saveDryerSettings() {
  prefs.begin("dryer", false);
  prefs.putInt("temp", dryer.targetTemperature);
  prefs.putInt("humidity", dryer.targetHumidity);
  prefs.end();
}

void drawDryerStatus() {
  if (currentScreen != SCR_DRYER) return;
  tft.fillRect(8, 38, 304, 54, C_BG);
  tft.setTextSize(2);
  if (!dryer.ntcOK) {
    tft.setTextColor(C_RED);
    tft.setCursor(16, 56);
    tft.print("ERRO: NTC NAO CONECTADO");
    return;
  }

  tft.setTextColor(C_WHITE);
  tft.setCursor(16, 46);
  tft.print("NTC "); tft.print(dryer.temperature, 1); tft.print("C");
  tft.setCursor(174, 46);
  tft.print("UR ");
  if (isnan(state.umidade)) tft.print("--"); else tft.print((int)state.umidade);
  tft.print("%");
  tft.setTextSize(1);
  tft.setCursor(16, 74);
  tft.setTextColor(dryer.heaterOn ? C_ORANGE : C_GRAY);
  tft.print(dryer.heaterOn ? "AQUECEDOR ON" : "AQUECEDOR OFF");
  tft.setCursor(190, 74);
  tft.setTextColor(dryer.fanOn ? C_CYAN : C_GRAY);
  tft.print(dryer.fanOn ? "FAN ON" : "FAN OFF");
}

void drawDryer() {
  tft.fillScreen(C_BG);
  tft.fillRect(0, 0, 320, 36, C_DARK);
  tft.setTextColor(C_CYAN);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.print("SECAGEM FILAMENTO");
  drawDryerStatus();

  const char* labels[DRYER_COUNT] = {
    "INICIAR", "TEMPERATURA", "UMIDADE FIM", "< VOLTAR"
  };
  for (int i = 0; i < DRYER_COUNT; i++) {
    int y = 98 + i * 32;
    bool selected = i == dryerIdx;
    uint16_t bg = selected ? (dryerEditing ? C_ORANGE : C_ACTIVE) : C_DARK;
    tft.fillRect(10, y, 300, 27, bg);
    tft.setTextColor(selected ? C_WHITE : C_GRAY);
    tft.setTextSize(2);
    tft.setCursor(22, y + 5);
    if (i == 0) tft.print(dryer.active ? "PARAR" : labels[i]);
    else tft.print(labels[i]);
    tft.setCursor(248, y + 5);
    if (i == 1) { tft.print(dryer.targetTemperature); tft.print("C"); }
    if (i == 2) { tft.print(dryer.targetHumidity); tft.print("%"); }
  }
}

void drawToolActionMenu() {
  tft.fillScreen(C_BG);

  tft.fillRect(0, 0, 320, 36, C_DARK);
  tft.setTextColor(C_CYAN);
  tft.setTextSize(2);
  tft.fillCircle(20, 18, 8, toolColors[selectedToolIdx]);
  tft.setCursor(36, 10);
  tft.print("FILAMENTO T");
  tft.print(selectedToolIdx);

  tft.setTextColor(C_GRAY);
  tft.setTextSize(1);
  tft.setCursor(230, 13);
  tft.print(toolActionIdx + 1);
  tft.print("/");
  tft.print(TOOL_ACTION_COUNT);

  const int START_Y = 46;
  const int ITEM_H = 42;

  for (int i = 0; i < TOOL_ACTION_COUNT; i++) {
    int y = START_Y + i * ITEM_H;
    bool sel = (i == toolActionIdx);

    uint16_t bgColor = sel ? C_ACTIVE : C_DARK;
    uint16_t txtColor = sel ? C_WHITE : C_GRAY;

    tft.fillRoundRect(12, y, 296, 36, 7, bgColor);
    if (sel) {
      tft.drawRoundRect(12, y, 296, 36, 7, C_CYAN);
      tft.fillTriangle(22, y + 12, 22, y + 24, 34, y + 18, C_WHITE);
    }

    tft.setTextColor(txtColor);
    tft.setTextSize(2);
    tft.setCursor(48, y + 9);
    tft.print(toolActionLabels[i]);
  }

  tft.fillRect(0, 218, 320, 22, C_BG);
  tft.drawFastHLine(10, 218, 300, C_DARK);
  tft.setTextColor(C_GRAY);
  tft.setTextSize(1);
  tft.setCursor(55, 225);
  tft.print("GIRA=NAVEGA   OK=SELECIONA");
}

void drawColorSelect() {
  tft.fillScreen(C_BG);
  tft.fillRect(0, 0, 320, 32, C_DARK);
  tft.setTextColor(C_CYAN);
  tft.setTextSize(1);
  tft.setCursor(10, 8);
  tft.print("COR T");
  tft.print(colorToolIdx);
  tft.setTextColor(C_GRAY);
  tft.setCursor(200, 8);
  tft.print("OK = confirma");

  int cols = 5, bw = 56, bh = 50, padX = 8, padY = 40;

  for (int i = 0; i < COLOR_COUNT; i++) {
    int col = i % cols;
    int row = i / cols;
    int x   = padX + col * (bw + 4);
    int y   = padY + row * (bh + 4);
    bool sel = (i == colorPickIdx);

    if (sel) tft.drawRoundRect(x - 2, y - 2, bw + 4, bh + 4, 5, C_CYAN);
    tft.fillRoundRect(x, y, bw, bh - 16, 5, colorValues[i]);
    tft.setTextColor(sel ? C_WHITE : C_GRAY);
    tft.setTextSize(1);
    tft.setCursor(x + 2, y + bh - 14);
    char abbr[7];
    strncpy(abbr, colorNames[i], 6);
    abbr[6] = '\0';
    tft.print(abbr);
  }

  // Preview
  tft.fillRect(0, 195, 320, 45, C_DARK);
  tft.fillCircle(40, 218, 18, colorValues[colorPickIdx]);
  tft.fillCircle(40, 218,  7, C_DARK);
  tft.setTextColor(C_WHITE);
  tft.setTextSize(2);
  tft.setCursor(70, 210);
  tft.print(colorNames[colorPickIdx]);
}

// ═══════════════════════════════════════
// FEEDBACK — dois estados: enviando / resultado
// ═══════════════════════════════════════

void drawFeedback(const char* txt, int status) {
  tft.fillScreen(C_BG);

  // Moldura central
  tft.drawRoundRect(20, 40, 280, 160, 12, C_DARK);
  tft.drawRoundRect(22, 42, 276, 156, 12, C_DARK);

  if (status == 0) {
    // Enviando
    tft.setTextColor(C_GRAY);
    tft.setTextSize(2);
    tft.setCursor(82, 72);
    tft.print("ENVIANDO");

    tft.fillCircle(110, 120, 6, C_CYAN);
    tft.fillCircle(140, 120, 6, C_CYAN);
    tft.fillCircle(170, 120, 6, C_CYAN);
  }
  else if (status == 1) {
    // OK
    tft.setTextColor(C_GREEN);
    tft.setTextSize(4);
    tft.setCursor(110, 60);
    tft.print("OK");

    tft.fillCircle(160, 125, 28, C_GREEN);
    tft.drawLine(148, 126, 158, 138, C_BLACK);
    tft.drawLine(158, 138, 178, 110, C_BLACK);
  }
  else {
    // Erro
    tft.setTextColor(C_RED);
    tft.setTextSize(3);
    tft.setCursor(92, 65);
    tft.print("ERRO");

    tft.drawCircle(160, 125, 28, C_RED);
    tft.drawLine(145, 110, 175, 140, C_RED);
    tft.drawLine(175, 110, 145, 140, C_RED);
  }

  tft.setTextColor(C_WHITE);
  tft.setTextSize(3);

  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(txt, 0, 0, &x1, &y1, &w, &h);

  int tx = (320 - w) / 2;
  if (tx < 0) tx = 0;

  tft.setCursor(tx, 170);
  tft.print(txt);
}

void drawLoadAlert() {
  tft.fillScreen(C_BG);

  tft.drawRoundRect(18, 28, 284, 184, 12, C_ORANGE);
  tft.drawRoundRect(21, 31, 278, 178, 12, C_ORANGE);

  tft.setTextColor(C_ORANGE);
  tft.setTextSize(5);
  tft.setCursor(146, 48);
  tft.print("!");

  tft.drawCircle(160, 70, 32, C_ORANGE);

  tft.setTextColor(C_WHITE);
  tft.setTextSize(2);
  tft.setCursor(47, 118);
  tft.print("FILAMENTO QUEBROU");
  tft.setCursor(72, 144);
  tft.print("NO HOTEND OU");
  tft.setCursor(47, 170);
  tft.print("SENSOR COM DEFEITO");
}

void drawHotendLoadedAlert() {
  tft.fillScreen(C_BG);

  tft.drawRoundRect(18, 28, 284, 184, 12, C_CYAN);
  tft.drawRoundRect(21, 31, 278, 178, 12, C_GREEN);

  tft.fillCircle(160, 70, 32, C_GREEN);
  tft.setTextColor(C_BLACK);
  tft.setTextSize(4);
  tft.setCursor(148, 54);
  tft.print("i");

  tft.setTextColor(C_CYAN);
  tft.setTextSize(2);
  tft.setCursor(47, 122);
  tft.print("FILAMENTO JA ESTA");
  tft.setTextColor(C_GREEN);
  tft.setCursor(86, 154);
  tft.print("NO HOTEND");
}
