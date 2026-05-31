/* ═══════════════════════════════════════════════════════════════════════════
   Firmware 3DChameleon Mk4 — Raspberry Pi Pico + 2x TMC2209 Standalone
   Comunicação: USB Serial nativo — 115200 baud

   Pinout:
   ── Sensores Entrada de Filamento (NC) ──
     T0 entrada : GPIO 0  (Pino 1)
     T1 entrada : GPIO 1  (Pino 2)
     T2 entrada : GPIO 2  (Pino 4)
     T3 entrada : GPIO 3  (Pino 5)

   ── Sensor Saida Unico (NC) — compartilhado T0-T3 ──
     Saida      : GPIO 12 (Pino 16)

   ── Endstop Seletor (C ligado ao pino, NC) ──
     Endstop    : GPIO 8  (Pino 11)

   ── Sensor Hotend (switch K1 transferido, NC) ──
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

   Lógica sensores NC com pullup:
     HIGH = filamento presente
     LOW  = sem filamento

   Comandos Serial (115200 baud):
     T0-T3                → seleciona gate
     HOME                 → home do seletor pelo endstop GPIO8
     IDLE                 → move seletor para posição idle
     LOAD                 → empurra até GPIO9 acionar, ativa buffer
     LOAD <mm>            → empurra até sensor hotend + mm extra
     UNLOAD               → descarrega até sensor saída (sai do BUFFER_DRAIN)
     UNLOAD_RETRACAO <mm> → descarrega distância fixa em mm
     BUFFER_DRAIN         → esvazia buffer se cheio, mantém vazio até UNLOAD
     START_PRINT          → ativa monitoramento do buffer
     STOP_PRINT           → desativa monitoramento do buffer
     STATUS               → mostra estado atual
═══════════════════════════════════════════════════════════════════════════ */

#include <EEPROM.h>

// ── Sensores Entrada de Filamento (NC) ────────────────
#define SENSOR_ENTRADA_T0   0
#define SENSOR_ENTRADA_T1   1
#define SENSOR_ENTRADA_T2   2
#define SENSOR_ENTRADA_T3   3

// ── Sensor Saida Unico (NC) — compartilhado T0-T3 ─────
#define SENSOR_SAIDA        12

// ── Endstop Seletor ───────────────────────────────────
#define ENDSTOP_SELETOR     8

// ── Sensor Hotend (NC) — switch K1 transferido ────────
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
const float STEPS_PER_MM     = 151.0;
const int   SPEED_DELAY      = 170;    // µs — buffer normal
const int   SPEED_DELAY_FAST = 28;     // µs — load/unload rápido
const int   SEL_SPEED_DELAY  = 60;     // µs — seletor
const int   DEFAULT_BACKOFF  = 5;      // passos recuo após home

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

// ── Serial ────────────────────────────────────────────
String serialBuffer    = "";
bool   commandReceived = false;

// ── EEPROM ────────────────────────────────────────────
#define EEPROM_TOOL_ADDR 0

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

// Sensor saida unico para todos os gates
int sensorSaidaPin(int tool) { return SENSOR_SAIDA; }

bool filamentoPresente(int pin) { return digitalRead(pin) == HIGH; }
bool bufferVazio()  { return digitalRead(BUFFER_VAZIO_PIN) == HIGH; }
bool bufferCheio()  { return digitalRead(BUFFER_CHEIO_PIN) == HIGH; }

bool direcaoLoad(int tool)   { return (tool < 2) ? CW  : CCW; }
bool direcaoUnload(int tool) { return (tool < 2) ? CCW : CW;  }

void salvarTool(int tool) { EEPROM.write(EEPROM_TOOL_ADDR, tool + 1); }
int  carregarTool()       { return EEPROM.read(EEPROM_TOOL_ADDR) - 1; }

void extStep(int spd) {
  digitalWrite(EXT_STEP, HIGH); delayMicroseconds(spd);
  digitalWrite(EXT_STEP, LOW);  delayMicroseconds(spd);
}

// ═══════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("3DChameleon Pico Ready");

  pinMode(SEL_STEP, OUTPUT); pinMode(SEL_DIR, OUTPUT); pinMode(SEL_EN, OUTPUT);
  pinMode(EXT_STEP, OUTPUT); pinMode(EXT_DIR, OUTPUT); pinMode(EXT_EN, OUTPUT);
  digitalWrite(SEL_EN, HIGH);
  digitalWrite(EXT_EN, HIGH);

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

  // Monitoramento do buffer durante impressão normal
  if (imprimindo && !bufferDrainMode) {
    unsigned long now = millis();
    if (now - lastBufferCheck >= BUFFER_CHECK_MS) {
      monitorarBuffer();
      lastBufferCheck = now;
    }
  }

  // Modo BUFFER_DRAIN — mantém buffer vazio até UNLOAD
  if (bufferDrainMode) {
    executarBufferDrain();
  }

  delay(5);
}

// ═══════════════════════════════════════════════════════
// PROCESSAR COMANDOS
// ═══════════════════════════════════════════════════════

void processarComando(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "T0") { selecionarTool(0); return; }
  if (cmd == "T1") { selecionarTool(1); return; }
  if (cmd == "T2") { selecionarTool(2); return; }
  if (cmd == "T3") { selecionarTool(3); return; }

  if (cmd == "HOME") {
    homeSelector();
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
    imprimindo = false;   // para monitoramento do buffer durante unload
    if (currentExtruder < 0) { Serial.println("ERRO: Selecione tool primeiro"); return; }
    unloadAteSensor(currentExtruder);
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
    Serial.println("Print stopped");
    return;
  }

  if (cmd == "STATUS") {
    printStatus();
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
  digitalWrite(SEL_EN, HIGH);
}

// ═══════════════════════════════════════════════════════
// HOME DO SELETOR — endstop GPIO8
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
    digitalWrite(SEL_EN, HIGH);
    return;
  }

  digitalWrite(SEL_DIR, CCW);
  for (int i = 0; i < DEFAULT_BACKOFF * MICROSTEPS; i++) {
    digitalWrite(SEL_STEP, HIGH); delayMicroseconds(SEL_SPEED_DELAY);
    digitalWrite(SEL_STEP, LOW);  delayMicroseconds(SEL_SPEED_DELAY);
  }

  digitalWrite(SEL_EN, HIGH);
  currentExtruder = 0;
  Serial.println("Home OK");
}

// ═══════════════════════════════════════════════════════
// MOVER SELETOR ENTRE GATES
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
// BUFFER_DRAIN — esvazia buffer se cheio, mantém vazio até UNLOAD
//
// Fluxo:
//   1. Se buffer cheio → esvazia até sensor vazio
//   2. Ativa bufferDrainMode = true → responde "Buffer Drain OK"
//   3. loop() chama executarBufferDrain() continuamente:
//      → buffer cheio aciona → esvazia até vazio
//      → UNLOAD recebido → sai do modo e faz unload normal
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
          if (currentExtruder >= 0) unloadAteSensor(currentExtruder);
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
// UNLOAD — retrai até sensor de saída do gate
// ═══════════════════════════════════════════════════════

void unloadAteSensor(int tool) {
  Serial.print("Unload T"); Serial.println(tool);

  digitalWrite(EXT_EN, LOW);
  digitalWrite(EXT_DIR, direcaoUnload(tool));

  long steps    = 0;
  long maxSteps = (long)(4000.0 * STEPS_PER_MM);
  bool ok       = false;

  // Retrai até sensor saida GPIO12 detectar que filamento saiu (LOW)
  while (steps < maxSteps) {
    if (!filamentoPresente(SENSOR_SAIDA)) { ok = true; break; }
    extStep(SPEED_DELAY_FAST);
    steps++;
  }

  if (!ok) {
    digitalWrite(EXT_EN, HIGH);
    Serial.println("ERRO: Sensor saida nao detectou filamento saindo!");
    return;
  }

  Serial.print("Sensor saida OK: ");
  Serial.print((float)steps / STEPS_PER_MM);
  Serial.println("mm");

  // Retrai mais 40mm para liberar caminho para proximo filamento
  long extraSteps = (long)(40.0 * STEPS_PER_MM);
  for (long i = 0; i < extraSteps; i++) extStep(SPEED_DELAY_FAST);

  digitalWrite(EXT_EN, HIGH);
  Serial.print("Unload OK: ");
  Serial.print((float)(steps + extraSteps) / STEPS_PER_MM);
  Serial.println("mm");
}

// ═══════════════════════════════════════════════════════
// UNLOAD_RETRACAO — retrai distância fixa em mm
// ═══════════════════════════════════════════════════════

void unloadRetracao(int tool, float mm) {
  Serial.print("Unload retracao T"); Serial.print(tool);
  Serial.print(" "); Serial.print(mm); Serial.println("mm");

  long steps = (long)(mm * STEPS_PER_MM);

  digitalWrite(EXT_EN, LOW);
  digitalWrite(EXT_DIR, direcaoUnload(tool));

  for (long x = 0; x < steps; x++) extStep(SPEED_DELAY_FAST);

  digitalWrite(EXT_EN, HIGH);
  Serial.println("Unload retracao OK");
}

// ═══════════════════════════════════════════════════════
// LOAD — empurra até GPIO9 acionar
// Se filamento já presente retorna imediatamente
// Após Load OK ativa buffer automaticamente (imprimindo = true)
// ═══════════════════════════════════════════════════════

void loadContinuo(int tool) {
  Serial.print("Load continuo T"); Serial.println(tool);

  if (filamentoPresente(SENSOR_HOTEND)) {
    Serial.println("Load OK: filamento ja presente");
    encherBuffer(tool);
    imprimindo = true;
    return;
  }

  digitalWrite(EXT_EN, LOW);
  digitalWrite(EXT_DIR, direcaoLoad(tool));

  long steps    = 0;
  long maxSteps = (long)(4000.0 * STEPS_PER_MM);
  bool ok       = false;

  while (steps < maxSteps) {
    if (filamentoPresente(SENSOR_HOTEND)) { ok = true; break; }
    extStep(SPEED_DELAY_FAST);
    steps++;
  }

  digitalWrite(EXT_EN, HIGH);

  if (ok) {
    Serial.print("Load OK: ");
    Serial.print((float)steps / STEPS_PER_MM);
    Serial.println("mm");
    encherBuffer(tool);
    imprimindo = true;
  } else {
    Serial.println("ERRO: Load atingiu limite maximo sem sensor!");
  }
}

// Enche buffer até switch cheio acionar — chamado após Load OK
void encherBuffer(int tool) {
  if (bufferCheio()) {
    Serial.println("Buffer ja cheio.");
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

  digitalWrite(EXT_EN, HIGH);
  Serial.print("Buffer cheio: ");
  Serial.print((float)steps / STEPS_PER_MM);
  Serial.println("mm");
}

// ═══════════════════════════════════════════════════════
// LOAD ATÉ SENSOR HOTEND + distância extra
// ═══════════════════════════════════════════════════════

void loadAteSensorHotend(int tool, float extraMM) {
  Serial.print("Load T"); Serial.print(tool);
  Serial.print(" extra: "); Serial.print(extraMM); Serial.println("mm");

  digitalWrite(EXT_EN, LOW);
  digitalWrite(EXT_DIR, direcaoLoad(tool));

  long steps    = 0;
  long maxSteps = (long)(4000.0 * STEPS_PER_MM);
  bool ok       = false;

  while (steps < maxSteps) {
    if (filamentoPresente(SENSOR_HOTEND)) { ok = true; break; }
    extStep(SPEED_DELAY_FAST);
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
    for (long i = 0; i < extraSteps; i++) extStep(SPEED_DELAY_FAST);
    Serial.print("Extra "); Serial.print(extraMM); Serial.println("mm OK");
  }

  digitalWrite(EXT_EN, HIGH);
  Serial.println("Load OK");
}

// ═══════════════════════════════════════════════════════
// MONITOR BUFFER — durante impressão normal
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

  // Runout — sensor de entrada do gate ativo ficou LOW
  if (!filamentoPresente(sensorEntradaPin(lastExtruder))) {
    Serial.print("RUNOUT T"); Serial.println(lastExtruder);
  }

  // Reabastece quando vazio e filamento no hotend
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

  digitalWrite(EXT_EN, HIGH);

  if (cheio) {
    Serial.print("Buffer reabastecido: ");
    Serial.print((float)steps / STEPS_PER_MM);
    Serial.println("mm");
  } else {
    Serial.println("ERRO: Buffer nao encheu!");
  }
}
