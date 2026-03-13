# 3D Chameleon Mk4 — Creality K1 Max

Sistema de troca automática de filamento multi-material para Creality K1 Max rodando Klipper/Moonraker.

Controlado por um Raspberry Pi Pico com dois drivers TMC2209, sensores NC e comunicação via USB Serial.

---

## 🖨️ Compatibilidade

Este projeto foi desenvolvido para a **Creality K1 Max** mas pode ser adaptado para **qualquer impressora rodando Klipper**.

Para usar em outra impressora basta ajustar dois macros no `chameleon_macros.cfg`:

| Macro | O que ajustar |
|-------|--------------|
| `Fil_Cut` | Posição do cortador de filamento (`X Y`) e movimentos de corte |
| `Purga` | Posição de purga (`X Y Z`) e movimentos de limpeza do bico |

Todo o restante do sistema — Pico, bridge, sensores e lógica de troca — funciona sem modificações.

---

## 📋 Lista de Materiais

| Qtd | Componente | Modelo / Especificação |
|-----|-----------|----------------------|
| 10 | Sensor de filamento | KW2B (micro switch NC) |
| 2  | Driver de motor | A4988 / TMC2209 / TMC2208 Standalone |
| 2  | Shield Step Driver | Placa de expansão para driver |
| 1  | Microcontrolador | Raspberry Pi Pico (RP2040) |
| 10 | Esfera de aço | 4mm de diâmetro |
| 2  | Motor de passo | Nema 17 |
| 1  | Regulador Step Down | Mini360 / LM2596 — entrada 12V saída 5V — alimentação dos drivers |
| 4  | Parafuso M3 x 34.5mm | Fuso — fixação do Chameleon na base |
| 1  | Parafuso M3 x 15mm | Fixação do home do seletor |
| 2  | Parafuso M3 x 12mm | Fixação do Hub 4x1 |
| 3  | Inserto metálico M3 x 4mm | Para fixação nas peças impressas em 3D |
| 1  | Tubo PTFE | 1 metro — conexão entre hub e hotend |
| 4  | Tubo PTFE x 75.05mm | Conexão do Hub 4x1 ao Chameleon |

> O switch hotend original da K1 Max é transferido da placa da impressora para o GPIO9 do Pico.

---

## 🖨️ Peças Impressas em 3D

Peças customizadas desenvolvidas para facilitar a montagem:

| Peça | Descrição | Link |
|------|-----------|------|
| K1 Sugar Skull Extruder with Filament Cutter | Extrusora com cortador de filamento para série K1 — necessária para o sistema de corte funcionar | [Printables](https://www.printables.com/model/1293244-k1-sugar-skull-extruder-with-filament-cutter) |
| Suporte do rolamento (modificado) | Alteração na estrutura original do Chameleon que facilita a inserção do filamento | [Printables](https://www.printables.com/model/1632259-atualizacao-para-3d-chamaleon) |
| Suporte dos sensores | Suporte para fixar os sensores KW2B no Chameleon | [Printables](https://www.printables.com/model/1632259-atualizacao-para-3d-chamaleon) |
| TurtleNeck Buffer | Buffer de filamento usado neste projeto — projeto open source da ArmoredTurtle | [GitHub](https://github.com/ArmoredTurtle/TurtleNeck) |

> Arquivos STL customizados disponíveis no [Printables](https://www.printables.com/model/1632259-atualizacao-para-3d-chamaleon).

---

## 🔌 Pinout — Raspberry Pi Pico

| GPIO | Pino Físico | Função | Lógica |
|------|------------|--------|--------|
| GPIO 0  | Pino 1  | Sensor Entrada T0 (NC) | HIGH=presente / LOW=vazio |
| GPIO 1  | Pino 2  | Sensor Entrada T1 (NC) | HIGH=presente / LOW=vazio |
| GPIO 2  | Pino 4  | Sensor Entrada T2 (NC) | HIGH=presente / LOW=vazio |
| GPIO 3  | Pino 5  | Sensor Entrada T3 (NC) | HIGH=presente / LOW=vazio |
| GPIO 8  | Pino 11 | Endstop Seletor (NO)   | LOW=endstop acionado |
| GPIO 9  | Pino 12 | Sensor Hotend K1 (NO)  | HIGH=filamento presente |
| GPIO 10 | Pino 14 | Buffer Vazio (NC)       | HIGH=buffer vazio |
| GPIO 11 | Pino 15 | Buffer Cheio (NC)       | HIGH=buffer cheio |
| GPIO 12 | Pino 16 | Sensor Saída Único (NC) | LOW=filamento saiu — compartilhado T0-T3 |
| GPIO 16 | Pino 21 | Seletor STEP            | Pulso = 1 passo |
| GPIO 17 | Pino 22 | Seletor DIR             | HIGH=CW / LOW=CCW |
| GPIO 18 | Pino 24 | Seletor EN              | LOW=ativo / HIGH=livre |
| GPIO 19 | Pino 25 | Alimentador STEP        | Pulso = 1 passo |
| GPIO 20 | Pino 26 | Alimentador DIR         | HIGH=load / LOW=unload |
| GPIO 21 | Pino 27 | Alimentador EN          | LOW=ativo / HIGH=livre |
| 3.3V    | Pino 36 | VCC Sensores            | Alimentação sensores |
| GND     | Pino 38 | GND Comum               | Referência |

---

## 📁 Arquivos

| Arquivo | Descrição |
|---------|-----------|
| `chameleon_pico.ino` | Firmware do Raspberry Pi Pico — controle dos motores e sensores |
| `chameleon_bridge.py` | Bridge Python — comunicação serial entre Klipper e Pico |
| `chameleon_macros.cfg` | Macros Klipper — sequências de troca, purga e monitoramento |

---

## ⚙️ Instalação

### 1. Firmware do Pico

- Instale o [Arduino IDE](https://www.arduino.cc/en/software)
- Adicione o suporte ao Raspberry Pi Pico: `https://github.com/earlephilhower/arduino-pico`
- Abra `chameleon_pico.ino` e faça upload para o Pico

### 2. Arquivos na K1 Max

Copie via Fluidd/Mainsail para `/usr/data/printer_data/config/`:
```
chameleon_bridge.py
chameleon_macros.cfg
```

### 3. Instalar dependência

Acesse a K1 Max via SSH e execute:
```bash
pip3 install pyserial
```

### 4. Verificar porta serial

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```
Se a porta for diferente de `/dev/ttyACM0`, ajuste `ARDUINO_PORT` no `chameleon_bridge.py`.

### 5. Configurar printer.cfg

Adicione no `printer.cfg`:
```ini
[include chameleon_macros.cfg]
```

### 6. Gcode do fatiador (Creality Print)

**Start Gcode:**
```
START_PRINT EXTRUDER_TEMP=[nozzle_temperature_initial_layer] BED_TEMP=[bed_temperature_initial_layer_single]
CHAMELEON_PRINT_START TOOL=[initial_no_support_extruder]
```

**End Gcode:**
```
CHAMELEON_PRINT_END
END_PRINT
```

---

## 🔄 Fluxo de Funcionamento

### Início da impressão
```
CHAMELEON_PRINT_START
  → HOME seletor (endstop GPIO8)
  → Seleciona gate inicial
  → LOAD → empurra até GPIO9 acionar
  → Enche buffer até switch cheio
  → Monitoramento do buffer ativa
```

### Troca de filamento (ex: T0 → T1)
```
1. BUFFER_DRAIN    → esvazia buffer, ativa modo drain
2. Z hop +5mm      → sobe para não bater na peça
3. Move cortador   → X34 Y280
4. Retrai 40mm + corte duplo + retrai 70mm
5. Move purga      → X304 Y250 Z0.2
6. UNLOAD          → retrai até sensor saída GPIO12 + 40mm extra
7. T1              → seletor move para gate 1
8. LOAD            → empurra até GPIO9 acionar → enche buffer
9. Purga           → 50mm + limpeza silicone
10. RESTORE        → volta para posição XYZ da peça
```

### Detecção de problemas
| Evento | Ação |
|--------|------|
| Filamento acabou (sensor entrada LOW) | Pico envia `RUNOUT T{n}` → Bridge → `CHAMELEON_RUNOUT` → `PAUSE` |
| Load falhou (GPIO9 não acionou) | Pico envia `ERRO` → Bridge → `CHAMELEON_LOAD_FAIL` → `PAUSE` |

---

## 🖥️ Comandos Serial (115200 baud)

| Comando | Descrição |
|---------|-----------|
| `T0` / `T1` / `T2` / `T3` | Seleciona gate |
| `HOME` | Home do seletor pelo endstop GPIO8 |
| `LOAD` | Empurra filamento até GPIO9 acionar, enche buffer |
| `LOAD <mm>` | Empurra até sensor hotend + distância extra |
| `UNLOAD` | Retrai até sensor saída GPIO12 + 40mm extra |
| `UNLOAD_RETRACAO <mm>` | Retrai distância fixa |
| `BUFFER_DRAIN` | Esvazia buffer, mantém vazio até UNLOAD |
| `START_PRINT` | Ativa monitoramento do buffer manualmente |
| `STOP_PRINT` | Desativa monitoramento do buffer |
| `STATUS` | Mostra estado atual de todos os sensores |

---

## ☕ Apoie o Projeto

Se este projeto te ajudou, considere fazer uma doação para apoiar o desenvolvimento!

**PIX — Mercado Pago**
```
(47) 99678-1505
Luan Euclides Correia
```

---

## 🙏 Créditos

**Luan Euclides Correia** — Criador do projeto, desenvolvimento do hardware, montagem, testes e validação em produção.

*Firmware, macros e bridge desenvolvidos com assistência do [Claude](https://claude.ai) (Anthropic).*

Baseado no projeto [3D Chameleon Mk4](https://www.3dchameleon.com/) adaptado para Creality K1 Max com Klipper/Moonraker.

Buffer de filamento: [TurtleNeck](https://github.com/ArmoredTurtle/TurtleNeck) by ArmoredTurtle.

---

## 📄 Licença

MIT License — Copyright (c) 2026 Luan Euclides Correia

Veja o arquivo [LICENSE](LICENSE) para detalhes completos.
