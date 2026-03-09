#!/usr/bin/env python3
########################################################################################################################
# chameleon_bridge.py
# Ponte entre K1 Max (Klipper) e Pico via USB Serial
#
# Uso pelo Klipper via gcode_shell_command:
#   RUN_SHELL_COMMAND CMD=chameleon_cmd PARAMS="HOME"
#   RUN_SHELL_COMMAND CMD=chameleon_cmd PARAMS="T0"
#   RUN_SHELL_COMMAND CMD=chameleon_cmd PARAMS="LOAD"
#   RUN_SHELL_COMMAND CMD=chameleon_cmd PARAMS="UNLOAD"
#   RUN_SHELL_COMMAND CMD=chameleon_cmd PARAMS="UNLOAD_RETRACAO 1150"
#   RUN_SHELL_COMMAND CMD=chameleon_cmd PARAMS="BUFFER_DRAIN"
#   RUN_SHELL_COMMAND CMD=chameleon_cmd PARAMS="START_PRINT"
#   RUN_SHELL_COMMAND CMD=chameleon_cmd PARAMS="STOP_PRINT"
#   RUN_SHELL_COMMAND CMD=chameleon_cmd PARAMS="STATUS"
#
# Instalação na K1 Max:
#   1. Copiar para /usr/data/printer_data/config/chameleon_bridge.py
#   2. pip3 install pyserial
#   3. Verificar porta: ls /dev/ttyACM* /dev/ttyUSB*
#   4. Ajustar ARDUINO_PORT abaixo se necessário
########################################################################################################################

import serial
import sys
import time
import os

# ══════════════════════════════════════════════════════════════════════════════
# CONFIGURAÇÃO
# ══════════════════════════════════════════════════════════════════════════════

ARDUINO_PORT = "/dev/ttyACM0"
BAUD_RATE    = 115200
TIMEOUT      = 60       # segundos aguardando resposta do Pico
BOOT_WAIT    = 0.1      # segundos após abrir serial (Pico não reseta)

# DONE_KEYWORDS — respostas do .ino que indicam conclusão do comando
DONE_KEYWORDS = [
    "HOME OK",
    "T0 OK", "T1 OK", "T2 OK", "T3 OK",
    "SELETOR OK",
    "LOAD OK",
    "UNLOAD OK:",
    "UNLOAD RETRACAO OK",
    "IDLE OK",
    "BUFFER DRAIN OK",
    "BUFFER REABASTECIDO:",
    "PRINT STARTED",
    "PRINT STOPPED",
    "STATUS OK",
    "ERRO",
]

# ══════════════════════════════════════════════════════════════════════════════
# SEND COMMAND — abre serial, envia comando, aguarda DONE_KEYWORD
# ══════════════════════════════════════════════════════════════════════════════

def send_command(command):
    try:
        ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
        time.sleep(BOOT_WAIT)
        ser.flushInput()
        ser.flushOutput()

        cmd_bytes = (command.strip() + "\n").encode("utf-8")
        ser.write(cmd_bytes)
        print(f">> Enviado: {command}", flush=True)

        start_time = time.time()
        has_error  = False
        done       = False

        while not done and (time.time() - start_time) < TIMEOUT:
            if ser.in_waiting > 0:
                raw  = ser.readline()
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                print(f"<< Pico: {line}", flush=True)
                line_upper = line.upper()

                # Runout detectado pelo Pico — chama macro no Klipper via Moonraker
                if line_upper.startswith("RUNOUT T"):
                    tool = line_upper.replace("RUNOUT T", "").strip()
                    print(f">> RUNOUT T{tool} detectado! Avisando Klipper...", flush=True)
                    os.system(
                        f'curl -s -X POST "http://localhost:7125/printer/gcode/script" '
                        f'-H "Content-Type: application/json" '
                        f'-d \'{{"script":"CHAMELEON_RUNOUT TOOL={tool}"}}\' '
                        f'> /dev/null 2>&1 &'
                    )

                for keyword in DONE_KEYWORDS:
                    if keyword in line_upper:
                        done = True
                        if "ERRO" in line_upper:
                            has_error = True
                            # Se erro foi no LOAD — avisa Klipper para pausar
                            if command.startswith("LOAD"):
                                print(">> LOAD falhou! Avisando Klipper...", flush=True)
                                os.system(
                                    'curl -s -X POST "http://localhost:7125/printer/gcode/script" '
                                    '-H "Content-Type: application/json" '
                                    '-d \'{"script":"CHAMELEON_LOAD_FAIL"}\' '
                                    '> /dev/null 2>&1 &'
                                )
                        break
            else:
                time.sleep(0.02)

        ser.close()

        if not done:
            print(f"TIMEOUT: Sem resposta do Pico apos {TIMEOUT}s para: {command}", flush=True)
            sys.exit(2)
        if has_error:
            print(f"FALHA: Pico reportou erro para: {command}", flush=True)
            sys.exit(1)

        print("Comando concluido com sucesso.", flush=True)
        sys.exit(0)

    except serial.SerialException as e:
        print(f"ERRO SERIAL: {e}", flush=True)
        sys.exit(1)
    except KeyboardInterrupt:
        print("Interrompido.", flush=True)
        sys.exit(1)
    except Exception as e:
        print(f"ERRO INESPERADO: {e}", flush=True)
        sys.exit(1)


# ══════════════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Uso: chameleon_bridge.py <COMANDO> [parametro]", flush=True)
        print("Comandos: HOME IDLE T0-T3 LOAD LOAD <mm> UNLOAD UNLOAD_RETRACAO <mm> BUFFER_DRAIN START_PRINT STOP_PRINT STATUS", flush=True)
        sys.exit(1)

    command = " ".join(sys.argv[1:]).upper()
    send_command(command)
