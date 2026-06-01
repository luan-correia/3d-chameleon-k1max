#!/usr/bin/env python3
"""
3D Chameleon daemon for Klipper.

Keep this process running in the printer. It keeps the Pico USB serial port
open and exposes a small local HTTP API for Klipper shell commands.
"""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import threading
import time
import urllib.parse
import urllib.request

import serial


VERSION = "2026-05-31-initial-purge-170"
SERIAL_PORT = os.environ.get("CHAMELEON_SERIAL_PORT", "/dev/ttyACM0")
BAUD_RATE = int(os.environ.get("CHAMELEON_BAUD_RATE", "115200"))
SERIAL_TIMEOUT = float(os.environ.get("CHAMELEON_SERIAL_TIMEOUT", "0.2"))
COMMAND_TIMEOUT = float(os.environ.get("CHAMELEON_COMMAND_TIMEOUT", "60"))
HOST = os.environ.get("CHAMELEON_DAEMON_HOST", "127.0.0.1")
PORT = int(os.environ.get("CHAMELEON_DAEMON_PORT", "8765"))
MOONRAKER_URL = os.environ.get(
    "CHAMELEON_MOONRAKER_URL",
    "http://127.0.0.1:7125/printer/gcode/script",
)

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


def done_keywords_for(command):
    if command in ("T0", "T1", "T2", "T3"):
        return [f"{command} OK", "ERRO"]
    if command == "HOME":
        return ["HOME OK", "ERRO"]
    if command == "IDLE":
        return ["IDLE OK", "ERRO"]
    if command == "BUFFER_DRAIN":
        return ["BUFFER DRAIN OK", "ERRO"]
    if command == "UNLOAD":
        return ["UNLOAD OK:", "ERRO"]
    if command.startswith("UNLOAD_RETRACAO "):
        return ["UNLOAD RETRACAO OK", "ERRO"]
    if command == "LOAD":
        return ["BUFFER CHEIO:", "BUFFER JA CHEIO.", "ERRO"]
    if command == "LOAD_MANUAL":
        return ["BUFFER CHEIO:", "BUFFER JA CHEIO.", "ERRO"]
    if command.startswith("LOAD "):
        return ["LOAD OK", "ERRO"]
    if command == "START_PRINT":
        return ["PRINT STARTED", "ERRO"]
    if command == "STOP_PRINT":
        return ["PRINT STOPPED", "ERRO"]
    if command == "STATUS":
        return ["STATUS OK", "ERRO"]
    return DONE_KEYWORDS


def post_klipper_gcode(script):
    payload = json.dumps({"script": script}).encode("utf-8")
    request = urllib.request.Request(
        MOONRAKER_URL,
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    urllib.request.urlopen(request, timeout=3).read()


def should_pause_on_failure(command):
    return command == "LOAD" or command == "LOAD_MANUAL" or command.startswith("LOAD ")


class ChameleonSerial:
    def __init__(self):
        self.serial = None
        self.lock = threading.Lock()
        self.stop_event = threading.Event()

    def open(self):
        if self.serial and self.serial.is_open:
            return

        self.serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT)
        time.sleep(0.1)
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()
        print(f"Serial aberta em {SERIAL_PORT} @ {BAUD_RATE}", flush=True)

    def close(self):
        if self.serial:
            self.serial.close()

    def _read_line(self):
        raw = self.serial.readline()
        if not raw:
            return ""
        return raw.decode("utf-8", errors="ignore").strip()

    def _handle_async_line(self, line):
        upper = line.upper()
        if not upper.startswith("RUNOUT T"):
            return

        tool = upper.replace("RUNOUT T", "").strip()
        if not tool.isdigit():
            print(f"RUNOUT ignorado, tool invalida: {line}", flush=True)
            return

        try:
            post_klipper_gcode(f"CHAMELEON_RUNOUT TOOL={tool}")
            print(f"RUNOUT T{tool} enviado ao Klipper", flush=True)
        except Exception as exc:
            print(f"Falha ao enviar RUNOUT T{tool}: {exc}", flush=True)

    def _pause_klipper(self, reason):
        safe_reason = reason.replace('"', "'")[:120]
        try:
            post_klipper_gcode(f'RESPOND TYPE=error MSG="Chameleon: {safe_reason}"\nPAUSE')
            print(f"PAUSE enviado ao Klipper: {safe_reason}", flush=True)
        except Exception as exc:
            print(f"Falha ao enviar PAUSE ao Klipper: {exc}", flush=True)

    def _send_serial_command_locked(self, command):
        self.open()
        self.serial.reset_input_buffer()
        self.serial.write((command + "\n").encode("utf-8"))
        self.serial.flush()
        print(f">> Enviado: {command}", flush=True)

        start_time = time.time()
        lines = []
        expected_done = done_keywords_for(command)

        while time.time() - start_time < COMMAND_TIMEOUT:
            line = self._read_line()
            if not line:
                continue

            lines.append(line)
            print(f"<< Pico: {line}", flush=True)
            self._handle_async_line(line)

            upper = line.upper()
            for keyword in expected_done:
                if keyword in upper:
                    return {
                        "ok": "ERRO" not in upper,
                        "command": command,
                        "lines": lines,
                        "error": line if "ERRO" in upper else "",
                    }

        return {
            "ok": False,
            "command": command,
            "lines": lines,
            "error": f"TIMEOUT apos {COMMAND_TIMEOUT}s",
        }

    def ensure_loaded(self, command, purge_macro="Purga", usage="ENSURE_LOADED <tool>"):
        parts = command.split()
        if len(parts) < 2 or not parts[1].isdigit():
            return {"ok": False, "command": command, "lines": [], "error": f"Uso: {usage}"}

        tool = int(parts[1])
        if tool < 0 or tool > 3:
            return {"ok": False, "command": command, "lines": [], "error": "Tool invalida"}

        with self.lock:
            all_lines = []

            select_result = self._send_serial_command_locked(f"T{tool}")
            all_lines.extend(select_result.get("lines", []))
            if not select_result["ok"]:
                return {**select_result, "command": command, "lines": all_lines}

            load_result = self._send_serial_command_locked("LOAD")
            all_lines.extend(load_result.get("lines", []))

            if not load_result["ok"]:
                self._pause_klipper(load_result.get("error", "Falha no LOAD"))
                return {**load_result, "command": command, "lines": all_lines}

            already_loaded = any("FILAMENTO JA PRESENTE" in line.upper() for line in load_result["lines"])
            if already_loaded:
                print(f"T{tool} ja estava carregada; {purge_macro} ignorada.", flush=True)
            else:
                try:
                    post_klipper_gcode(purge_macro)
                    print(f"T{tool} carregada agora; {purge_macro} enviada ao Klipper.", flush=True)
                except Exception as exc:
                    return {
                        "ok": False,
                        "command": command,
                        "lines": all_lines,
                        "error": f"Falha ao chamar {purge_macro}: {exc}",
                    }

            return {
                "ok": True,
                "command": command,
                "lines": all_lines,
                "already_loaded": already_loaded,
                "purged": not already_loaded,
                "error": "",
            }

    def monitor_idle_serial(self):
        while not self.stop_event.is_set():
            if not self.serial or not self.serial.is_open:
                time.sleep(0.5)
                continue

            if not self.lock.acquire(blocking=False):
                time.sleep(0.05)
                continue

            try:
                while self.serial.in_waiting:
                    line = self._read_line()
                    if line:
                        print(f"<< Pico async: {line}", flush=True)
                        self._handle_async_line(line)
            except Exception as exc:
                print(f"Erro no monitor serial: {exc}", flush=True)
            finally:
                self.lock.release()

            time.sleep(0.05)

    def send_command(self, command):
        command = command.strip().upper()
        if not command:
            return {"ok": False, "error": "Comando vazio", "lines": []}

        if command.startswith("ENSURE_LOADED_INITIAL "):
            return self.ensure_loaded(
                command,
                purge_macro="Purga_Inicial",
                usage="ENSURE_LOADED_INITIAL <tool>",
            )

        if command.startswith("ENSURE_LOADED "):
            return self.ensure_loaded(command)

        with self.lock:
            self.open()
            self.serial.reset_input_buffer()
            self.serial.write((command + "\n").encode("utf-8"))
            self.serial.flush()
            print(f">> Enviado: {command}", flush=True)

            start_time = time.time()
            lines = []
            has_error = False
            expected_done = done_keywords_for(command)

            while time.time() - start_time < COMMAND_TIMEOUT:
                line = self._read_line()
                if not line:
                    continue

                lines.append(line)
                print(f"<< Pico: {line}", flush=True)
                self._handle_async_line(line)

                upper = line.upper()
                for keyword in expected_done:
                    if keyword in upper:
                        has_error = "ERRO" in upper
                        if has_error and should_pause_on_failure(command):
                            self._pause_klipper(line)
                        return {
                            "ok": not has_error,
                            "command": command,
                            "lines": lines,
                            "error": line if has_error else "",
                        }

            if should_pause_on_failure(command):
                self._pause_klipper(f"Timeout no comando {command}")

            return {
                "ok": False,
                "command": command,
                "lines": lines,
                "error": f"TIMEOUT apos {COMMAND_TIMEOUT}s",
            }


chameleon = ChameleonSerial()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        print("HTTP " + fmt % args, flush=True)

    def _send_json(self, status, data):
        body = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/health":
            self._send_json(200, {"ok": True, "serial": SERIAL_PORT, "version": VERSION})
            return

        if parsed.path != "/cmd":
            self._send_json(404, {"ok": False, "error": "Use /cmd?command=HOME"})
            return

        params = urllib.parse.parse_qs(parsed.query)
        command = params.get("command", [""])[0]
        result = chameleon.send_command(command)
        self._send_json(200 if result["ok"] else 500, result)


def main():
    chameleon.open()
    monitor = threading.Thread(target=chameleon.monitor_idle_serial, daemon=True)
    monitor.start()

    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"Chameleon daemon {VERSION} ouvindo em http://{HOST}:{PORT}", flush=True)
    try:
        server.serve_forever()
    finally:
        chameleon.stop_event.set()
        chameleon.close()


if __name__ == "__main__":
    main()
