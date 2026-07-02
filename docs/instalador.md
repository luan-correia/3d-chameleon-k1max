# Instalador do painel Camaleao

O instalador adiciona uma placa exclusiva do 3D Camaleao ao dashboard do Mainsail e instala a comunicacao Python com o Pico.

## Sistemas suportados

- Creality K1, K1 Max e variantes com root, Moonraker e Mainsail
- Raspberry Pi e outros computadores Linux com Klipper, Moonraker, Mainsail, nginx e systemd

## Requisitos

- Pico gravado com o firmware deste repositorio
- Pico conectado por USB ao computador que executa o Klipper
- Python 3
- `unzip`
- acesso root ou `sudo`
- `gcode_shell_command.py` para usar as macros Klipper

O painel Mainsail funciona diretamente com o daemon. Caso `gcode_shell_command.py` nao esteja instalado, o diagnostico avisa e o painel ainda pode ser usado manualmente.

## Instalacao

```bash
git clone https://github.com/luan-correia/3d-chameleon-k1max.git
cd 3d-chameleon-k1max
sudo ./installer/install.sh
```

O processo:

1. Detecta K1/Buildroot ou Linux comum.
2. Localiza Mainsail, nginx e `printer.cfg`.
3. Detecta a porta do Pico.
4. Cria backup com data e hora.
5. Instala daemon, cliente e macros.
6. Instala o Mainsail personalizado em uma pasta separada.
7. Valida o nginx antes de ativar a configuracao.
8. Testa `/health` e o comando `STATUS` do Pico.

## Caminhos na K1

- Configuracao: `/usr/data/printer_data/config`
- Daemon: `/usr/data/printer_data/config/chameleon`
- Painel: `/usr/data/mainsail-chameleon`
- Estado: `/usr/data/chameleon_installer/state.env`
- Backups: `/usr/data/chameleon_backups/`

## Caminhos no Linux comum

- Daemon: `/opt/chameleon`
- Painel: `/opt/mainsail-chameleon`
- Servico: `chameleon-daemon.service`
- Estado: `/var/lib/chameleon-installer/state.env`
- Backups: `/var/backups/chameleon/`

## Configuracao manual de caminhos

Se a deteccao automatica nao encontrar algum componente, informe antes do comando:

```bash
sudo CHAMELEON_SERIAL_PORT=/dev/ttyACM0 \
  CHAMELEON_CONFIG_DIR=/home/pi/printer_data/config \
  CHAMELEON_MAINSAIL_DIR=/home/pi/mainsail \
  CHAMELEON_NGINX_CONF=/etc/nginx/sites-enabled/mainsail \
  ./installer/install.sh
```

## Diagnostico

```bash
sudo ./installer/diagnose.sh
```

Ele mostra porta serial, respostas do daemon, `STATUS` do Pico, processos e caminhos instalados.

## Atualizacao

```bash
sudo ./installer/update.sh
```

O atualizador busca o repositorio e executa novamente a instalacao, criando outro backup.

## Desinstalacao

```bash
sudo ./installer/uninstall.sh
```

O desinstalador restaura o nginx salvo, remove painel, daemon e include das macros. Os backups permanecem guardados.

## Codigo-fonte do painel

O painel foi criado sobre o Mainsail `v2.17.0`. O patch utilizado para gerar o pacote esta em `installer/mainsail-src/mainsail-v2.17.0-chameleon.patch`.
