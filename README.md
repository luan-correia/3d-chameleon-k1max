# 3D Camaleao - 3D Chameleon modificado

Projeto de controle para um sistema tipo 3D Chameleon com:

- Raspberry Pi Pico controlando motores, sensores e comunicacao com a K1/Klipper
- ESP32 com display ST7789 atualizado, encoder e menu de configuracao
- 2 drivers TMC2209 em modo standalone
- sensores de filamento no hub, hotend, buffer e entradas T0 a T3
- arquivos Klipper e modelos STL do conjunto mecanico

Este repositorio esta organizado para documentar o hardware, os firmwares e os arquivos auxiliares usados no projeto.

## Instalacao automatica no Klipper

O instalador configura o daemon serial, as macros e o painel dedicado **Camaleao** no Mainsail. Ele detecta automaticamente Creality K1/Buildroot e instalacoes Linux comuns do Klipper.

```bash
git clone https://github.com/luan-correia/3d-chameleon-k1max.git
cd 3d-chameleon-k1max
sudo ./installer/install.sh
```

Depois da instalacao, reinicie o Klipper e atualize o navegador com `Ctrl+F5`. O instalador sempre cria backup antes de alterar arquivos.

Comandos de manutencao:

```bash
sudo ./installer/update.sh
sudo ./installer/diagnose.sh
sudo ./installer/uninstall.sh
```

Consulte [docs/instalador.md](docs/instalador.md) para requisitos, variaveis e recuperacao.

## Atualizacao do display

A versao atual substitui o display simples antigo por um painel com **ESP32 + ST7789 320x240**.

Antes, o controle visual ficava mais limitado. Agora o ESP32 faz a interface completa e conversa com o Pico por I2C. O Pico continua sendo responsavel pela logica principal dos motores e sensores.

O painel novo no ESP32 inclui:

- tela inicial com status das ferramentas T0 a T3
- status de hotend, buffer, impressao e comunicacao com o Pico
- menu por encoder
- load, unload, home, start e stop pelo display
- menu de configuracao
- calibracao e reset da calibracao
- ajuste de velocidade em porcentagem
- secador de filamento opcional, com controle de temperatura e umidade
- alertas visuais de filamento no hotend, filamento quebrado ou sensor com defeito
- logo `3DC CAMALEAO - VERSAO BRASIL`

Com essa mudanca, o projeto fica dividido assim:

```text
Pico  -> motores, sensores, comandos da K1/Klipper e seguranca
ESP32 -> display ST7789, encoder, menus, alertas e configuracao visual
```

## Estrutura

```text
firmware/
  esp32_display/
    platformio.ini
    src/ESP32_3D_Chameleon_ST7789_I2C_V16_GOTA_UMIDADE.ino
  pico/
    Pico_3D_Chameleon_I2C_COMPLETO.ino

docs/
  pinout.md
  funcionamento.md
  configuracao.md

camaleao e base/
  arquivos STL da base, hub e suporte

rebobinador/
  arquivos STL do rebobinador

klipper upgrade comunicao python e macros para camaleoa/
  macros Klipper e daemon Python

installer/
  instalacao, atualizacao, diagnostico e desinstalacao
```

## Estado atual do firmware

### Pico

O Pico e o controlador principal do sistema. Ele cuida de:

- motor seletor
- motor alimentador/extrusor
- sensores T0, T1, T2 e T3
- sensor unico do hub/saida
- sensor hotend
- sensores de buffer vazio e cheio
- endstop do seletor
- comandos USB serial vindos da K1/Klipper
- comunicacao I2C com o ESP32

### ESP32

O ESP32 cuida da interface:

- display ST7789 320x240
- encoder
- menu principal
- menu de configuracao
- alertas visuais
- envio de comandos para o Pico por I2C

O ESP32 nao movimenta os motores diretamente. Ele envia comandos para o Pico, e o Pico executa a acao com os sensores de seguranca.

## Aquecedor/secador opcional

O aquecedor e um recurso **opcional**. O 3D Camaleao funciona normalmente sem montar o secador; ele nao e necessario para selecionar, carregar ou descarregar filamento.

Quem quiser usar o recurso pode ligar ao ESP32:

| Funcao | Pino do ESP32 |
|--------|---------------|
| Sensor de temperatura NTC | GPIO34 |
| Controle do aquecedor | GPIO26 |
| Controle do ventilador | GPIO14 |

O menu `SECAGEM FILAMENTO` permite definir temperatura e umidade. O firmware so aciona o aquecedor quando o NTC e identificado corretamente, desliga o aquecimento ao atingir o limite configurado e mantem o ventilador ligado durante o resfriamento ate a temperatura ficar abaixo de 30 C.

> **Atencao:** nao ligue o aquecedor diretamente ao ESP32. Use um modulo MOSFET adequado a tensao e corrente do aquecedor, fonte dimensionada, fusivel e protecao termica. Todos os GND devem estar em comum.

## Logica principal

- `LOAD`: carrega ate o sensor do hotend detectar filamento, depois anda mais 4 mm e enche o buffer.
- `UNLOAD`: descarrega ate o sensor do hub/saida indicar que o filamento saiu, usando a distancia configurada como limite de seguranca.
- A distancia de load/unload serve como protecao: se o filamento travar e o sensor nao mudar, o Pico para e informa erro.
- O buffer e mantido pela logica de reabastecimento durante impressao.
- O pre-load aciona quando um filamento entra no sensor de entrada, vai ate o sensor do hub e retrai 40 mm.

## Microsteps e distancia

O firmware atual assume:

```cpp
MICROSTEPS = 16
STEPS_PER_REV = 200
STEPS_PER_MM = 428.0
```

Os drivers TMC2209 estao em modo standalone. Por isso, o microstep real depende dos DIP switches/jumpers da placa do driver.

No teste atual:

- seletor funcionou com `1 2 3 ON`
- extrusor funcionou melhor com `1 ON`, `2 OFF`, `3 ON`

Se mudar a configuracao de microstep, pode ser necessario recalibrar ou ajustar `STEPS_PER_MM`.

## Como compilar o ESP32

O firmware do ESP32 usa PlatformIO.

```powershell
cd firmware\esp32_display
pio run
pio run -t upload
```

No projeto original, o upload estava usando `COM3`.

O arquivo principal do display atualizado fica em:

```text
firmware/esp32_display/src/ESP32_3D_Chameleon_ST7789_I2C_V16_GOTA_UMIDADE.ino
```

## Como gravar o Pico

Abra o arquivo do Pico na Arduino IDE ou ambiente equivalente com suporte ao Raspberry Pi Pico:

```text
firmware/pico/Pico_3D_Chameleon_I2C_COMPLETO.ino
```

Grave no Raspberry Pi Pico conectado por USB.

## Lista de materiais

| Quantidade | Item |
|------------|------|
| 9 | Micro switch KW10-B |
| 1 | Raspberry Pi Pico |
| 1 | ESP32 |
| 1 | Encoder |
| 1 | Fonte 12 V ou 24 V 5 A |
| 1 | Step-down para 5 V 3 A |
| 2 | Suporte para driver |
| 2 | Driver TMC2209 ou A4988 |
| 1 | Display ST7789 240x320 |
| 4 | Parafuso M3 cabeca chata 10 mm |
| 8 | Parafuso M3 cabeca chata 15 mm com porcas |
| 4 | Parafuso M3 cabeca cilindrica 35 mm |
| 20 | Parafuso M3 cabeca cilindrica 6 mm |
| 8 | Parafuso M3 cabeca cilindrica 12 mm |
| 4 | Conector PTFE |
| 6 | Esfera de 4 mm |
| 2 m | Tubo PTFE 4 mm x 2.5 mm |
| 1 | Caixa acrilica 39 x 29 x 25 cm |
| 1 | Tomada interruptor AS-03 tripolar macho com chave gangorra |
| 1 | Porta USB tipo C |
| 4 | Molas: 8 mm de comprimento, 4 mm externo, arame de 1 mm ou 0.7 mm |
| 50 | Bucha de inserto metalico M3 x 4 mm |

Para o secador opcional, acrescente um sensor NTC, um aquecedor, um ventilador e um modulo MOSFET compativel com a carga utilizada.

## Observacoes importantes

- Todos os GND precisam estar em comum: Pico, ESP32, drivers e fonte dos motores.
- Os sinais do Pico sao 3.3 V.
- Nao conecte ou desconecte motor com o driver energizado.
- Ajuste a corrente dos TMC2209 antes de testar com filamento.
- Se o motor so vibrar, confira pares das bobinas, corrente do driver e DIP switches.

## Creditos e agradecimentos

Este projeto e uma adaptacao/modificacao feita para uso na Creality K1 Max, baseada na ideia do 3D Chameleon.

Agradecimentos aos projetos e comunidades que ajudaram como referencia:

- 3D Chameleon original: https://www.3dchameleon.com/
- Filamentalist Rewinder / ERCF: https://github.com/SkiBikePrint/Filamentalist
- ERCF v2: https://github.com/Carrot-collective/ERCF_v2

## Apoie o projeto

Se este projeto te ajudou e voce quiser apoiar para ele crescer, qualquer contribuicao ajuda nos testes, pecas, placas e melhorias.

PIX:

```text
47996781505
```

PayPal:

```text
luan.lec94@gmail.com
```
