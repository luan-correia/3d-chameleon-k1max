# Funcionamento

## LOAD

O comando `LOAD` faz:

1. Verifica se o filamento ja esta no hotend.
2. Se nao estiver, gira o motor alimentador no sentido de load.
3. Para quando o sensor do hotend detectar filamento.
4. Anda mais 4 mm para encostar o filamento.
5. Enche o buffer.

A distancia configurada e usada como limite de seguranca. Se o sensor do hotend nao acionar dentro do limite, o Pico para e informa erro.

## UNLOAD

O comando `UNLOAD` faz:

1. Desativa o modo buffer drain, se estiver ativo.
2. Gira o motor alimentador no sentido de unload.
3. Para quando o sensor do hub/saida indicar que o filamento saiu.
4. Retrai mais 40 mm para o filamento nao sair do Camaleao.

A distancia configurada tambem e limite de seguranca. Se o sensor nao mudar, o Pico para e informa erro.

## PRE-LOAD

O pre-load e acionado quando um sensor de entrada muda de sem filamento para com filamento.

Fluxo:

1. Confere se o hub esta livre.
2. Se o hub ja tiver filamento, ignora para evitar colisao.
3. Move o seletor para a ferramenta detectada.
4. Empurra ate o sensor do hub detectar filamento.
5. Retrai 40 mm.

## Buffer

Durante a impressao, o Pico monitora os sensores de buffer. Quando o buffer fica vazio e o hotend ainda tem filamento, ele reabastece ate o sensor de buffer cheio.

## Alertas no ESP32

O display mostra alertas quando:

- o hotend detecta filamento e o hub nao detecta, indicando possivel filamento quebrado ou sensor com defeito
- hub e hotend detectam filamento, indicando que o filamento ja esta no hotend

