# Configuracao

## Menu do ESP32

O menu do ESP32 permite:

- selecionar ferramentas T0 a T3
- executar load e unload
- fazer home
- iniciar/parar impressao
- calibrar distancia
- ajustar velocidade
- ajustar rampa
- resetar configuracao de calibracao

## Velocidades

O display mostra velocidades em porcentagem.

```text
100% = 70 mm/s
```

As velocidades principais sao:

- `VEL LOAD/UN`: velocidade rapida de load/unload
- `VEL RAMPA`: velocidade da parte final da rampa
- `RAMPA MM`: quantidade de milimetros antes do fim onde a rampa comeca

## Calibracao

A calibracao mede a distancia ate o sensor do hotend. O valor salvo e usado como base para o limite de seguranca dos movimentos.

Exemplo:

```text
calibracao = 560 mm
distancia configurada = 700 mm
```

Nesse caso, o `LOAD` continua parando no sensor do hotend. Os 700 mm sao apenas limite para detectar erro se o filamento travar.

## Reset de calibracao

O menu possui uma opcao para resetar a configuracao de calibracao. Isso invalida a configuracao salva e volta aos valores padrao.

