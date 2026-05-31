# Pinout

## Pico

### Sensores de entrada

```text
T0 entrada : GPIO 0
T1 entrada : GPIO 1
T2 entrada : GPIO 2
T3 entrada : GPIO 3
```

### I2C para ESP32

```text
SDA : GPIO 4
SCL : GPIO 5
Endereco I2C: 0x12
```

### Sensores principais

```text
Endstop seletor : GPIO 8
Sensor hotend   : GPIO 9
Buffer vazio    : GPIO 10
Buffer cheio    : GPIO 11
Sensor hub      : GPIO 12
```

### Motor seletor

```text
STEP : GPIO 16
DIR  : GPIO 17
EN   : GPIO 18
```

### Motor alimentador/extrusor

```text
STEP : GPIO 19
DIR  : GPIO 20
EN   : GPIO 21
```

## ESP32

### Display ST7789

```text
TFT_CS   : GPIO 15
TFT_DC   : GPIO 2
TFT_RST  : GPIO 4
TFT_MOSI : GPIO 23
TFT_SCLK : GPIO 18
```

### I2C para Pico

```text
SDA : GPIO 21
SCL : GPIO 22
```

### Encoder

```text
CLK : GPIO 32
DT  : GPIO 33
SW  : GPIO 25
```

### DHT22

```text
DATA : GPIO 27
```

