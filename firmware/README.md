# Firmware Nativo ESP32-S3 N16R8 — FPVSim (ILI9341V IPS 320×240 DMA + FT6336 Touch)

Proyecto de **PlatformIO** listo para compilar y flashear directamente en tu consola portátil con **ESP32-S3-DevKitC-1-N16R8** (16 MB Flash Quad + **8 MB PSRAM Octal**).

## 🚀 Compilar y Flashear

Desde la carpeta `firmware/`:

```bash
# 1. Compilar el firmware
pio run

# 2. Subir a la placa ESP32-S3 N16R8 por USB-C
pio run -t upload

# 3. Subir los mapas .fpvm a la partición LittleFS (opcional, ver abajo)
pio run -t uploadfs

# 4. Abrir monitor serie (opcional, 115200 baudios)
pio device monitor
```

La tabla de particiones `partitions_fpvsim.csv` deja 4 MB para la app y ~12.4 MB de LittleFS (caben 2 mapas de 4 MB).

## 🧠 Arquitectura de Render y Memoria (objetivo 48 FPS con SPI a 60 MHz)

- **Doble framebuffer en PSRAM** (`fbPsram[2]`, 2 × 150 KB): el Core 1 dibuja el frame N+1 mientras la tarea de pantalla (Core 0) envía el frame N.
- **Envío por bandas DMA**: la tarea de pantalla copia el frame de PSRAM a 2 bandas de 320×40 en SRAM (`dmaBand[2]`, 2 × 25 KB) invirtiendo los bytes (el ILI9341 espera RGB565 big-endian) y las manda por DMA en ping-pong. La copia de una banda se solapa con la transmisión de la anterior.
- **Raycast en los dos núcleos**: columnas pares en el Core 1, impares en el Core 0 (`RAY_DUAL_CORE`, compílalo con `-DRAY_DUAL_CORE=0` para comparar).
- **Física a paso fijo 240 Hz** con acumulador: el vuelo se comporta igual a cualquier fps (igual que el simulador web).
- **Mapas PSRAM en bloques tileados 8×8** (`voxTileIdx`), mismo formato que el simulador web.
- **SRAM interna**: `voxBuf` 128 KB + bandas DMA 50 KB + LUTs. **PSRAM**: framebuffers 300 KB + `voxMap` 2 MB (texel 16 bit = altura 8 bit + índice de paleta 8 bit) + `voxSkyMap` 0.5 MB. Paleta `voxPal[256]` y niebla `FOG_LUT[24][256]` (técnica colormap) en SRAM.

## 🗺️ Mapas exportados desde el simulador

1. En el simulador web abre el mapa que quieras y pulsa **⬇ MAPA**: descarga un archivo `.fpvm` (4 MB, paleta de 256 colores por median-cut, texels tileados 8×8).
2. Copia el archivo a `firmware/data/maps/` y ejecuta `pio run -t uploadfs`.
3. En la consola, el botón **`MENU`** recorre los mapas de LittleFS y termina en el mapa procedural integrado.

## 🔊 Audio I2S (MAX98357A)

`BCLK = GPIO 43`, `LRCK = GPIO 44`, `DIN = GPIO 13`, 22.05 kHz mono. La síntesis del motor replica la del simulador web (diente de sierra + cuadrada + ruido filtrado) y suena un pitido al cruzar cada gate. Corre en una tarea del Core 0. Desactívalo con `-DENABLE_AUDIO=0`.

## 📡 Multijugador ESP-NOW

Cada consola difunde su posición ~30 veces por segundo (canal 1, paquete de 40 bytes) y dibuja hasta 4 rivales como un rombo magenta con su distancia. Solo se ven consolas que usan el mismo mapa (CRC del nombre). Desactívalo con `-DENABLE_ESPNOW=0` si necesitas la SRAM que usa el Wi-Fi.

## ⏱️ Medición por Etapa

Pulsa **`SELECT`** para ver en pantalla los tiempos del último frame: raycast de cada núcleo, blit, OSD, entradas, física, espera del bus SPI y duración del DMA. Los mismos datos salen cada 2 s por el monitor serie (`[FRAME] ...`). Si `ESPERA SPI` es alta, el límite es el bus de la pantalla; si `RAY`/`BLIT` suman más de ~20 ms, el límite es el CPU.

## 🛠️ Modo de Prueba de Soldadura y Hardware (`SELECT`)

Una vez encendida la consola:
- Pulsa el botón **`SELECT` (`GPIO 41`)** para mostrar u ocultar en pantalla el **Inspector de Diagnóstico Hardware en Tiempo Real**, donde podrás verificar:
  - Los 4 canales analógicos de 12 bits en `ADC1` (`L: 0..4095`, `R: 0..4095`).
  - El estado `0/1` de los 12 botones físicos (`UP`, `DOWN`, `LEFT`, `RIGHT`, `A`, `B`, `X`, `Y`, `L3`, `R3`, `SELECT`, `MENU`).
  - Las coordenadas `(X, Y)` en vivo del panel táctil capacitivo `FT6336U`.
  - La memoria libre en `SRAM` y `PSRAM Octal` junto con los FPS reales por DMA.
