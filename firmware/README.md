# Firmware Nativo ESP32-S3 N16R8 — FPVSim (ILI9341V IPS 320×240 DMA + FT6336 Touch)

Proyecto de **PlatformIO** listo para compilar y flashear directamente en tu consola portátil con **ESP32-S3-DevKitC-1-N16R8** (16 MB Flash Quad + **8 MB PSRAM Octal**).

## 🚀 Compilar y Flashear

Desde la carpeta `firmware/`:

```bash
# 1. Compilar el firmware
pio run

# 2. Subir a la placa ESP32-S3 N16R8 por USB-C
pio run -t upload

# 3. Abrir monitor serie (opcional, 115200 baudios)
pio device monitor
```

## 🧠 Arquitectura de Render y Memoria (objetivo 48 FPS con SPI a 60 MHz)

- **Doble framebuffer en PSRAM** (`fbPsram[2]`, 2 × 150 KB): el Core 1 dibuja el frame N+1 mientras la tarea de pantalla (Core 0) envía el frame N.
- **Envío por bandas DMA**: la tarea de pantalla copia el frame de PSRAM a 2 bandas de 320×40 en SRAM (`dmaBand[2]`, 2 × 25 KB) invirtiendo los bytes (el ILI9341 espera RGB565 big-endian) y las manda por DMA en ping-pong. La copia de una banda se solapa con la transmisión de la anterior.
- **Raycast en los dos núcleos**: columnas pares en el Core 1, impares en el Core 0 (`RAY_DUAL_CORE`, compílalo con `-DRAY_DUAL_CORE=0` para comparar).
- **Física a paso fijo 240 Hz** con acumulador: el vuelo se comporta igual a cualquier fps (igual que el simulador web).
- **Mapas PSRAM en bloques tileados 8×8** (`voxTileIdx`), mismo formato que el simulador web.
- **SRAM interna**: `voxBuf` 128 KB + bandas DMA 50 KB + LUTs. **PSRAM**: framebuffers 300 KB + `voxHeight` 2 MB + `voxColor` 4 MB + `voxSkyMap` 0.5 MB.

## ⏱️ Medición por Etapa

Pulsa **`SELECT`** para ver en pantalla los tiempos del último frame: raycast de cada núcleo, blit, OSD, entradas, física, espera del bus SPI y duración del DMA. Los mismos datos salen cada 2 s por el monitor serie (`[FRAME] ...`). Si `ESPERA SPI` es alta, el límite es el bus de la pantalla; si `RAY`/`BLIT` suman más de ~20 ms, el límite es el CPU.

## 🛠️ Modo de Prueba de Soldadura y Hardware (`SELECT`)

Una vez encendida la consola:
- Pulsa el botón **`SELECT` (`GPIO 41`)** para mostrar u ocultar en pantalla el **Inspector de Diagnóstico Hardware en Tiempo Real**, donde podrás verificar:
  - Los 4 canales analógicos de 12 bits en `ADC1` (`L: 0..4095`, `R: 0..4095`).
  - El estado `0/1` de los 12 botones físicos (`UP`, `DOWN`, `LEFT`, `RIGHT`, `A`, `B`, `X`, `Y`, `L3`, `R3`, `SELECT`, `MENU`).
  - Las coordenadas `(X, Y)` en vivo del panel táctil capacitivo `FT6336U`.
  - La memoria libre en `SRAM` y `PSRAM Octal` junto con los FPS reales por DMA.
