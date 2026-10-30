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

## 🧠 Arquitectura de Memoria y Paridad (AGENTS.md)

- **SRAM Interna (`MALLOC_CAP_INTERNAL`)**:
  - `vram565`: `320 × 240 × 2 = 153,600 B` (`150 KB` con `MALLOC_CAP_DMA` para transferencia SPI2 DMA a 60 MHz).
  - `voxBuf`: `256 × 256 × 2 = 131,072 B` (`128 KB` para el raycaster vertical des-rotado).
  - `OSD_SMALL_ROM` y tablas LUT en punto fijo `Int32` (`RAY_S_Q8`, `BASE_V_Q8`, `PROJ_SCALE_Q8`, `CLOUD_DIST_Q8`): `< 9 KB`.
- **PSRAM Octal (`MALLOC_CAP_SPIRAM`, 8 MB a 80 MHz)**:
  - `voxHeight`: `2048 × 1024 × 1 = 2.0 MB`.
  - `voxColor`: `2048 × 1024 × 2 = 4.0 MB`.
  - `voxSkyMap`: `512 × 512 × 2 = 0.5 MB`.

## 🛠️ Modo de Prueba de Soldadura y Hardware (`SELECT`)

Una vez encendida la consola:
- Pulsa el botón **`SELECT` (`GPIO 41`)** para mostrar u ocultar en pantalla el **Inspector de Diagnóstico Hardware en Tiempo Real**, donde podrás verificar:
  - Los 4 canales analógicos de 12 bits en `ADC1` (`L: 0..4095`, `R: 0..4095`).
  - El estado `0/1` de los 12 botones físicos (`UP`, `DOWN`, `LEFT`, `RIGHT`, `A`, `B`, `X`, `Y`, `L3`, `R3`, `SELECT`, `MENU`).
  - Las coordenadas `(X, Y)` en vivo del panel táctil capacitivo `FT6336U`.
  - La memoria libre en `SRAM` y `PSRAM Octal` junto con los FPS reales por DMA.
