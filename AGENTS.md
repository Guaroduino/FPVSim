# Regla de Paridad Estricta de Hardware: ESP32-S3 N16R8 + ILI9341V (320×240 RGB565)

Todo el código desarrollado en este repositorio (tanto en el prototipo web como en código C/C++ de firmware) **DEBE ser 100% trasladable 1 a 1 al hardware real ESP32-S3 N16R8** (Dual-Core XTensa LX7 @ 240 MHz, 512 KB SRAM interna, 8 MB Octal PSRAM, 16 MB Flash) con pantalla **ILI9341V IPS 2.8" SPI DMA (320×240 RGB565) + Touch Capacitivo FT6336**.

## Restricciones Obligatorias en Todo Desarrollo

1. **Presupuesto Estricto de Memoria (SRAM Interna vs. PSRAM Octal)**:
   - **SRAM Interna (`MALLOC_CAP_INTERNAL`, máx ~340 KB para gráficos)**:
     - Reservada exclusivamente para buffers de acceso aleatorio por columna o transferencia DMA:
       - `vram565`: `320 × 240 × 2` bytes (`150 KB`, o buffers ping-pong DMA).
       - `voxBuf`: `256 × 256 × 2` bytes (`128 KB`, buffer de raycasting vertical des-rotado).
       - Tablas LUT críticas de ciclo interno (`rayStepS`, `cloudDistLut`, glifos OSD 2-bit: `< 8 KB`).
   - **PSRAM Octal (`MALLOC_CAP_SPIRAM`, 8 MB disponibles)**:
     - Reservada para mapas de lectura secuencial/espacial: `voxHeight` (`256 KB`), `voxColor` (`512 KB`), `voxSkyMap` (`512 KB`).
   - Prohibido crear objetos dinámicos, arrays temporales o asignaciones de memoria (`new` / `malloc`) dentro del bucle de renderizado por frame.

2. **Prohibido usar Divisiones Flotantes o Funciones Trascendentales en Bucles de Píxeles / Rayos**:
   - Dentro de los bucles por columna (`256` columnas), pasos de rayo (`~96` pasos) o píxeles (`320×240`):
     - **PROHIBIDO** ejecutar `/`, `Math.sin`, `Math.cos`, `Math.sqrt`, `Math.atan2` o `Math.pow` por píxel o por paso de cielo.
     - **OBLIGATORIO** usar tablas precalculadas (LUTs de pasos de rayo `RAY_S_LUT`, `RAY_INV_S_LUT`, `CLOUD_DIST_LUT` indexadas por entero) y aritmética en **Punto Fijo `16.16` (`Int32`)** con desplazamientos `>> 16` y máscaras de potencia de dos (`& 255`, `& 511`, `<< 8`, `<< 9`).

3. **OSD y Fuentes con Sprites Pre-compilados de 2 bits (Estilo Chip MAX7456)**:
   - Prohibido hacer múltiples pasadas o bucles vecinos por frame para generar bordes de texto.
   - Todo glifo del OSD debe pre-compilarse al iniciar en un buffer plano de **2 bits por píxel (`0 = transparente, 1 = borde negro 0x0000, 2 = relleno blanco 0xFFFF`)** para dibujarse en una sola pasada lineal.

4. **Prohibido usar APIs de Canvas 2D para Renderizar el Juego**:
   - Todo el renderizado del simulador debe escribirse directamente sobre el array `vram565` (`uint16_t[320 * 240]` en formato `RGB565`). El Canvas HTML solo actúa como bus SPI DMA (`putImageData`) al final del frame.
