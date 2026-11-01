# Regla de Paridad Estricta de Hardware: ESP32-S3 N16R8 + ILI9341V (320×240 RGB565)

Todo el código desarrollado en este repositorio (tanto en el prototipo web como en código C/C++ de firmware) **DEBE ser 100% trasladable 1 a 1 al hardware real ESP32-S3 N16R8** (Dual-Core XTensa LX7 @ 240 MHz, 512 KB SRAM interna, 8 MB Octal PSRAM, 16 MB Flash) con pantalla **ILI9341V IPS 2.8" SPI DMA (320×240 RGB565) + Touch Capacitivo FT6336**.

## Restricciones Obligatorias en Todo Desarrollo

1. **Presupuesto Estricto de Memoria (SRAM Interna vs. PSRAM Octal)** — objetivo **48 FPS (SPI 60 MHz, 20.8 ms/frame)**:
   - **SRAM Interna (`MALLOC_CAP_INTERNAL`, ~320 KB libres sin Wi-Fi)**:
     - `voxBuf`: `256 × 256 × 2` bytes (`128 KB`, buffer de raycasting vertical des-rotado).
     - `voxDepth8`: `256 × 256 × 1` (`64 KB`) y `vramDepth8`: `320 × 240 × 1` (`75 KB`): Z-Buffers de **8 bits** con código pseudo-logarítmico (`depthCode8`). Prohibido volver a Z-Buffers de 16 bits (sumaban 278 KB y no caben).
     - Bandas DMA ping-pong `2 × 320 × 40 × 2` bytes (`50 KB`) y LUTs críticas (`< 16 KB`).
     - Con Wi-Fi/ESP-NOW activo (40–60 KB) mover `vramDepth8` a PSRAM.
   - **PSRAM Octal (`MALLOC_CAP_SPIRAM`, 8 MB disponibles)**:
     - **Doble framebuffer** `2 × 320 × 240 × 2` (`300 KB`): el Core 1 dibuja uno mientras el Core 0 envía el otro por DMA en bandas.
     - Mapas en bloques tileados 8×8: `voxMap` (`2 MB`, texel 16 bit = altura + índice de paleta), `voxSkyMap` (`512 KB`). Paleta `voxPal[256]` + `FOG_LUT[24][256]` en SRAM.
   - Prohibido crear objetos dinámicos, arrays temporales o asignaciones de memoria (`new` / `malloc`) dentro del bucle de renderizado por frame.
   - La física corre a **paso fijo de 1/240 s** con acumulador (`stepFixedPhysics` en web, `updateDronePhysics` en firmware). Prohibido integrar con el `dt` variable del frame.
   - El simulador muestra el costo estimado en el S3 en la píldora del modo SPI (`S3 est.`); si se pone en rojo, el cambio no cabe en el presupuesto.

2. **Prohibido usar Divisiones Flotantes o Funciones Trascendentales en Bucles de Píxeles / Rayos**:
   - Dentro de los bucles por columna (`256` columnas), pasos de rayo (`~96` pasos) o píxeles (`320×240`):
     - **PROHIBIDO** ejecutar `/`, `Math.sin`, `Math.cos`, `Math.sqrt`, `Math.atan2` o `Math.pow` por píxel o por paso de cielo.
     - **OBLIGATORIO** usar tablas precalculadas (LUTs de pasos de rayo `RAY_S_LUT`, `RAY_INV_S_LUT`, `CLOUD_DIST_LUT` indexadas por entero) y aritmética en **Punto Fijo `16.16` (`Int32`)** con desplazamientos `>> 16` y máscaras de potencia de dos (`& 255`, `& 511`, `<< 8`, `<< 9`).

3. **OSD y Fuentes con Sprites Pre-compilados de 2 bits (Estilo Chip MAX7456)**:
   - Prohibido hacer múltiples pasadas o bucles vecinos por frame para generar bordes de texto.
   - Todo glifo del OSD debe pre-compilarse al iniciar en un buffer plano de **2 bits por píxel (`0 = transparente, 1 = borde negro 0x0000, 2 = relleno blanco 0xFFFF`)** para dibujarse en una sola pasada lineal.

4. **Prohibido usar APIs de Canvas 2D para Renderizar el Juego**:
   - Todo el renderizado del simulador debe escribirse directamente sobre el array `vram565` (`uint16_t[320 * 240]` en formato `RGB565`). El Canvas HTML solo actúa como bus SPI DMA (`putImageData`) al final del frame.
