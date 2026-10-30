# FPVSim 128×64 — Simulador FPV 3D Monocromático (Web a ESP32-S3)

Simulador de vuelo FPV en modo **Acro** con estética vectorial wireframe monocromática de **1 bit (128×64 píxeles, framebuffer de 1.024 bytes)**, diseñado como banco de pruebas interactivo y arquitectura base portable a C/C++ para una consola portátil basada en **ESP32-S3** con pantalla OLED (SSD1306 / SH1106) y sticks analógicos autocentrados.

## 🌐 Demo Pública (Smartphone / PC) y Guía de Hardware
- 🕹️ **Simulador Web Interactivo**: **[https://guaroduino.github.io/FPVSim/](https://guaroduino.github.io/FPVSim/)**
- 🔌 **Guía de Construcción y Pinout Exacto (ESP32-S3 N16R8 + IPS 2.8")**: **[hardware.html](hardware.html)** (`https://guaroduino.github.io/FPVSim/hardware.html`)

---

## ✨ Características (Hitos 1 y 2 Completados)

- **Framebuffer Estático de 1 KB (`Uint8Array(1024)`)**: Renderizado 1-bit por píxel idéntico al buffer DMA de una pantalla OLED SSD1306/SH1106 de 128×64.
- **Rasterizador Entero Bresenham + Recorte Cohen-Sutherland**: Trazado puro de líneas sin antialiasing y recorte en plano cercano 3D (`Z > 0.15`).
- **Física Acro con Cuaterniones (Sin Gimbal Lock)**:
  - Integración de tasas angulares a ~240 Hz (4 sub-pasos por frame).
  - Empuje alineado al eje `+Y` normal del chasis, gravedad configurable (`-9.81 m/s²`) y drag aerodinámico lineal/cuadrático.
  - Mapeo de acelerador adaptado a **joysticks autocentrados** (centro neutral = *Hover ~40%*).
  - Inclinación de cámara FPV (*Camera Uptilt* de `0°` a `45°`).
- **Circuito de 8 Puertas y Cronómetro de Vueltas**:
  - Detección geométrica de cruce de plano por puerta con indicador direccional en el OSD y resaltado de doble marco en la puerta objetivo.
- **Soporte Multi-Control**:
  - **Smartphone / Tablet**: Joysticks virtuales multitouch con retorno por muelle al centro.
  - **Gamepad Físico (USB / Bluetooth)**: API nativa de Gamepad en Modo 2.
  - **Teclado**: `W/S` (Throttle), `A/D` (Yaw), Flechas (Pitch/Roll), `R` (Reset), `M` (Alternar Acro / Auto-Level).
- **Panel de Calibración en Tiempo Real**: Sliders en vivo para ajustar *Camera Uptilt*, *Hover %*, *Empuje*, *Gravedad*, *Drag*, *Rates Pitch/Roll/Yaw*, *Curva Expo* y *Deadzone*.
