# Simuladores 3D Standalone en Lua para RadioMaster TX16S (EdgeTX ColorLCD)

Port 1:1 de los 3 simuladores de `index.html` a scripts independientes de **EdgeTX Tools** (`.lua`) diseñados para correr directamente en la pantalla IPS `480×272` de la **RadioMaster TX16S / TX16S MKII** (y radios ColorLCD compatibles como Boxer Color, Horus X10/X12S) a **~30 FPS** aprovechando el acelerador hardware **STM32 DMA2D** (`lcd.drawFilledTriangle`, `lcd.drawFilledRectangle`, `lcd.drawLine`).

---

## Archivos Incluidos

Copia la carpeta `SCRIPTS` directamente en la raíz de la tarjeta SD de tu emisora TX16S:

| Script | Ubicación en la SD | Nombre en menú Tools | Descripción |
| :--- | :--- | :--- | :--- |
| [`fpv_wing.lua`](./SCRIPTS/TOOLS/fpv_wing.lua) | `/SCRIPTS/TOOLS/fpv_wing.lua` | **FPV Fixed-Wing** | Simulador 6-DOF de Ala Fija con aerodinámica por momentos (`Lp, Ld, Lb, Ma, Md, Mq, Nr, Nb`), TPA de INAV (`1/Va²`), mezcla elevon, pérdida asimétrica de punta de ala (tip-stall), batería LiPo 3S 2200mAh con caída de tensión (`sag`), aterrizaje de panza (`mu=0.45`), 6 aviones (`DART 250G`, `Z-84`, `AR WING LR`, `BIXLER`, `TALON`, `EDGE 540`) y circuitos Chorobobo (`C1..C3` + Vuelo Libre). |
| [`fpv_drone.lua`](./SCRIPTS/TOOLS/fpv_drone.lua) | `/SCRIPTS/TOOLS/fpv_drone.lua` | **FPV Drone 5"** | Simulador 6-DOF de Dron FPV 5" Quad-X con lazo PID Betaflight (P + I + D sobre giroscopio + FeedForward + I-Term Relax), 4 motores independientes (`mRpm[1..4]`), AirMode, turbulencia de propwash, batería LiPo 6S/4S con sag, desarme por impacto + **Modo Tortuga (Flip Over After Crash)** y 4 presets (`FREESTYLE`, `RACING`, `CINEMATIC`, `BANDO 6S`). |
| [`rc_sail.lua`](./SCRIPTS/TOOLS/rc_sail.lua) | `/SCRIPTS/TOOLS/rc_sail.lua` | **RC Sailboat** | Simulador LOS de Velero RC (`IOM 1M PRO`, `DF65 AGILE`, `RG65 CARBON`) con modelo aerodinámico polar de Mayor y Foque independientes, embolsamiento dinámico a sotavento (`camber`), hidrodinámica de orza con bulbo y timón con pérdida, rachas visibles en el agua (`Catspaws`), 5 circuitos de regata, 2 rivales IA (`AI1`, `AI2`), vectores de fuerza 3D y cámaras `LOS Auto-Zoom`, `Zoom Manual` y `Follow 3D`. |

---

## Instalación en la RadioMaster TX16S

1. Conecta tu **TX16S** por USB en modo **SD Card Storage** (o extrae la tarjeta microSD).
2. Copia los tres archivos `.lua` dentro de la carpeta `/SCRIPTS/TOOLS/` de tu tarjeta SD.
   Requiere **EdgeTX 2.8 o posterior** (`lcd.drawFilledTriangle`).
3. Enciende la emisora, presiona el botón **`SYS`** para abrir el menú **Tools** y selecciona cualquiera de los 3 simuladores:
   - **FPV Fixed-Wing**
   - **FPV Drone 5"**
   - **RC Sailboat**

---

## Controles en la Emisora

### 1. FPV Fixed-Wing (`fpv_wing.lua`) y FPV Drone 5" (`fpv_drone.lua`)
- **Sticks Gimbals (`ail`, `ele`, `thr`, `rud`)**: Control en tiempo real desde los gimbals físicos de la radio (independiente del modo de sticks configurado en EdgeTX).
- **Rueda Roller (`ROTARY`) / Toque en esquina superior izquierda**: Abre el menú de configuración rápida:
  - **Modo de Vuelo**: `ACRO` / `STAB` (Horizon/Angle) / `RTH` (o **Modo Tortuga** en el Dron cuando está invertido tras un choque).
  - **Preset de Aeronave**: Cambia entre los 6 aviones o 4 drones.
  - **Circuito**: `C1 Ovalo`, `C2 Ocho`, `C3 Tecnico`, `Vuelo Libre`.
  - **Vista**: `FPV Cockpit` (1ª persona con OSD estilo INAV/Betaflight) o `LOS Torre` (3ª persona desde pista con Auto-Zoom).
  - **Reiniciar Vuelo (`RESET`)**: Reposiciona en cabecera de pista con batería al 100%.
- **Botón `RTN`**: Sale del simulador.

### 2. RC Sailboat (`rc_sail.lua`)
- **Stick `AIL` o `RUD` (Izquierda/Derecha)**: Timón (`Rudder`) con física de pérdida hidrodinámica si se fuerza en exceso a baja velocidad.
- **Stick `THR` (Abajo/Arriba)**: Winche de Escota (`Sheet Winch`) para cazar (`abajo`) o largar (`arriba`) la Mayor y el Foque simultáneamente. Incluye indicador de marca blanca óptima en la barra de escota.
- **Stick `ELE` (Arriba/Abajo)**: Controla el Zoom en tiempo real cuando la cámara está en modo `2. LOS ZOOM MANUAL`.
- **Botón `ENT` (Pulsar Rueda)**: Alterna rápidamente entre las 3 cámaras (`CAM:AUTO` desde el muelle, `CAM:MAN` y `CAM:3D` de persecución).
- **Botón `SYS` / `TELE` o toque en la barra superior**: Abre el menú para elegir barco (`IOM 1M PRO`, `DF65 AGILE`, `RG65 CARBON`), cualquiera de los **5 circuitos de regata**, intensidad del viento base, vectores de fuerza 3D. Dentro del menú, `RTN` lo cierra.
- **Botón `RTN` (fuera del menú)**: Sale del simulador.

---

## Notas técnicas

- **Resolución fija 480×272**: con la API Lua cada triángulo se rellena a resolución nativa, así que renderizar a 240×136 y duplicar coordenadas no ahorraba tiempo; solo hacía la imagen más tosca.
- **Recorte 3D**: pista, gates, edificios y la orilla del velero se recortan contra el plano cercano de la cámara (Sutherland-Hodgman / Liang-Barsky) en lugar de descartarse enteros cuando un vértice queda detrás de la cámara.
- **Física a paso fijo**: acumulador con `PHYS_DT` (dron y ala 1/120 s, velero 1/60 s), así el vuelo no cambia con los fps ni con la resolución de 10 ms de `getTime()`.
